#!/usr/bin/env python3
"""Run the whole test suite on Modal, fanned out across N containers.

    modal run tests/stress/modal_suite/run_all_modal.py                # 50 shards
    modal run tests/stress/modal_suite/run_all_modal.py --shards 25
    modal run tests/stress/modal_suite/run_all_modal.py --fast         # unit only
    modal run tests/stress/modal_suite/run_all_modal.py --filters 908  # one family

`tests/run_all.py --shard I/N` does the splitting, so DISCOVERY AND
CLASSIFICATION HAVE ONE SOURCE OF TRUTH -- this driver never globs `test_*.py`
itself. A local `python3 tests/run_all.py` and a 50-way cloud fan-out therefore
run the same set; if they ever disagree, the bug is in one place.

The image is the sweep's pinned clean checkout of HEAD (`_image_source`), so a
run is reproducible and you can keep editing locally while it runs. Set
KICAD_SWEEP_DIRTY=1 to ship the WORKING TREE instead -- for a suite run over an
uncommitted change. It warns, and the provenance printed in the report is
suffixed `+dirty`.

Why this exists: the suite is ~594 files and ~40 min of laptop (and of
battery). Fifty containers turn it into the length of its slowest shard.

WHAT THE EXIT CODE MEANS. Each shard's OWN exit code decides that shard, and
the driver is red if ANY shard is red or MISSING. The parsed counts are for the
report only -- a container that OOMs, or whose image is broken, prints no
summary line at all, and a driver that decided on parsed counts would read that
silence as zero failures. Silence is not success: an unaccounted shard fails
the run and is named.

NOTE ON COVERAGE: the cloud image has NO KiCad, so every test needing pcbnew/wx
self-skips (exit 77 with a `SKIP:` reason). Those are reported in their own
bucket and are NOT passes. The wx/pcbnew parity gates in tests/gui_parity/ are
not collected by run_all at all and still need a local KiCad-python session.
"""
from __future__ import annotations

import os
import re
import sys
import time
from pathlib import Path

import modal

REPO = "/opt/kicad-routing-tools"

app = modal.App("kicad-run-all")

_here = Path(__file__).resolve().parent
_repo_root = _here.parent.parent.parent
sys.path.insert(0, str(_repo_root / "tests" / "stress" / "modal_sweep"))

# _image_source() shells out to `git`, which the container has neither of (no
# git, no repo). Compute locally; hand the containers the sha as an env var.
# Running it container-side is what once made every container crash at import.
def _tracked_manifest():
    """(path of a NUL-separated `git ls-files` dump, tracked board count).

    The container needs a git INDEX, because `run_utils.corpus_boards()` asks
    "which boards does git track?" via `git ls-files` and is documented to
    return [] -- with callers required to SKIP -- when git cannot answer. A
    bare image cannot answer, so nine tests graded against an empty corpus on
    the first full cloud run and failed. They pass locally: the signature of
    an image that is not a faithful checkout.

    `git init && git add -A` is NOT that reconstruction. `add` honours
    .gitignore, while a file added BEFORE a matching rule stays tracked
    forever -- `kicad_files/interf_u_unrouted.kicad_pcb` is exactly that, so a
    rebuilt index holds 21 of the 22 boards. Ship git's own answer instead and
    replay it with --force.
    """
    import subprocess as _sp
    import tempfile as _tf
    r = _sp.run(["git", "-C", str(_repo_root), "ls-files", "-z"],
                capture_output=True)
    path = os.path.join(_tf.mkdtemp(prefix="krt-tracked-"), "tracked")
    with open(path, "wb") as fh:
        fh.write(r.stdout)
    names = [n for n in r.stdout.decode("utf-8", "replace").split("\0") if n]
    boards = [n for n in names
              if n.startswith("kicad_files/") and n.endswith(".kicad_pcb")]
    return path, len(boards)


if modal.is_local():
    from modal_app import _image_source
    _src_dir, GIT_SHA = _image_source()
    _TRACKED, _N_BOARDS = _tracked_manifest()
else:
    _src_dir, GIT_SHA = "/tmp", os.environ.get("KICAD_SWEEP_GIT", "unknown")
    _TRACKED, _N_BOARDS = "/tmp/tracked", 0

# Installed from the repo's OWN requirements.txt, not from a hand-copied list.
# The sweep and route images pin numpy/scipy/shapely because that is the whole
# dependency set of the ROUTING path they exercise; the suite is not that
# narrow, and copying their list cost six phantom failures on the first real
# run -- test_897/test_898 died in `startup_checks.check_render_dependencies`
# ("Missing required Python libraries"), test_798/895/896 downstream of the
# same absent renderer, none of them self-skipping. Pillow is a HARD
# requirement of render_placement.py / route_render.py (module-scope import,
# no fallback), which requirements.txt says at length and a hardcoded triple
# cannot know.
#: The container's Python, matched to the interpreter that LAUNCHED the run.
#: Not pinned to a constant, because the version is not cosmetic here: this
#: repo has already had a Python upgrade change ROUTING RESULTS (math.fsum),
#: so a suite that grades on 3.12 is not grading the code the developer runs
#: on 3.14. It also changes what TESTS see -- `test_run8_write_order` passes
#: on 3.13+ and fails on 3.12 purely because 3.13 echoes the offending source
#: line in a `python -c` traceback, and that test's assertion (`'early line'
#: in the log`) is satisfied by the echo rather than by the program running.
#: A runner that silently picks its own interpreter turns differences like
#: that into phantom failures, and hides the real ones.
#:
#: Override with KICAD_SUITE_PYTHON=3.12 if Modal has no image for yours.
PY_VERSION = os.environ.get(
    "KICAD_SUITE_PYTHON",
    f"{sys.version_info.major}.{sys.version_info.minor}") \
    if modal.is_local() else "3.12"

#: Requirement names this image must NOT install, each with the reason. On
#: ipc-migration `requirements.txt` is also the file KiCad 10 provisions the
#: PLUGIN's venv from, so it declares the GUI front's runtime -- which this
#: image has no use for and, in wxPython's case, cannot build.
#:
#: Keyed by the normalised distribution name (lowercased, `_`/`.` -> `-`).
_IMAGE_EXCLUDE = {
    "wxpython":
        "PyPI publishes no manylinux wheel, so pip builds it from source and "
        "the image build DIES (`failed to build installable wheels`) before a "
        "single shard runs. Nothing here wants it: this image has no KiCad "
        "and no display, every wx test self-skips, and main's own image -- "
        "whose requirements.txt does not list wxPython -- has never had it. "
        "Dropping it restores parity with that image rather than removing "
        "something the suite uses.",
}


def _image_requirements():
    """requirements.txt minus `_IMAGE_EXCLUDE`, as a path to a temp file.

    Filtered rather than replaced by a hand-written list, for the reason the
    comment above gives at length: a hardcoded set cost six phantom failures
    on the first real run. Only the names declared above are dropped, and the
    declaration is checked in BOTH directions -- a name that is no longer IN
    requirements.txt fails here, so this set cannot rot into a silent
    exclusion of something that was renamed.

    Environment markers are left alone: `pyobjc-framework-Cocoa;
    sys_platform == 'darwin'` needs no entry because pip already skips it on
    Linux, and hiding it here would claim a decision this file did not make.
    """
    import re as _re
    import tempfile as _tf
    src = _repo_root / "requirements.txt"
    kept, dropped = [], set()
    for line in src.read_text(encoding="utf-8").splitlines(True):
        bare = line.split("#", 1)[0].strip()
        if not bare:
            kept.append(line)
            continue
        name = _re.split(r"[<>=!~;\[ ]", bare, 1)[0].strip()
        norm = _re.sub(r"[-_.]+", "-", name).lower()
        if norm in _IMAGE_EXCLUDE:
            dropped.add(norm)
            kept.append("# (dropped for the suite image: %s)\n" % norm)
            continue
        kept.append(line)
    stale = sorted(set(_IMAGE_EXCLUDE) - dropped)
    if stale:
        raise SystemExit(
            "run_all_modal: _IMAGE_EXCLUDE names %s, which requirements.txt "
            "no longer declares. A stale exclusion is how a dependency the "
            "suite DOES need gets dropped silently -- fix the name or delete "
            "the entry." % stale)
    if dropped:
        print("suite image: not installing %s (%s)"
              % (", ".join(sorted(dropped)),
                 "; ".join(_IMAGE_EXCLUDE[d] for d in sorted(dropped))[:120]
                 + "..."))
    path = os.path.join(_tf.mkdtemp(prefix="krt-req-"), "requirements.txt")
    with open(path, "w", encoding="utf-8") as fh:
        fh.writelines(kept)
    return path


image = (
    modal.Image.debian_slim(python_version=PY_VERSION)
    .pip_install_from_requirements(
        _image_requirements() if modal.is_local()
        else str(_repo_root / "requirements.txt"))
    # pytest is a TEST-only dependency, so it is deliberately absent from
    # requirements.txt (which is the shipping runtime). A handful of tests
    # import it for fixtures/parametrisation and die with
    # ModuleNotFoundError without it -- not a skip, a hard failure.
    .pip_install("pytest")
    .apt_install("git", "procps", "curl")
    .env({"KICAD_SWEEP_GIT": GIT_SHA, "PYTHONUNBUFFERED": "1"})
    .add_local_dir(_src_dir, REPO, copy=True, ignore=[
        "**/.git/**", "**/__pycache__/**", "**/target/**",
        "**/.claude/worktrees/**",
    ])
    .add_local_file(_TRACKED, "/opt/krt_tracked", copy=True)
    .run_commands(
        # Rebuild the index from git's OWN tracked list (see
        # `_tracked_manifest`), with --force so a tracked-but-now-ignored file
        # is not dropped. Before build_router.py, so the built .so is never
        # in it.
        #
        # `rm -rf .git` first: KICAD_SWEEP_DIRTY=1 from a git WORKTREE ships a
        # .git FILE -- a gitlink naming a host path that does not exist here
        # -- and `git init` refuses it with "not a git repository". The ignore
        # list cannot catch that: `**/.git/**` matches a directory's contents,
        # never a plain file of the same name.
        f"cd {REPO} && rm -rf .git && git init -q && "
        f"git add --force --pathspec-from-file=/opt/krt_tracked "
        f"--pathspec-file-nul",
        # Assert the image is a FAITHFUL checkout, against the count measured
        # on the host -- not a hardcoded threshold, which would rot the first
        # time a board is added. A silently short corpus is the failure this
        # whole dance exists to prevent, and it is invisible in the test
        # output (tests just grade a smaller set).
        f"cd {REPO} && python3 -c \"import subprocess,sys; "
        f"n=len(subprocess.run(['git','ls-files','kicad_files/*.kicad_pcb'],"
        f"capture_output=True,text=True).stdout.split()); "
        f"want={_N_BOARDS}; print('corpus boards visible to git:', n, 'want', want); "
        f"sys.exit(0 if n == want else "
        f"f'image is not a faithful checkout: {{n}} of {{want}} boards')\"",
        f"cd {REPO} && python3 build_router.py",
        # Prove the extension imports IN A FRESH PROCESS before any shard runs.
        # A broken .so would otherwise surface as 594 identical import errors
        # spread over 50 containers.
        f"cd {REPO} && python3 -c \"import sys; sys.path.insert(0,'rust_router'); "
        f"import grid_router; print('grid_router', grid_router.__version__)\"",
    )
)

#: One shard of ~12 files at --jobs 2. The per-TEST budget is run_all's own
#: --timeout; this is the whole-shard ceiling and must exceed
#: ceil(files/jobs) * per_test, or a slow shard dies as an unaccounted
#: container rather than as a named timed-out test.
SHARD_TIMEOUT_S = 5400


@app.function(image=image, cpu=2.0, memory=4096, timeout=SHARD_TIMEOUT_S,
              max_containers=64, retries=0)
def run_shard(spec: tuple) -> dict:
    """Run one shard; return its rc, counts and full log.

    `retries=0` deliberately: a retried flaky test would be reported green on
    its second run, and this suite is also how flakiness gets noticed.
    """
    import subprocess
    index, count, fast, per_test_timeout, jobs, filters = spec
    argv = [sys.executable, "-X", "utf8", "run_all.py",
            "--shard", f"{index}/{count}",
            "--timeout", str(per_test_timeout),
            "--jobs", str(jobs)]
    if fast:
        argv.append("--fast")
    argv += list(filters or [])
    t0 = time.time()
    try:
        r = subprocess.run(argv, cwd=os.path.join(REPO, "tests"),
                           capture_output=True, text=True,
                           encoding="utf-8", errors="replace",
                           timeout=SHARD_TIMEOUT_S - 120)
        rc, out = r.returncode, (r.stdout or "") + (r.stderr or "")
    except subprocess.TimeoutExpired as e:
        rc = 124
        out = ((e.stdout or "") if isinstance(e.stdout, str)
               else (e.stdout or b"").decode("utf-8", "replace"))
        out += f"\n!! SHARD TIMED OUT at {SHARD_TIMEOUT_S - 120}s"
    return {"index": index, "rc": rc, "log": out,
            "seconds": time.time() - t0, **_counts(out)}


_SUMMARY_RE = re.compile(
    r"^(\d+) passed, (\d+) failed, (\d+) timed out, (\d+) skipped "
    r"\(\+(\d+) self-skipped\) in ([\d.]+)s", re.M)


def _counts(log: str) -> dict:
    """Parse run_all's summary line. REPORTING ONLY -- never the verdict.

    Returns `parsed=False` when the line is absent, which is what a crashed or
    OOM-killed container produces. The caller must treat that as a shard that
    did not report rather than as a shard with zero failures.
    """
    m = None
    for m in _SUMMARY_RE.finditer(log):
        pass                      # the LAST one: shards print exactly one
    if not m:
        return {"parsed": False, "passed": 0, "failed": 0, "timed_out": 0,
                "skipped": 0, "self_skipped": 0,
                "failed_names": [], "timed_out_names": []}
    return {
        "parsed": True,
        "passed": int(m.group(1)), "failed": int(m.group(2)),
        "timed_out": int(m.group(3)), "skipped": int(m.group(4)),
        "self_skipped": int(m.group(5)),
        "failed_names": _named(log, "Failed: "),
        "timed_out_names": _named(log, "Timed out: "),
    }


def _named(log: str, prefix: str) -> list:
    for line in log.splitlines():
        if line.startswith(prefix):
            return [p.strip() for p in line[len(prefix):].split(",") if p.strip()]
    return []


@app.local_entrypoint()
def main(shards: int = 50, fast: bool = False, timeout: float = 600.0,
         jobs: int = 2, filters: str = "", out_dir: str = ""):
    """Fan the suite out over `shards` containers and aggregate.

    filters: space-separated substrings, passed straight to run_all (so
    `--filters "908 910"` runs those families across all shards).
    out_dir:  write every shard's log here (default: a temp dir, printed).
    """
    import tempfile

    if shards < 1:
        raise SystemExit("--shards must be >= 1")
    terms = filters.split()
    out = Path(out_dir) if out_dir else Path(
        tempfile.mkdtemp(prefix="run_all_modal_"))
    out.mkdir(parents=True, exist_ok=True)

    print(f"suite on Modal: {shards} shard(s), cpu=2 jobs={jobs}, "
          f"per-test timeout {timeout:g}s, source {GIT_SHA}"
          + (f", filters {terms}" if terms else "")
          + (", --fast (unit only)" if fast else ""))
    specs = [(i, shards, fast, timeout, jobs, terms) for i in range(shards)]

    t0 = time.time()
    results, seen = [], set()
    # Exceptions are surfaced as results rather than killing the fan-out, so
    # one broken container cannot hide the other 49 shards' findings.
    for r in run_shard.map(specs, order_outputs=False,
                           return_exceptions=True):
        if isinstance(r, Exception):
            print(f"  shard RAISED: {type(r).__name__}: {r}", flush=True)
            continue
        results.append(r)
        seen.add(r["index"])
        (out / f"shard_{r['index']:03d}.log").write_text(r["log"],
                                                         encoding="utf-8")
        mark = "ok  " if r["rc"] == 0 else "FAIL"
        print(f"  [{mark}] shard {r['index']:>3}/{shards}  rc={r['rc']:<3} "
              f"{r['passed']:>3}p {r['failed']}f {r['timed_out']}t "
              f"{r['self_skipped']}ss  {r['seconds']:.0f}s"
              + ("" if r["parsed"] else "   <-- NO SUMMARY LINE"),
              # flush: redirected to a file, Python block-buffers, and a
              # 50-shard run then prints NOTHING for minutes and reads as a
              # hang. (It is not: the per-shard logs are already landing in
              # out_dir.)
              flush=True)

    dt = time.time() - t0
    missing = sorted(set(range(shards)) - seen)
    unparsed = sorted(r["index"] for r in results if not r["parsed"])
    red = sorted(r["index"] for r in results if r["rc"] != 0)

    tot = {k: sum(r[k] for r in results)
           for k in ("passed", "failed", "timed_out", "skipped", "self_skipped")}
    failed_names = [n for r in results for n in r["failed_names"]]
    timeout_names = [n for r in results for n in r["timed_out_names"]]

    print(f"\n{'=' * 66}\nSUITE on {GIT_SHA}: {tot['passed']} passed, "
          f"{tot['failed']} failed, {tot['timed_out']} timed out, "
          f"{tot['skipped']} skipped (+{tot['self_skipped']} self-skipped) "
          f"in {dt:.0f}s wall across {len(results)}/{shards} shard(s)")
    print(f"logs: {out}")

    if failed_names:
        print(f"\nFAILED ({len(failed_names)}):")
        for n in sorted(failed_names):
            print(f"  {n}")
    if timeout_names:
        print(f"\nTIMED OUT ({len(timeout_names)}) -- a timeout is not evidence "
              f"of a broken test; re-run each alone before recording it:")
        for n in sorted(timeout_names):
            print(f"  {n}")
    if tot["self_skipped"]:
        print(f"\n{tot['self_skipped']} self-skipped -- these asserted NOTHING "
              f"(the cloud image has no KiCad). They are not passes.")
    # A shard that never reported is the failure mode this block exists for:
    # its tests did not run, and without this the run would print green.
    if missing:
        print(f"\n!! {len(missing)} SHARD(S) NEVER REPORTED: {missing}\n"
              f"   Their tests did NOT run. This suite result is INCOMPLETE.")
    if unparsed:
        print(f"\n!! {len(unparsed)} SHARD(S) PRODUCED NO SUMMARY LINE: "
              f"{unparsed}\n   Read their logs above -- the counts exclude them.")

    ok = not (red or missing or unparsed)
    print(f"\n{'ALL GREEN' if ok else 'RED'}"
          + ("" if ok else f"  (failing shards: {red or 'none'};"
                           f" missing: {missing or 'none'};"
                           f" unparsed: {unparsed or 'none'})"))
    print("=" * 66)
    if not ok:
        raise SystemExit(1)
