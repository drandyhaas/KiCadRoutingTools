#!/usr/bin/env python3
"""Modal app: fan a routing-parameter sweep across the stress corpus.

    modal run tests/stress/modal_sweep/modal_app.py \
        --arms tests/stress/modal_sweep/arms.example.json \
        --sets set1,set2,set3,set4,set5,set6,set7,set8,set9,set10

ONE TASK = ONE (arm, board). That granularity is the whole point: an arm run as
a single task is ~9 CPU-hours SERIAL, so 100 of those in parallel still take
9 hours. Fanning out to board level makes the wall-clock floor the single
longest BOARD (tens of minutes) instead.

Measured on sets 1-10 (see sweep_lib.load_cost_table for provenance):
  one arm ~8.6 CPU-h  ->  100 arms ~864 CPU-h / 15,000 tasks
  500 concurrent cores ~1.7 h   |   ~1,730 cores reaches the floor

Memory: ONE small tier (1 GB) for everything -- Modal bills max(reserved,
used), so reservations only add cost while burst covers real peaks (p90
5.2 GB tasks complete fine; measured 0.3% over the pure-usage floor). The
8 GB tier is opt-in OOM insurance via KICAD_SWEEP_BIG_BOARDS; see
sweep_lib's tier note for the 414-task measurement behind this.

Prerequisites and known-unverified bits are in README.md. Notably this file has
NOT been executed against Modal's API from this repo -- pin `modal` per the
README and expect to adjust image-builder method names if you are on an older
client.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import modal

# Recorded manifests bake ABSOLUTE tool paths. Rather than assume one prefix, we
# detect each manifest's own and remap it (sweep_lib.recorded_repo_prefix), so
# manifests recorded from a worktree or another machine replay unchanged.
REPO = "/opt/kicad-routing-tools"
CORPUS = "/corpus"          # read-only volume: runs_setN/ manifests + boards
RESULTS = "/results"        # writable volume: one JSON per (arm, board)

# Dashboard-distinguishable runs: KICAD_SWEEP_NAME wins; otherwise derive
# from the --arms filename in sys.argv (arms.586.json -> kicad-sweep-586) so
# BARE `modal run` launches are named too, not just run_sweep.sh ones (the
# first 586 run showed up as the generic name because it bypassed the
# script). In-container neither source exists -- generic name, harmless:
# the name only matters at client-side app creation.
def _run_name():
    n = os.environ.get("KICAD_SWEEP_NAME")
    if n:
        return n
    if "--arms" in sys.argv[:-1]:
        stem = Path(sys.argv[sys.argv.index("--arms") + 1]).stem
        for pre in ("arms.", "arms_"):
            if stem.startswith(pre):
                stem = stem[len(pre):]
        # Same arms file on different board sets = identically-named apps on
        # the dashboard (two cross_lean runs, curated vs holdout). Append a
        # compact sets tag whenever --sets is explicit.
        if "--sets" in sys.argv[:-1]:
            sets = [s.strip() for s in
                    sys.argv[sys.argv.index("--sets") + 1].split(",") if s.strip()]
            if sets:
                tag = sets[0] if len(sets) == 1 else f"{sets[0]}-{sets[-1]}"
                stem = f"{stem}-{tag}"
        if stem:
            return f"kicad-sweep-{stem}"
    return "kicad-routing-sweep"


app = modal.App(_run_name())

corpus_vol = modal.Volume.from_name("kicad-corpus", create_if_missing=True)
results_vol = modal.Volume.from_name("kicad-sweep-results", create_if_missing=True)

_here = Path(__file__).resolve().parent
_repo_root = _here.parent.parent.parent


def _image_source() -> tuple:
    """(dir_to_ship, provenance) -- a CLEAN checkout of HEAD by default.

    Shipping the working tree makes the baseline arm whatever happens to be
    uncommitted locally. That is not hypothetical: the first green smoke run had
    `baseline` and `tp2` produce byte-identical results (8.05s, 100%, 0 DRC)
    because another session's uncommitted TRACK_PROXIMITY_COST=2.0 was baked in
    as "pristine", so both arms ran at 2.0. A sweep would have compared 100 arms
    against an unknown baseline and reported "no effect".

    `git archive HEAD` pins the image to a commit, which also means a sweep is
    reproducible and you can keep editing locally while it runs.

    KICAD_SWEEP_DIRTY=1 ships the working tree instead -- for deliberately
    sweeping an uncommitted engine change. It warns, and the commit recorded in
    every result row is suffixed `+dirty` so the provenance is never silent.
    """
    import subprocess as _sp
    import tempfile

    def _git(*a):
        return _sp.run(["git", "-C", str(_repo_root), *a],
                       capture_output=True, text=True).stdout.strip()

    sha = _git("rev-parse", "--short", "HEAD") or "unknown"
    if os.environ.get("KICAD_SWEEP_DIRTY") == "1":
        dirty = _git("status", "--porcelain")
        print(f"!! KICAD_SWEEP_DIRTY=1: shipping the WORKING TREE, not {sha}."
              f" Uncommitted files:\n{dirty or '   (none -- tree is clean)'}")
        return str(_repo_root), f"{sha}+dirty"
    tmp = Path(tempfile.mkdtemp(prefix="kicad-sweep-src-"))
    rc = _sp.run(["git", "-C", str(_repo_root), "archive", "--format=tar", "HEAD",
                  "-o", str(tmp / "src.tar")]).returncode
    if rc != 0:
        raise SystemExit("git archive HEAD failed -- cannot build a reproducible image")
    _sp.run(["tar", "-xf", str(tmp / "src.tar"), "-C", str(tmp)], check=True)
    (tmp / "src.tar").unlink()
    print(f"image source: clean checkout of {sha}")
    return str(tmp), sha


def _build_step() -> str:
    """The image's Rust-router build command: pinnable, and retried.

    A bare `build_router.py` has exactly two ways to fail here and both cost the
    entire image (there is no Rust toolchain, so its source-build fallback can
    only die): a flaky GitHub asset fetch, and #615 refusing the prebuilt when
    the shipped `rust_router/` differs from origin/main -- which is every image
    built at an old commit, i.e. every bisect arm.

    Both are handled here rather than by rewriting this file from the caller
    (corpus_bisect.sh used to sed the command in): a rewrite that silently
    no-ops launches an arm that dies half an hour later.

    LAST resort is a source build, which needs the rustup layer below. It is
    the ONLY path when the branch's crate is ahead of every published release
    (placement's 0.21.1 vs the v0.20.4 assets): no prebuilt exists, and pinning
    --tag to an older release would silently run a crate MISSING the parameters
    this Python passes (0.21.0 `attraction_potential`, 0.21.1
    `layer_direction_preferences`).
    """
    tag = os.environ.get("KICAD_SWEEP_BUILD_TAG", "").strip()
    br = "python3 build_router.py" + (f" --tag {tag}" if tag else "")
    src = ". $HOME/.cargo/env && python3 build_router.py --from-source"
    return (f"cd {REPO} && ({br} || (sleep 20 && {br}) || (sleep 60 && {br})"
            f" || ({src}))")


# THIS FILE IS IMPORTED INSIDE THE CONTAINER TOO, so nothing at module scope may
# assume a local dev environment. _image_source() shells out to `git`, which the
# container image does not have (and there is no repo there anyway): running it
# container-side raised FileNotFoundError on import, so EVERY container died at
# startup and Modal reported "crash-looping". Compute locally, then hand the
# result to the containers as an image env var.
if modal.is_local():
    _src_dir, GIT_SHA = _image_source()
else:
    # In-container: the image is already built, so the path is unused; the sha
    # comes from the env var stamped in at build time.
    _src_dir, GIT_SHA = "/tmp", os.environ.get("KICAD_SWEEP_GIT", "unknown")

# numpy/scipy/shapely are the ENTIRE runtime dependency set of the default
# image: check_drc.py and the router are pure python + the compiled grid_router.
# PINNED exactly (2026-08-11): unpinned ranges let image rebuilds float the
# numeric stack, and a float-behavior drift between two builds routed
# 28/96 boards DIFFERENTLY at identical config -- silently voiding every
# cross-era comparison (verified: both code eras identical locally).
# Bump only deliberately, expecting a new baseline era.
_PY_PINS = ("numpy==2.5.2", "scipy==1.18.0", "shapely==2.1.2")

# OPT-IN KiCad in the image (#650). KICAD_SWEEP_WITH_KICAD=1, which
# cloud_replay_sets.py --with-kicad sets; read CLIENT-side, where the image is
# defined.
#
# WHY it is opt-in and not the default: a different base image is a different
# numeric stack, i.e. a NEW BASELINE ERA (see the pin note above), so flipping
# it for everyone would silently void every cross-wave comparison. An arm pair
# launched --with-kicad shares one image, so ITS comparison stays internally
# valid while nobody else's baseline moves.
#
# WHY you would want it: without KiCad every oracle leg is DEAD, not degraded.
# oracle_reconnect returns {'available': False} the moment find_kicad_cli()
# comes back None, so the plane finalize's audit, its #589 re-audit and the
# the oracle summary check all no-op -- and a change that only acts
# through them (e.g. #650's in-run floor sync) A/Bs as a perfect null that
# looks like a measurement.
KICAD_IMAGE = os.environ.get("KICAD_SWEEP_KICAD_IMAGE", "kicad/kicad:10.0.0")
WITH_KICAD = os.environ.get("KICAD_SWEEP_WITH_KICAD", "") in ("1", "true", "on")


def _base_image():
    """The base + runtime deps, with or without KiCad."""
    if not WITH_KICAD:
        return (
            modal.Image.debian_slim(python_version="3.12")
            .pip_install(*_PY_PINS)
            # procps: redo_stress_test's peak-RSS sampler shells out to ps/pgrep;
            # absent from debian_slim, the sampler's except-pass reported 0 MB for
            # every cloud task -- which is exactly the data needed to right-size
            # (and stop over-paying for) the container memory tiers.
            .apt_install("curl", "procps", "build-essential")
        )
    return (
        modal.Image.from_registry(
            KICAD_IMAGE,
            # The official KiCad image runs as USER kicad; apt and pip need root.
            setup_dockerfile_commands=["USER root"])
        # python-is-python3: the image ships python3 but NO `python`, and
        # Modal's own builder shells out to `python -m pip` -- so pip_install
        # dies with exit 127 ("command not found") before installing anything.
        .apt_install("curl", "procps", "build-essential", "python3-pip",
                     "python-is-python3")
        # PEP 668: Ubuntu marks its system python externally managed, so a bare
        # pip install REFUSES rather than installing. Same pins either way --
        # the whole point is that only KiCad differs between the two images.
        .pip_install(*_PY_PINS, extra_options="--break-system-packages")
        .run_commands(
            # Prove BOTH oracle front-ends work before any board depends on
            # them, in the same spirit as the grid_router import probe below.
            # pcbnew specifically: it is the DETERMINISTIC link source (#490),
            # and its absence is silent -- the oracle just falls back to
            # kicad-cli, whose threaded connectivity reported 103/65/92 on
            # identical input on orangecrab. That noise would swamp a 75-board
            # A/B, so a missing pcbnew must kill the BUILD, not the result.
            #
            # This is also why the image must run on its OWN system python:
            # from_registry(add_python=...) installs a standalone interpreter
            # that cannot import the distro-built pcbnew.
            'python3 -c "import pcbnew, sys; print(\'pcbnew\', '
            'pcbnew.GetBuildVersion(), \'on python\', sys.version.split()[0])"',
            "kicad-cli version",
        )
    )


image = (
    _base_image()
    # Rust toolchain, for the source-build fallback in _build_step(). Placed
    # BEFORE add_local_dir on purpose: a layer after it rebuilds on every
    # source change, and rustup is a ~1 min download we do not want to repay
    # per commit. Costs a ~10 min cargo build the first time an image is built
    # for a given crate, then Modal caches the whole layer stack.
    .run_commands("curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal")
    # Carry the source commit to the containers. They cannot compute it: no git
    # binary, no repo -- see the is_local() guard above.
    .env({"KICAD_SWEEP_GIT": GIT_SHA,
          # Provenance: a wave built WITH KiCad ran its oracle legs and one
          # built without did not, so the two are not comparable.
          "KICAD_SWEEP_WITH_KICAD": "1" if WITH_KICAD else "0"})
    .add_local_dir(_src_dir, REPO, copy=True, ignore=[
        "**/.git/**", "**/__pycache__/**", "**/target/**", "**/.claude/worktrees/**",
    ])
    .run_commands(
        # A Rust toolchain IS installed above, so the last resort in _build_step()
        # is a real source build rather than a guaranteed death. The prebuilt is
        # still tried first -- it is ~10 min cheaper. Historical note follows.
        #
        # the v0.20.2 release publishes grid_router-linux-x86_64.so
        # built from crate 0.20.1, so build_router.py downloads the prebuilt and keeps
        # it (verified: the released macos-arm64 asset reports __version__ 0.20.1,
        # matching Cargo.toml). That took a rustup + build-essential layer and a ~10 min
        # cargo build off the cold image.
        #
        # This FAILS LOUDLY at image-build time if a future crate bump ships without
        # publishing binaries -- which is the behaviour you want. To unblock, either
        # publish the assets (a python-only release republishes current crate binaries;
        # see CLAUDE.md) or re-add:
        #   .apt_install("build-essential")
        #   .run_commands("curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal")
        # and append `|| (. $HOME/.cargo/env && python3 build_router.py --from-source)`.
        # Retried 3x with backoff: GitHub intermittently drops the release-asset
        # fetch ("Remote end closed connection without response"), and with no
        # Rust here build_router's fallback is a source build that cannot
        # succeed -- so ONE flaky download kills the whole image, and with it
        # every arm riding it (that is how the set20 upload died on 2026-08-12,
        # and how three bisect arms died together when six concurrent image
        # builds pulled the same asset).
        #
        # KICAD_SWEEP_BUILD_TAG pins the release the prebuilt comes from, for
        # callers building an image at an OLD engine commit: #615 makes
        # build_router skip the prebuilt whenever rust_router/ differs from
        # origin/main, and unpinned that means a source build -- i.e. death,
        # half an hour in. Read client-side, where the image is defined.
        _build_step(),
        # Pristine copy for per-task restore: containers are REUSED, so without
        # this a previous arm's routing_defaults patch leaks into the next task.
        f"cp {REPO}/py_router/routing_defaults.py {REPO}/py_router/routing_defaults.py.orig",
        # Prove the extension actually imports in a FRESH interpreter before 15,000
        # tasks depend on it. build_router verifies in a subprocess for the same
        # reason: an in-process re-import of a compiled extension reports the
        # PREVIOUSLY loaded library, so it can mask a bad install.
        f"cd {REPO} && python3 -c \"import sys; sys.path.insert(0,'rust_router'); "
        f"import grid_router; print('grid_router', grid_router.__version__)\"",
    )
)

with image.imports():
    sys.path.insert(0, f"{REPO}/tests/stress/modal_sweep")
    import sweep_lib


_TEE_RE = None


def _run_teed(cmd, env, tag, timeout_s):
    """subprocess.run(capture_output=True) replacement that TEES the chain's
    step-level progress to container stdout, so the Modal logs tab shows
    which board is on which routing step instead of 'no logs to display'
    (each task was otherwise silent by design: the full output goes to the
    per-board _replay.log). Echoed lines only -- step starts ([3/15] tool
    ...), step end-timings (-> 12.3s exit 0), the wave's per-board result
    line, and errors; the complete text is still captured and returned on a
    subprocess.run-compatible object for the harvest/log-persistence logic.
    """
    global _TEE_RE
    import re as _re
    import threading
    if _TEE_RE is None:
        _TEE_RE = _re.compile(
            r"^\[\d+/\d+\] |^\s*-> [\d.]+s|^\[[^]]+\] \S+: chain="
            r"|Traceback|FATAL|NORESULT|ABORT")
    proc = subprocess.Popen(cmd, env=env, text=True, errors="replace",
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    timer = threading.Timer(timeout_s, proc.kill)
    timer.start()
    buf = []
    try:
        for line in proc.stdout:
            buf.append(line)
            if _TEE_RE.search(line):
                print(f"[{tag}] {line.rstrip()[:200]}", flush=True)
        proc.wait()
    finally:
        timer.cancel()
    out = "".join(buf)
    print(f"[{tag}] done rc={proc.returncode}", flush=True)
    return subprocess.CompletedProcess(cmd, proc.returncode, out, "")


def _dep_versions() -> str:
    """numpy/scipy/shapely versions, stamped into every row: the era-drift
    postmortem's second defense -- if the pin is ever bumped or bypassed,
    rows self-document which numeric stack produced them."""
    from importlib.metadata import version
    return ",".join(f"{m}={version(m)}" for m in ("numpy", "scipy", "shapely"))


def _replay_one(task: dict) -> dict:
    """Replay + grade ONE board under ONE arm. Runs inside the container."""
    t0 = time.time()
    arm, board, set_name = task["arm"], task["board"], task["set"]
    print(f"[{arm}/{board}] start ({set_name})", flush=True)

    # 0. Restore a pristine routing_defaults.py. Modal REUSES containers across
    #    tasks and the image's repo copy is a writable overlay, so a previous
    #    arm's patch would otherwise persist -- a `baseline` task landing in a
    #    container that already ran `tp2` would silently route with tp2's costs
    #    and the two arms would look identical. Failing loudly if the pristine
    #    copy is missing beats sweeping with contaminated defaults.
    pristine = Path(REPO) / "py_router" / "routing_defaults.py.orig"
    if not pristine.exists():
        raise RuntimeError("routing_defaults.py.orig missing -- image built without it; "
                           "arms would contaminate each other in a reused container")
    shutil.copy(pristine, Path(REPO) / "py_router" / "routing_defaults.py")

    # 1. Stage the board at its RECORDED absolute path. Manifests bake the input
    #    board absolutely (<stress>/boards_unrouted_<set>/<board>.kicad_pcb) while
    #    intermediates are relative, and ab_replay_grade's own --remap src:dst
    #    assumes --set IS the recorded run dir. Staging anywhere else breaks both:
    #    the first tool exits rc=1 on a missing input, which is what the smoke test
    #    caught. Reconstructing the recorded path makes every baked path resolve
    #    natively, so only the REPO prefix ever needs remapping.
    manifest_src = Path(CORPUS) / f"runs_{set_name}" / board / "redo_commands.sh"
    recorded_stress = sweep_lib.recorded_corpus_prefix(manifest_src.read_text())
    if not recorded_stress:
        raise RuntimeError(f"{board}: no '# cwd=' line -- cannot place the corpus")
    root = Path(recorded_stress)
    set_dir = root / f"runs_{set_name}"
    set_dir.mkdir(parents=True, exist_ok=True)
    dst = set_dir / board
    # Clear EVERY board dir, not just this one. Modal reuses containers, and each
    # task stages into the same recorded set_dir -- so a previous task's board
    # was still sitting there when the next one ran. ab_replay_grade --set
    # replays every board dir it finds, so the wave then contained TWO boards and
    # `rows[0]` below could return the WRONG one: rows appeared with another
    # board's step names and net counts (ploopy_nano graded with
    # sensorwatch_round's step3b_fix_coxa chain). That silently varied chain
    # length across arms on 53 of 130 boards and corrupted every cross-arm
    # comparison, because a short chain grades artificially WELL -- nets its
    # missing steps never attempted are not counted as incomplete.
    for _stale in set_dir.iterdir() if set_dir.exists() else []:
        if _stale.is_dir():
            shutil.rmtree(_stale, ignore_errors=True)
    shutil.copytree(Path(CORPUS) / f"runs_{set_name}" / board, dst)
    # Board dirs are read-only inputs -> symlink rather than copy.
    # NAME SPLIT: the RUN dir may carry a recording suffix (runs_set3_llm0801)
    # while the board dirs never do (boards_unrouted_set3) -- deriving them
    # from the full set_name made srcd.exists() False, silently skipped the
    # symlink, and every chain's step 1 died on its absolute input path.
    # Symlink both spellings; manifests reference the unsuffixed one.
    base = re.match(r"(set\d+[a-z]*)", set_name)
    names = {f"boards_unrouted_{set_name}", f"boards_{set_name}"}
    if base:
        names |= {f"boards_unrouted_{base.group(1)}", f"boards_{base.group(1)}"}
    for d in sorted(names):
        srcd, lnk = Path(CORPUS) / d, root / d
        if srcd.exists() and not lnk.exists():
            lnk.symlink_to(srcd)
    # set_dir now holds exactly this ONE board (enforced above), so
    # ab_replay_grade replays only it --
    # and emits the SAME summary.json schema as a full-set wave, so `--compare`
    # still works on the aggregate and no grading logic is duplicated here.
    stage = set_dir
    work = Path(f"/tmp/{arm}__{set_name}__{board}")

    # 2. Apply this arm's parameters.
    spec = task["arm_spec"]
    env = dict(os.environ)
    env.update({k: str(v) for k, v in (spec.get("env") or {}).items()})
    patched = {}
    if spec.get("defaults"):
        patched = sweep_lib.patch_routing_defaults(Path(REPO), spec["defaults"])
    manifest_path = dst / "redo_commands.sh"
    text = manifest_path.read_text()
    if spec.get("manifest_sed"):
        text = sweep_lib.patch_manifest(text, spec["manifest_sed"])
        manifest_path.write_text(text)

    # 3. Remap the recorded repo prefix onto this container's repo.
    extra = []
    prefix = sweep_lib.recorded_repo_prefix(text)
    if prefix and prefix.rstrip("/") != REPO:
        extra = ["--extra-remap", f"{prefix.rstrip('/')}/:{REPO}/"]

    out_dir = work / "wave"
    cmd = [sys.executable, f"{REPO}/tests/stress/ab_replay_grade.py",
           "--set", str(stage), "--out", str(out_dir),
           "--label", arm, "--jobs", "1"] + extra
    proc = _run_teed(cmd, env=env, tag=f"{arm}/{board}",
                     timeout_s=task.get("timeout_s", 4 * 3600))

    # 4. Harvest the single summary row.
    row, summary_file = {}, out_dir / "summary.json"
    if summary_file.exists():
        try:
            rows = json.loads(summary_file.read_text())
            # Select BY BOARD NAME. rows[0] is whatever sorted first, which is
            # only this board when the wave holds exactly one -- the invariant
            # container reuse broke above. Belt and braces: even if a stale dir
            # survives, we never publish another board's numbers under this
            # board's name.
            _mine = [r for r in rows if r.get("board") == board]
            if _mine:
                row = _mine[0]
            elif rows:
                row = dict(rows[0])
                row["wrong_board"] = (f"summary had no row for {board}; "
                                      f"got {[r.get('board') for r in rows]}")
        except Exception as e:
            row = {"parse_error": str(e)[:200]}
    row.update({
        "arm": arm, "set": set_name, "board": board, "git": GIT_SHA,
        "deps": _dep_versions(),
        "rc": proc.returncode,
        "sweep_wall_s": round(time.time() - t0, 1),
        "patched_defaults": {k: v[1] for k, v in patched.items()},
        "env_overrides": spec.get("env") or {},
    })
    if not summary_file.exists():
        # A board that produced no summary at all is a real finding, not noise --
        # keep enough context to reproduce it rather than dropping the row.
        # (stderr rides stdout since the _run_teed change.)
        row["stderr_tail"] = (proc.stderr or proc.stdout or "")[-2000:]
        row["chain_complete"] = False
    if not row.get("chain_complete"):
        # A summary can exist and STILL report a broken chain (a tool exited
        # non-zero). The engine's error lives in the per-board _replay.log inside
        # the ephemeral container, so carry its tail out -- without it the row
        # says only "chain_complete: false" and the failure is undiagnosable,
        # which is exactly where the first smoke run left us.
        log = out_dir / board / "_replay.log"
        if log.exists():
            row["replay_log_tail"] = log.read_text(errors="replace")[-3000:]

    dest = Path(RESULTS) / arm
    dest.mkdir(parents=True, exist_ok=True)
    if not row.get("chain_complete"):
        # Tail-only context proved insufficient (the first failing step's
        # error scrolls off a 3000-char tail on a 15-command chain): persist
        # the WHOLE replay log as a sibling on a broken chain. Cheap -- only
        # broken rows pay -- and it turns cloud debugging from re-run-with-
        # a-tweak roundtrips into a single volume get.
        log = out_dir / board / "_replay.log"
        if log.exists():
            (dest / f"{set_name}__{board}.log").write_text(
                log.read_text(errors="replace"))

    # Keep the routed board itself, not just its score (KICAD_SWEEP_KEEP_ARTIFACTS).
    #
    # A sweep only needs the numbers, so this is OPT-IN -- 150 boards of copper is
    # storage a parameter sweep should not pay for. A cloud REPLAY is the other
    # case: the whole point is to end up with boards you can open, re-grade with a
    # later grader, or diff against a baseline, and the container is gone the
    # moment the row is written.
    #
    # The final .kicad_pcb travels WITH ITS SIBLINGS (.kicad_pro / .kicad_dru /
    # .kicad_prl), never alone. The project carries the DRC floor the chain routed
    # to and the dru carries per-layer clearance (#498); a bare .kicad_pcb is
    # re-graded against the STOCK netclass instead, which manufactures phantom
    # sub-floor violations on copper that is actually correct (#441 -- icepi_zero
    # turned a dropped 0.09 floor into 160 phantom grazes). Shipping the pcb
    # without the pro would make every artifact we keep un-regradable.
    # Carried on the ARM SPEC, which is serialized into the task -- a Modal
    # container does NOT inherit the launching shell's environment, so setting
    # KICAD_SWEEP_KEEP_ARTIFACTS locally silently kept nothing (caught by the
    # smoke run: chain green, 0 artifacts). The env var remains an in-container
    # override for ad-hoc use.
    keep = (task["arm_spec"].get("keep_artifacts")
            or os.environ.get("KICAD_SWEEP_KEEP_ARTIFACTS"))
    if keep and row.get("final"):
        final = out_dir / board / str(row["final"])
        if final.exists():
            adir = dest / "artifacts" / f"{set_name}__{board}"
            adir.mkdir(parents=True, exist_ok=True)
            kept = []
            for sib in sorted(final.parent.glob(final.stem + ".*")):
                if sib.suffix in (".kicad_pcb", ".kicad_pro", ".kicad_dru", ".kicad_prl"):
                    shutil.copy2(sib, adir / sib.name)
                    kept.append(sib.name)
            # The manifest travels too: --regrade needs it to recover the routed
            # --clearance, and grading at anything else is the classic phantom-DRC
            # mistake (CLAUDE.md: grade at the clearance the board was routed to).
            man = dst / "redo_commands.sh"
            if man.exists():
                shutil.copy2(man, adir / "redo_commands.sh")
                kept.append("redo_commands.sh")
            row["artifacts"] = kept
            if ".kicad_pro" not in {Path(k).suffix for k in kept}:
                # Loud, because the failure mode is silent and only shows up much
                # later as phantom DRC on a regrade.
                row["artifact_warning"] = f"{row['final']} has no sibling .kicad_pro"
                print(f"[{arm}/{board}] WARNING: no .kicad_pro beside {row['final']}",
                      flush=True)
        else:
            row["artifact_warning"] = f"final {row['final']} not found in wave dir"

    (dest / f"{set_name}__{board}.json").write_text(json.dumps(row, indent=1))
    results_vol.commit()
    return row


@app.function(image=image, memory=4096, timeout=1800,
              volumes={CORPUS: corpus_vol})
def extract_corpus(archive_rel: str) -> int:
    """Server-side half of the compressed corpus upload: extract a tar.gz
    that upload_corpus.py placed on the volume, then delete it. The corpus
    is s-expression text (~6-8x gzip), so shipping one compressed archive
    turns a ~2.2 GB / ~10 min upload into ~2 min total. Extraction runs
    where bandwidth is free."""
    import os
    import tarfile
    ar = os.path.join(CORPUS, archive_rel.lstrip("/"))
    n = 0
    with tarfile.open(ar, "r:gz") as tf:
        for m in tf:
            tf.extract(m, CORPUS, filter="data")
            n += 1
    os.remove(ar)
    corpus_vol.commit()
    return n


# NO cpu= request: the chain is single-threaded, so a cpu=1.0 request
# reserved a full physical core while one hyperthread consumed ~half of it
# -- billed on max(request, usage), that's ~2x the honest CPU price. At the
# 0.125-core floor request, billing follows consumption (~0.5 core/job) and
# scheduling stays wide-not-fat (Andy). OMP/BLAS pinning at a low request is
# single-thread -- identical to the cpu=1.0 behavior, so results unchanged.
@app.function(image=image, memory=1024,
              timeout=4 * 3600, retries=modal.Retries(max_retries=3),
              volumes={CORPUS: corpus_vol, RESULTS: results_vol})
def replay_standard(task: dict) -> dict:
    return _replay_one(task)


@app.function(image=image, memory=8192,
              timeout=6 * 3600, retries=modal.Retries(max_retries=3),
              volumes={CORPUS: corpus_vol, RESULTS: results_vol})
def replay_big(task: dict) -> dict:
    """Same work, bigger tier (4 GB while measuring -- was 12 GB sized
    from macOS footprint; real Linux RSS now recorded per task, resize from
    that). For boards whose recorded peak footprint exceeds
    ~3.5 GB -- four boards in the reference wave, one at 10 GB."""
    return _replay_one(task)


@app.local_entrypoint()
def main(arms: str, sets: str = "set1,set2,set3,set4,set5,set6,set7,set8,set9,set10",
         stress_dir: str = "", out: str = "", dry_run: bool = False,
         boards: str = "", limit: int = 0, stage: str = "full",
         gate_margin: float = 0.25, ignore_excludes: bool = False,
         exclude: str = ""):
    """boards: comma-separated board names to restrict to (smoke tests).
    exclude: comma-separated board names to SKIP this run -- the ad-hoc lever
            for multi-hour monsters that set the wall-clock floor (on sets
            11-20, four boards are half the cost and all of the 11 h floor).
    limit:  keep only the N CHEAPEST boards -- a smoke test wants fast feedback,
            and the default longest-first order would otherwise hand you the
            50-minute monster first.
    stage:  'full' = every arm x every board (minus board_value.json excludes).
            'screened' = successive halving: every arm first runs the ~12-board
            SCREEN set from board_value.json (~$0.15/arm); arms whose screen
            pad-verdict is worse than baseline's by more than gate_margin
            (fractional, min +3 absolute) are KILLED before the full set --
            a clearly-bad arm costs cents instead of dollars.
    gate_margin: screened-stage kill threshold; generous by default so
            regime-specialists (good on boards the cheap screen lacks) survive
            to the full set. The gate can only save money, not flip winners:
            final rankings still come from the full set."""
    stress = Path(stress_dir or os.path.expanduser("~/Documents/kicad_stress_test"))
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import sweep_lib as sl

    arm_specs = json.loads(Path(arms).read_text())
    set_list = [s.strip() for s in sets.split(",") if s.strip()]
    board_list = sl.discover_boards(stress, set_list)
    costs = sl.load_cost_table(stress)

    bv_path = Path(__file__).resolve().parent / "board_value.json"
    board_value = json.loads(bv_path.read_text()) if bv_path.exists() else {}
    # --ignore-excludes: board_value's exclude list is FITTED to the arm
    # family it was measured on (see the caveat field) -- a NEW knob family's
    # first sweep must not inherit it (a board where soft-cost arms all tie
    # may discriminate heuristic/rip-up arms fine).
    excluded = set() if ignore_excludes else set(board_value.get("exclude", []))
    if excluded:
        # Report what was actually DROPPED from this run, not the whole exclude
        # list: printing the list next to a count of 0 reads as "46 boards
        # excluded" when the truth is "none of them are in these sets".
        dropped = sorted({b for _, b in board_list} & excluded)
        board_list = [(s, b) for s, b in board_list if b not in excluded]
        if dropped:
            print(f"  board_value: excluded {len(dropped)} zero-information "
                  f"board(s): {dropped}")

    # --exclude: boards to skip in THIS run, by name. Distinct from
    # board_value's list (which is a fitted, persistent judgement about boards
    # that discriminate nothing): this is the ad-hoc "not worth its wall clock
    # today" lever -- a handful of multi-hour monsters can be most of a sweep's
    # cost and all of its wall-clock floor, and dropping them buys back hours
    # without materially changing the corpus a verdict rests on.
    if exclude:
        skip = {b.strip() for b in exclude.split(",") if b.strip()}
        matched = skip & {b for _, b in board_list}
        board_list = [(s, b) for s, b in board_list if b not in skip]
        print(f"  --exclude: dropped {len(matched)} board(s): {sorted(matched)}")
        if skip - matched:
            # Loud: a typo'd name silently leaves the monster in the run, and
            # you find that out from the wall clock many hours later.
            print(f"  WARNING: excluded name(s) matching no board in {set_list}: "
                  f"{sorted(skip - matched)} -- check for typos")

    if boards:
        want = {b.strip() for b in boards.split(",") if b.strip()}
        board_list = [(s, b) for s, b in board_list if b in want]
        missing = want - {b for _, b in board_list}
        if missing:
            raise SystemExit(f"not found in sets {set_list}: {sorted(missing)}")
    if limit:
        board_list = sorted(board_list,
                            key=lambda sb: (costs.get(sb[1]) or {}).get("seconds", 1e9))[:limit]

    tasks = sl.build_tasks(arm_specs, board_list, costs)
    boards = board_list  # keep the downstream name

    est_h = sum(t["est_seconds"] for t in tasks) / 3600
    floor_m = max((t["est_seconds"] for t in tasks), default=0) / 60
    big = [t for t in tasks if t["memory_mb"] > sl.DEFAULT_MEM_MB]
    print(f"{len(arm_specs)} arms x {len(boards)} boards = {len(tasks)} tasks")
    print(f"  estimated {est_h:,.0f} CPU-hours; floor (longest board) {floor_m:.0f} min")
    print(f"  {len(big)} tasks routed to the big tier")
    unknown = sum(1 for t in tasks if t["board"] not in costs)
    if unknown:
        print(f"  NOTE {unknown} tasks have no recorded cost -- ordered mid-pack, "
              f"so a hidden monster could extend the tail")
    if dry_run:
        for t in tasks[:10]:
            print(f"   {t['est_seconds']/60:6.1f}m {t['memory_mb']:>6}MB {t['arm']:20} {t['board']}")
        return

    started = time.time()

    # Resume: a stopped run's completed work is banked as rows on the results
    # volume -- re-running those tasks would just re-bill them. Skip any
    # (arm, board) whose row exists; true holes (preempted-twice, OOM) have
    # no row and re-run naturally. This is what makes stop -> retier ->
    # relaunch iterations cheap.
    # ONE recursive listing, and failures are LOUD: the first version did 25
    # per-arm listdir calls with except-pass, transient failures silently
    # un-skipped those arms' rows, and 43 banked tasks re-ran (harmless --
    # deterministic rows overwrite identically -- but re-billed).
    existing = set()
    try:
        for e in results_vol.listdir("/", recursive=True):
            p = e.path.lstrip("/")
            if p.endswith(".json") and "/" in p:
                existing.add(p)
    except Exception as ex:
        print(f"  resume: results-volume listing FAILED ({ex}); running all tasks")
    skipped = []
    if existing:
        def _row_path(t):
            return f"{t['arm']}/{t['set']}__{t['board']}.json"
        skipped = [t for t in tasks if _row_path(t) in existing]
        tasks = [t for t in tasks if _row_path(t) not in existing]
        print(f"  resume: {len(skipped)} tasks already have rows on "
              f"the volume -- skipped; {len(tasks)} to run")

    # .map() BLOCKS until its tier drains, so calling the two tiers in sequence
    # would hold every big-tier board back until the small ones finished (or
    # vice versa) -- and the big tier holds the longest boards, i.e. exactly
    # the ones that must start first. Drain both concurrently instead.
    import threading

    def run_tasks(items):
        rows, lock = [], threading.Lock()

        def drain(fn, sub):
            if not sub:
                return
            for r in fn.map(sub, order_outputs=False):
                with lock:
                    rows.append(r)

        threads = [threading.Thread(target=drain, args=(
                       replay_big,
                       [t for t in items if t["memory_mb"] > sl.DEFAULT_MEM_MB])),
                   threading.Thread(target=drain, args=(
                       replay_standard,
                       [t for t in items if t["memory_mb"] <= sl.DEFAULT_MEM_MB]))]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        return rows

    if stage == "screened":
        screen_boards = set(board_value.get("screen", []))
        if not screen_boards:
            raise SystemExit("--stage screened needs board_value.json with a "
                             "screen list (run curate_boards.py)")
        screen_tasks = [t for t in tasks if t["board"] in screen_boards]
        rest = [t for t in tasks if t["board"] not in screen_boards]
        print(f"  stage 1: screen -- {len(screen_tasks)} tasks on "
              f"{len(screen_boards)} boards")
        results = run_tasks(screen_tasks)
        # Gate on the pad-verdict proxy summed over screen boards, PAIRED:
        # an arm is judged only against the boards where both it and baseline
        # completed this run. Arms whose screen rows were all banked (resume)
        # have no fresh coverage and ADVANCE -- the gate saves money on
        # clearly-bad NEW arms, it never blocks on missing data, and final
        # rankings always come from the full set.
        # Score = nets_incomplete ALONE. It already counts unrouted PLUS
        # connectivity-issue nets, so adding `conn` back counted every
        # connectivity-issue net twice -- which is not a harmless rescaling: it
        # priced "failed to connect a net" at double "lost the net's copper
        # entirely", and the gate KILLS arms on this number. ab_replay_grade
        # states the same definition where it grades its own A/B.
        def _verdict(r):
            return r.get("nets_incomplete") or 0

        per = {}
        for r in results:
            if r and r.get("chain_complete"):
                per.setdefault(r["arm"], {})[r["board"]] = _verdict(r)
        # Top up screen coverage from BANKED rows: an arm fully banked by a
        # prior run (baseline, after any earlier campaign) has no FRESH rows,
        # which used to skip the gate entirely. Banked rows are equally valid
        # comparison data -- read them off the volume.
        for spec in arm_specs:
            aname = spec["name"]
            have = per.get(aname, {})
            for sb in screen_boards - set(have):
                hit = next((p for p in existing
                            if p.startswith(f"{aname}/")
                            and p.endswith(f"__{sb}.json")), None)
                if not hit:
                    continue
                try:
                    raw = b"".join(results_vol.read_file(hit))
                    r = json.loads(raw)
                    r = r[0] if isinstance(r, list) else r
                    if r.get("chain_complete"):
                        per.setdefault(aname, {})[sb] = _verdict(r)
                except Exception as ex:
                    print(f"  gate: banked row {hit} unreadable ({ex})")
        killed = set()
        # The control arm is the first one declaring no overrides -- NOT the
        # literal name "baseline". Arms files deliberately name it something
        # unique (baseline590, base590h) because the results volume banks rows
        # BY ARM NAME, so a prior campaign's "baseline" rows at another commit
        # satisfy the resume check and poison the anchor. Matching on the name
        # meant every such wave silently ran with the gate disabled.
        base_arm = next((s["name"] for s in arm_specs
                         if not s.get("env") and not s.get("defaults")
                         and not s.get("manifest_sed")), None)
        base = per.get(base_arm) if base_arm else None
        if not base:
            print(f"  gate: no control-arm screen rows "
                  f"({base_arm or 'no control arm declared'}) -- gate skipped")
        else:
            print(f"  gate: control arm is {base_arm}")
            for a_, sc in sorted(per.items()):
                if a_ == base_arm:
                    continue
                both = set(sc) & set(base)
                if len(both) < max(3, len(base) // 2):
                    continue    # too little paired coverage to judge
                s_arm = sum(sc[b] for b in both)
                s_base = sum(base[b] for b in both)
                thresh = s_base + max(3.0, gate_margin * s_base)
                if s_arm > thresh:
                    killed.add(a_)
                    print(f"  gate: KILLED {a_} (screen {s_arm} vs baseline "
                          f"{s_base} on {len(both)} boards, threshold {thresh:.1f})")
        survivors = {s["name"] for s in arm_specs} - killed
        rest = [t for t in rest if t["arm"] in survivors]
        print(f"  stage 2: full -- {len(survivors)}/{len(arm_specs)} arms, "
              f"{len(rest)} tasks")
        results += run_tasks(rest)
    else:
        results = run_tasks(tasks)

    # Merge the RESUMED rows back in. Skipping a banked task saves the compute;
    # dropping its row from the output silently shrinks the comparison instead,
    # and every downstream consumer (rank_arms' pairing, check_sweep's
    # arms-differ check, the summary table) reads the output json alone. A
    # resumed sweep therefore used to report a partial corpus as if it were the
    # whole one -- worst on the boards a relaunch is most likely to follow, the
    # slow ones that got banked before an interruption.
    if skipped:
        recovered, lost = [], []
        for t in skipped:
            path = f"{t['arm']}/{t['set']}__{t['board']}.json"
            try:
                r = json.loads(b"".join(results_vol.read_file(path)))
                recovered.append(r[0] if isinstance(r, list) else r)
            except Exception as ex:
                lost.append(f"{path} ({ex})")
        results += recovered
        print(f"  resume: merged {len(recovered)} banked row(s) into the result")
        if lost:
            # Loud: an unreadable banked row is a HOLE in the comparison, and a
            # hole reads as "this board was never run" rather than as an error.
            print(f"  WARNING: {len(lost)} banked row(s) unreadable, so those "
                  f"(arm, board) pairs are MISSING from this result: {lost[:5]}")

    summary = sl.summarize(results)
    summary["wall_clock_s"] = round(time.time() - started, 1)
    if out:
        out_path = Path(out)
    else:
        # Name the default by ARM, not by the clock alone. `started` is stamped
        # AFTER the image build, so two runs launched seconds apart that share a
        # cached image reach it within the same second and derive the SAME
        # filename -- and the second silently clobbers the first. That is not
        # hypothetical: the v0.20.2-vs-HEAD engine A/B ran both arms
        # concurrently and lost one arm's result json outright. It was only
        # recoverable because rows are ALSO banked per-arm on the results
        # volume; the local json is the convenient copy, not the record.
        _tag = "-".join(sorted(s["name"] for s in arm_specs))[:60] or "sweep"
        out_path = stress / f"sweep_{int(started)}_{_tag}.json"
        _n = 1
        while out_path.exists():
            # Two runs of the SAME arms in one second is unlikely, but never
            # overwrite a sibling run's result on the way to finding out.
            out_path = stress / f"sweep_{int(started)}_{_tag}_{_n}.json"
            _n += 1
    out_path.write_text(json.dumps({"summary": summary, "rows": results}, indent=1))
    print(f"\nwall {summary['wall_clock_s']/3600:.2f} h -> {out_path}")
    print(f"{'arm':24} {'boards':>6} {'ok':>4} {'mean_compl%':>11} {'DRC':>6} {'CPU-h':>7}")
    for a in summary["arms"]:
        print(f"{a['arm']:24} {a['boards']:6} {a['chain_complete']:4} "
              f"{str(a['mean_completion_pct']):>11} {str(a['total_drc_real']):>6} {a['cpu_hours']:7.2f}")
