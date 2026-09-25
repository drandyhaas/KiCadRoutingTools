"""Re-grade an existing cloud arm's kept boards ON MODAL, with one checkout's grader.

The cloud form of `cloud_replay_sets.py --regrade-baseline`. That flag runs
`ab_replay_grade.py --regrade` on this machine; this runs the SAME command, one
board per container, in the image `modal_app.py` builds from a checkout's HEAD
-- the image a new arm replays in. So a baseline arm and a new arm are graded
by one grader, on one machine, without a laptop-hours local regrade.

Reads  /results/<src_arm>/artifacts/<set>__<board>/   final pcb + siblings, and
       the manifest the arm REPLAYED (which rode with the artifact)
       /results/<src_arm>/<set>__<board>.json          the arm's own row
Writes /results/<dst_arm>/<set>__<board>.json          that row with every GRADED
       field replaced by the regrade. Fields describing the ROUTING run are kept:
       timing, rc, and the diff-pair stats (parsed from a replay log the regrade
       does not have). `pre_regrade` holds the replaced values, `regraded_by`
       the grading image's sha.
A board whose regrade yields no complete grade, or a different final board,
gets `<set>__<board>.log` and NO row, so a pairing can never read the old
grader's numbers as the new one's. Re-running resumes: boards that already
have a row in <dst_arm> are skipped.

The board is staged exactly as `modal_app._replay_one` stages a replay (corpus
at its recorded path, the unrouted input reachable), so the #405 input baseline
resolves the same way. Checked: regrading an arm's own boards with the image
that replayed it reproduces every graded key of its rows.

    # grade with THIS checkout's HEAD
    modal run tests/stress/modal_sweep/regrade_arm.py \\
        --src-arm heads110-kc_96912901 --dst-arm heads110-kc_96912901_rg<sha>
    # grade with ANOTHER checkout's HEAD (e.g. a PR worktree)
    KICAD_REGRADE_REPO=/path/to/checkout modal run .../regrade_arm.py ...

Then pair it against the new arm's harvested wave:

    modal volume get kicad-sweep-results /<dst_arm> <wave>/_raw/
    python3 tests/stress/pair_arms.py <wave> <new arm's wave dir>

The image carries KiCad (KICAD_SWEEP_WITH_KICAD defaults to 1 here), matching
cloud_replay_sets' default -- grade a `--no-kicad` arm with
KICAD_SWEEP_WITH_KICAD=0.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

_CONTAINER_SWEEP = "/opt/kicad-routing-tools/tests/stress/modal_sweep"
if os.path.isdir(_CONTAINER_SWEEP):
    sys.path.insert(0, _CONTAINER_SWEEP)
else:
    # modal_app builds its image from the checkout it is imported FROM, so
    # this choice is the choice of grader.
    _repo = os.environ.get("KICAD_REGRADE_REPO") or str(Path(__file__).resolve().parents[3])
    sys.path.insert(0, str(Path(_repo) / "tests" / "stress" / "modal_sweep"))
os.environ.setdefault("KICAD_SWEEP_WITH_KICAD", "1")

import modal  # noqa: E402
import modal_app as M  # noqa: E402

app = modal.App(os.environ.get("KICAD_SWEEP_NAME", "kicad-regrade-arm"))


def _is_routing_key(k):
    return k.startswith("diff_") or k == "replay_rc"


@app.function(image=M.image, memory=4096, timeout=3 * 3600,
              retries=modal.Retries(max_retries=2),
              volumes={M.CORPUS: M.corpus_vol, M.RESULTS: M.results_vol})
def regrade_one(task: dict) -> dict:
    import sweep_lib
    t0 = time.time()
    src, dst_arm, s, b = task["src_arm"], task["dst_arm"], task["set"], task["board"]
    adir = Path(M.RESULTS) / src / "artifacts" / f"{s}__{b}"
    orig = json.loads((Path(M.RESULTS) / src / f"{s}__{b}.json").read_text())
    orig = orig[0] if isinstance(orig, list) else orig
    art_man = adir / "redo_commands.sh"
    man_txt = art_man.read_text()

    # Stage the corpus at its RECORDED path, as _replay_one does.
    root = Path(sweep_lib.recorded_corpus_prefix(man_txt))
    set_dir = root / f"runs_{s}"
    set_dir.mkdir(parents=True, exist_ok=True)
    for stale in list(set_dir.iterdir()):
        if stale.is_dir():
            shutil.rmtree(stale, ignore_errors=True)
    shutil.copytree(Path(M.CORPUS) / f"runs_{s}" / b, set_dir / b)
    corpus_man = (set_dir / b / "redo_commands.sh").read_text()
    shutil.copy2(art_man, set_dir / b / "redo_commands.sh")
    base = re.match(r"(set\d+[a-z]*)", s)
    names = {f"boards_unrouted_{s}", f"boards_{s}"}
    if base:
        names |= {f"boards_unrouted_{base.group(1)}", f"boards_{base.group(1)}"}
    for d in sorted(names):
        srcd, lnk = Path(M.CORPUS) / d, root / d
        if srcd.exists() and not lnk.exists():
            lnk.symlink_to(srcd)

    wave = Path(f"/tmp/rg__{dst_arm}__{s}__{b}") / s
    shutil.rmtree(wave.parent, ignore_errors=True)
    (wave / b).mkdir(parents=True)
    for f in adir.iterdir():
        if f.is_file() and f.suffix in (".kicad_pcb", ".kicad_pro", ".kicad_dru", ".kicad_prl"):
            shutil.copy2(f, wave / b / f.name)

    proc = subprocess.run(
        [sys.executable, f"{M.REPO}/tests/stress/ab_replay_grade.py",
         "--regrade", str(wave), "--set", str(set_dir)],
        capture_output=True, text=True, timeout=3 * 3600 - 300)
    dest = Path(M.RESULTS) / dst_arm
    dest.mkdir(parents=True, exist_ok=True)
    new = {}
    summ = wave / "summary.json"
    if summ.exists():
        new = next((r for r in json.loads(summ.read_text()) if r.get("board") == b), {})
    if not new or not new.get("chain_complete") or new.get("final") != orig.get("final"):
        (dest / f"{s}__{b}.log").write_text(
            f"regrade failed rc={proc.returncode} new={json.dumps(new)[:2000]}\n"
            f"orig final={orig.get('final')}\n--- stdout\n{proc.stdout[-6000:]}\n"
            f"--- stderr\n{proc.stderr[-6000:]}\n")
        M.results_vol.commit()
        return {"set": s, "board": b, "ok": False}

    row = dict(orig)
    graded = {k: v for k, v in new.items() if not _is_routing_key(k)}
    row["pre_regrade"] = {k: orig.get(k) for k in graded}
    row.update(graded)
    row.update({"arm": dst_arm, "regraded_from": src, "regraded_by": M.GIT_SHA,
                "regrade_wall_s": round(time.time() - t0, 1),
                "regrade_manifest_matches_corpus": corpus_man == man_txt})
    (dest / f"{s}__{b}.json").write_text(json.dumps(row, indent=1))
    M.results_vol.commit()
    return {"set": s, "board": b, "ok": True}


@app.local_entrypoint()
def main(src_arm: str, dst_arm: str, boards: str = ""):
    """boards: comma-separated board names to restrict to (smoke)."""
    have_art, have_dst = [], set()
    for e in M.results_vol.listdir(f"/{src_arm}/artifacts"):
        name = e.path.rstrip("/").split("/")[-1]
        if "__" in name:
            have_art.append(name)
    if not have_art:
        raise SystemExit(f"no kept boards under /{src_arm}/artifacts -- was the arm "
                         f"launched with artifacts ON?")
    try:
        for e in M.results_vol.listdir(f"/{dst_arm}"):
            p = e.path.split("/")[-1]
            if p.endswith(".json"):
                have_dst.add(p[:-5])
    except Exception:
        pass  # a fresh dst arm has no directory yet
    want = {x.strip() for x in boards.split(",") if x.strip()}
    tasks = []
    for name in sorted(have_art):
        s, b = name.split("__", 1)
        if (want and b not in want) or name in have_dst:
            continue
        tasks.append({"src_arm": src_arm, "dst_arm": dst_arm, "set": s, "board": b})
    print(f"{len(have_art)} kept boards in {src_arm}; {len(have_dst)} already regraded; "
          f"{len(tasks)} to run -> {dst_arm}  (grader image {M.GIT_SHA})")
    ok = bad = 0
    for r in regrade_one.map(tasks, return_exceptions=True, order_outputs=False):
        if isinstance(r, dict) and r.get("ok"):
            ok += 1
        else:
            bad += 1
            print(f"  FAILED: {r}")
        if (ok + bad) % 10 == 0:
            print(f"  {ok + bad}/{len(tasks)} done ({bad} failed)", flush=True)
    print(f"DONE ok={ok} failed={bad}")
