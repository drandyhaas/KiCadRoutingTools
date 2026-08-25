#!/bin/bash
# Run a FULL parameter sweep on Modal: every arm x every board in the given sets.
#
#   bash tests/stress/modal_sweep/run_sweep.sh <arms.json> [sets]
#
#   bash tests/stress/modal_sweep/run_sweep.sh \
#        tests/stress/modal_sweep/arms.track_proximity.json
#
# Defaults to sets 1-10 (150 boards). It ALWAYS prints the plan -- task count,
# estimated CPU-hours, the wall-clock floor -- before spending anything, because
# the cost scales with arms x boards and the floor is set by ONE long board that
# no amount of parallelism beats.
#
# Env:
#   SWEEP_SETS=set1,set2      override the sets (or pass as $2)
#   SWEEP_UPLOAD=1            (re)populate the corpus volume first -- a ~2.2 GB
#                             one-off; only needed the first time or when the
#                             corpus changes
#   SWEEP_DRY=1               stop after the plan, spend nothing
#   SWEEP_OUT=/path.json      where to write the result (default: stress dir)
#   SWEEP_EXCLUDE=a,b         skip these boards -- the wall-clock lever. A
#                             handful of multi-hour monsters can be half a
#                             sweep's cost and ALL of its floor (sets 11-20:
#                             4 boards = 50% of the CPU-h and the whole 11 h
#                             floor), and dropping them barely moves a verdict
#                             drawn from ~150 boards.
#   KICAD_SWEEP_DIRTY=1       ship the WORKING TREE instead of a clean HEAD
#                             checkout -- for deliberately sweeping an
#                             uncommitted engine change. Otherwise the image is
#                             pinned to a commit, which is what keeps `baseline`
#                             meaningful and lets you keep editing while it runs.
#
# RUN smoke_test.sh FIRST. It costs cents and catches the failure modes that a
# green-looking full sweep will not show you.
set -euo pipefail

SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARMS="${1:?usage: run_sweep.sh <arms.json> [sets]}"
SETS="${2:-${SWEEP_SETS:-set1,set2,set3,set4,set5,set6,set7,set8,set9,set10}}"
OUT="${SWEEP_OUT:-}"
EXCLUDE="${SWEEP_EXCLUDE:-}"
EXARG=(); [ -n "$EXCLUDE" ] && EXARG=(--exclude "$EXCLUDE")
# Every use below expands as ${EXARG[@]+"${EXARG[@]}"}, never as the bare
# "${EXARG[@]}": macOS ships bash 3.2, where expanding an EMPTY array that way
# is an "unbound variable" error under `set -u`. Every earlier run happened to
# set SWEEP_EXCLUDE, so the plain form worked until the first launch that did
# not -- and it died AFTER the priced plan printed, i.e. at the moment of
# spending, which is the worst place to learn about it.

[ -f "$ARMS" ] || { echo "no such arms file: $ARMS" >&2; exit 1; }

# Name the Modal app after the arms file so runs are distinguishable on the
# dashboard: arms.586.json -> kicad-sweep-586, arms.holdout.json -> kicad-sweep-holdout.
_stem="$(basename "$ARMS" .json)"; _stem="${_stem#arms.}"; _stem="${_stem#arms_}"
export KICAD_SWEEP_NAME="${KICAD_SWEEP_NAME:-kicad-sweep-${_stem}}"
# Exit 3 means specifically "no baseline arm"; any other non-zero is a malformed
# file whose own error was already printed. Collapsing the two would report
# "no baseline" for e.g. duplicate arm names and send you hunting the wrong bug.
set +e
python3 -c "import json,sys; a=json.load(open(sys.argv[1])); \
  assert isinstance(a,list) and a, 'arms file must be a non-empty list'; \
  names=[x['name'] for x in a]; \
  assert len(set(names))==len(names), 'duplicate arm names: %r'%names; \
  print('arms:', ', '.join(names)); \
  sys.exit(0 if any(not x.get('defaults') and not x.get('env') and not x.get('manifest_sed') for x in a) else 3)" \
  "$ARMS"
_rc=$?
set -e
case "$_rc" in
  0) ;;
  3) echo "REFUSING: no baseline arm (one with no overrides)." >&2
     echo "Without an anchor you cannot tell a real effect from corpus/commit drift." >&2
     exit 1 ;;
  *) echo "REFUSING: malformed arms file (see the error above)." >&2; exit 1 ;;
esac

if [ "${SWEEP_UPLOAD:-0}" = "1" ]; then
  echo "== populating the corpus volume (~2.2 GB, one-off)"
  # No --exclude here on purpose: the corpus volume is shared across runs, so
  # a board this ONE sweep skips for wall-clock reasons must still be uploaded
  # for the next one.
  python3 "$SELF/upload_corpus.py" --sets "$SETS"
  echo
fi

echo "== plan (spends nothing)"
modal run "$SELF/modal_app.py" --arms "$ARMS" --sets "$SETS" ${EXARG[@]+"${EXARG[@]}"} --dry-run

if [ "${SWEEP_DRY:-0}" = "1" ]; then
  echo
  echo "SWEEP_DRY=1 -- stopping after the plan."
  exit 0
fi

echo
echo "== running the sweep"
if [ -n "$OUT" ]; then
  modal run "$SELF/modal_app.py" --arms "$ARMS" --sets "$SETS" ${EXARG[@]+"${EXARG[@]}"} --out "$OUT"
  RESULT="$OUT"
else
  modal run "$SELF/modal_app.py" --arms "$ARMS" --sets "$SETS" ${EXARG[@]+"${EXARG[@]}"} | tee /tmp/sweep_run.log
  RESULT="$(grep -oE '/[^ ]*sweep_[0-9]+\.json' /tmp/sweep_run.log | tail -1)"
fi

echo
echo "== verifying the result is trustworthy (not just green)"
# NOT --require-all-complete: a real sweep legitimately has boards whose chain
# breaks. Those are reported; the hard failures are "arms are identical" and
# "no provenance", which mean the sweep answered nothing.
python3 "$SELF/check_sweep.py" "$RESULT"
