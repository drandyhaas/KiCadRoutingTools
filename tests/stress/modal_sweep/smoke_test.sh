#!/bin/bash
# End-to-end Modal smoke test: 2 boards x 2 arms, self-verifying.
#
#   bash tests/stress/modal_sweep/smoke_test.sh
#
# Run this BEFORE any real sweep. It costs a few cents and exercises every
# moving part -- image build, corpus volume, the absolute-path remap, parameter
# patching, container reuse, grading, result harvesting -- then CHECKS the run
# rather than trusting a green exit. During bring-up a fully green run turned
# out to have both arms executing identical routing constants; check_sweep.py
# exists to catch exactly that.
#
# Prereq (interactive, once):  pip install modal && modal setup
set -euo pipefail
export KICAD_SWEEP_NAME="${KICAD_SWEEP_NAME:-kicad-sweep-smoke}"

SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SELF/../../.." && pwd)"
BOARDS="${SMOKE_BOARDS:-stm32_rfm95_lora,cc1101_rf_module}"
SET="${SMOKE_SET:-set9}"
OUT="${SMOKE_OUT:-/tmp/sweep_smoke.json}"

echo "== 1/3 upload the ${SET} slice (${BOARDS})"
python3 "$SELF/upload_corpus.py" --sets "$SET" --boards "$BOARDS"

echo
echo "== 2/3 run 2 arms x 2 boards on Modal"
# --out pins the result path so step 3 does not have to guess the newest file.
modal run "$SELF/modal_app.py" \
    --arms "$SELF/arms.smoke.json" \
    --sets "$SET" --boards "$BOARDS" --out "$OUT"

echo
echo "== 3/3 verify the run is trustworthy (not just green)"
python3 "$SELF/check_sweep.py" "$OUT" --require-all-complete
