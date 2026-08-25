#!/bin/bash
# Score ONE engine commit across the stress corpus, on the cloud.
#
#   bash tests/stress/corpus_bisect.sh <sha> <tag>
#   bash tests/stress/corpus_bisect.sh 46e4cd6 p09
#
# Bisecting a routing regression one board at a time does not work: per-board
# run-to-run spread is +-2..3 nets, so a real effect of that size is invisible
# and a noise excursion looks like a finding (this cost a whole session's worth
# of wrong attributions). Scoring each candidate commit over ~130 boards makes
# the deltas readable. ~$1 and ~30 min per commit, so 5-6 points bracket a
# 38-commit range for under $10.
#
# Ships TODAY's harness with the OLD engine: a worktree at current HEAD (the
# driver and modal_app postdate the range being bisected), then py_router/ and
# rust_router/ are checked out AT THE BISECT COMMIT.
#
# Four things this gets right, each of which silently produced garbage first:
#
#   1. `git checkout <sha> -- path` does NOT delete files the old commit lacks,
#      leaving a HYBRID engine. Hence the rm -rf, and the diff assertion that
#      aborts before spending anything.
#   2. KICAD_SWEEP_DIRTY=1 is REQUIRED. The default ships a clean HEAD, which
#      would score HEAD N times over while looking perfectly healthy.
#   3. The image's router download must be PINNED to the release whose asset
#      matches the commit's crate. #615 made build_router skip the prebuilt
#      whenever rust_router/ differs from main -- which a bisect worktree always
#      does -- sending it to a source build the image cannot do (no Rust
#      toolchain). An explicit --tag is honored as-given. Crate 0.20.1 maps to
#      release v0.20.2, because v0.20.1's asset is a stale 0.20.0 build.
#   4. core64_logic is excluded: it replays in 4 min historically and now never
#      terminates (issue #625), and one such board holds every arm at N-1.
#
# Compare arms only on boards where they replayed an IDENTICAL chain (same
# nets_total and step count) -- see the note in modal_app.py about a reused
# container leaking the previous board into the next grade (fixed, but older
# result rows still carry it).
set -u
SHA=${1:?usage: corpus_bisect.sh <sha> <tag>}
TAG=${2:?usage: corpus_bisect.sh <sha> <tag>}
REPO=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
S=${SCRATCH:-/tmp}/kicad-corpus-bisect
ST=${STRESS_DIR:-$HOME/Documents/kicad_stress_test}
SETS=${SETS:-set10-set19}
LIMIT=${LIMIT:-130}
# BOARDS: score a FIXED board list instead of "the N cheapest". A bisect
# wants the boards that actually MOVED -- --limit picks by cost, which is
# uncorrelated with where a regression lives, so without this a 5-point
# ladder can spend its whole budget on boards that never differed.
BOARDS=${BOARDS:-}
mkdir -p "$S"
WT=$S/bx_$TAG
rm -rf "$WT"; git -C "$REPO" worktree prune
git -C "$REPO" worktree add -q -f --detach "$WT" HEAD || exit 1
rm -rf "$WT/py_router" "$WT/rust_router"
git -C "$WT" checkout "$SHA" -- py_router rust_router || exit 1
if ! git -C "$WT" diff --quiet "$SHA" -- py_router; then
  echo "  $TAG: engine checkout MISMATCH -- aborting before spending"; exit 1
fi
CRATE=$(git -C "$WT" show "$SHA:rust_router/Cargo.toml" | grep -m1 '^version' | tr -d ' "' | cut -d= -f2)
BUILD_TAG=""
if [ -n "$CRATE" ]; then
  REL=$CRATE; [ "$CRATE" = "0.20.1" ] && REL=0.20.2
  # Pin the image's prebuilt to this commit's crate release. This used to REWRITE
  # modal_app.py's build command from here; it now rides an env var that
  # modal_app._build_step() reads client-side, because a rewrite that silently
  # no-ops launches an arm that builds unpinned and dies 30 minutes later (#615
  # skips the prebuilt in a bisect worktree, and the image has no Rust). Keep the
  # loud guard: if the knob is gone, refuse to launch rather than spend on an arm
  # that cannot build.
  if ! grep -q "KICAD_SWEEP_BUILD_TAG" "$WT/tests/stress/modal_sweep/modal_app.py"; then
    echo "  $TAG: modal_app.py has no KICAD_SWEEP_BUILD_TAG support -- refusing to launch unpinned"
    exit 1
  fi
  BUILD_TAG="v$REL"
  echo "  $TAG: crate $CRATE -> image build pinned to release v$REL"
fi
echo "  $TAG: engine=$SHA harness=HEAD  (py_router matches $SHA)"
cd "$WT"
# Retry the LAUNCH, not just the build. Modal rate-limits app CREATION, and a
# bisect ladder starts several points back to back -- launching six ~2s apart
# had three rejected outright with "App create rate limit exceeded". The script
# used to exit rc=1 there, backgrounded, so a half-launched ladder was
# indistinguishable from a healthy one until someone read six separate logs (or
# noticed the missing apps on the dashboard). Same transient class as the flaky
# prebuilt fetch the image build already retries.
_launch() {
  KICAD_SWEEP_BUILD_TAG="$BUILD_TAG" \
  KICAD_SWEEP_DIRTY=1 caffeinate -dimsu nohup python3 tests/stress/cloud_replay_sets.py \
     --sets "$SETS" --arm "bis${TAG}" --label "bis${TAG}" --limit "$LIMIT" \
     ${BOARDS:+--boards "$BOARDS"} \
     --exclude core64_logic --out "$ST/cloud_bis${TAG}" --only plan,run \
     > "$S/bis_$TAG.log" 2>&1
}
(
  for _try in 1 2 3 4; do
    _launch && break
    if grep -q "rate limit exceeded" "$S/bis_$TAG.log" 2>/dev/null; then
      echo "  $TAG: app-create rate limited, retry $_try in $((_try*90))s" >> "$S/bis_$TAG.log"
      sleep $((_try * 90))
      continue
    fi
    break            # a non-rate-limit failure is real; leave it in the log
  done
) &
disown
echo "  $TAG: launched -> $S/bis_$TAG.log  (auto-retries an app-create rate limit)"
