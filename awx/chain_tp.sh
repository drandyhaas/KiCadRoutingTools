#!/bin/bash
# The TWO-PAGE chain: plan + escapes by page (fanout_from_plan
# --order-model --two-page), source applied, ribbon braid (TWO_PAGE=1).
# usage: chain_tp.sh TAG K [K...] [-- fanout options]
cd "$(dirname "$0")"
export TWO_PAGE=1
export SOURCE=1
export DIRS=${DIRS:-left,down}
export PLAN_OPTS="--order-model --two-page"
bash chain_k.sh "$@"
