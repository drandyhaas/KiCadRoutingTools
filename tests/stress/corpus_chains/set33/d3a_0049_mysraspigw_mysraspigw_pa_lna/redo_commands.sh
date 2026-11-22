#!/bin/bash
# Recorded chain: one autoroute.py run (cloud wave autoroute-kc_f0e994a33,
# 2026-11-19, the whole board planned and routed by autoroute.py; no LLM
# session recorded this board).
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set33/d3a_0049_mysraspigw_mysraspigw_pa_lna
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/autoroute.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set33/d3a_0049_mysraspigw_mysraspigw_pa_lna.kicad_pcb autoroute.kicad_pcb
