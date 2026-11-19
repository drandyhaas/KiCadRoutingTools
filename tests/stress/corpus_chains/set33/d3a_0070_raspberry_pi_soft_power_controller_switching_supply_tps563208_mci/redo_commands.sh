#!/bin/bash
# Recorded chain: one autoroute.py run (cloud wave autoroute-kc_f0e994a33,
# 2026-11-19, the whole board planned and routed by autoroute.py; no LLM
# session recorded this board).
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set33/d3a_0070_raspberry_pi_soft_power_controller_switching_supply_tps563208_mci
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/autoroute.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set33/d3a_0070_raspberry_pi_soft_power_controller_switching_supply_tps563208_mci.kicad_pcb autoroute.kicad_pcb
