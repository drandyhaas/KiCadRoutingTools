#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set15/covg_patchclamp.kicad_pcb step1_planes.kicad_pcb --nets GND GND +5V --plane-layers In1.Cu In4.Cu In2.Cu --clearance-ceiling 0.2 --via-size 0.6 --via-drill 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fan_IC1.kicad_pcb --component IC1 --nets '*' '!GND' '!+5V' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fan_IC1.kicad_pcb --component IC1 --nets '*' '!GND' '!+5V' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fan_IC1.kicad_pcb --output step2b_fan_U28.kicad_pcb --component U28 --nets '*' '!GND' '!+5V' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fan_U28.kicad_pcb --output step2c_fan_U30.kicad_pcb --component U30 --nets '*' '!GND' '!+5V' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_fan_U30.kicad_pcb --output step2d_fan_U32.kicad_pcb --component U32 --nets '*' '!GND' '!+5V' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2d_fan_U32.kicad_pcb --output step2e_fan_U34.kicad_pcb --component U34 --nets '*' '!GND' '!+5V' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2e_fan_U34.kicad_pcb --output step2f_fan_U5.kicad_pcb --component U5 --nets '*' '!GND' '!+5V' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2f_fan_U5.kicad_pcb --output step2g_fan_U58.kicad_pcb --component U58 --nets '*' '!GND' '!+5V' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2g_fan_U58.kicad_pcb --output step2h_fan_U13.kicad_pcb --component U13 --nets '*' '!GND' '!+5V' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2h_fan_U13.kicad_pcb step2i_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2i_capclear.kicad_pcb step3_diff.kicad_pcb --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --via-size 0.6 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +5V +15V -15V +3V3 +1V8 +2V5 /power/16V5 /power/-16V5 --power-nets-widths 0.5 0.5 0.5 0.5 0.35 0.3 0.3 0.35 0.35 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4b_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.15 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.2 --power-nets GND +5V +15V -15V +3V3 +1V8 +2V5 /power/16V5 /power/-16V5 --power-nets-widths 0.3 0.3 0.3 0.3 0.2 0.2 0.2 0.2 0.2 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4b_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.15 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.2 --power-nets GND +5V +15V -15V +3V3 +1V8 +2V5 /power/16V5 /power/-16V5 --power-nets-widths 0.3 0.3 0.3 0.3 0.2 0.2 0.2 0.2 0.2 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/covg_patchclamp
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4b_route.kicad_pcb step4c_retry.kicad_pcb --nets +15V --clearance-ceiling 0.2 --track-width 0.15 --via-size 0.5 --via-drill 0.25 --hole-to-hole-clearance 0.2 --power-nets +15V --power-nets-widths 0.2 --max-ripup 5
