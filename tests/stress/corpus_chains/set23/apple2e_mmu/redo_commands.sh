#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/apple2e_mmu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/apple2e_mmu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set23/apple2e_mmu.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.11
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/apple2e_mmu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/apple2e_mmu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u2.kicad_pcb --component U2 --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+3V3' --clearance 0.11 --track-width 0.15 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/apple2e_mmu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/apple2e_mmu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u2.kicad_pcb --component U2 --nets '*' '!GND' '!+3V3' --clearance 0.11 --width 0.15 --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/apple2e_mmu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u2_try2.kicad_pcb --component U2 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.1 --escape-method underpad --allow-via-in-pad --via-size 0.2 --via-drill 0.1 --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/apple2e_mmu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout_u2_try2.kicad_pcb step2b_cap_clear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/apple2e_mmu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/apple2e_mmu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2b_cap_clear.kicad_pcb step3_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.11 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V --power-nets-widths 0.3 0.3 0.3 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/apple2e_mmu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2b_cap_clear.kicad_pcb step3_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.11 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V --power-nets-widths 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/apple2e_mmu
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2b_cap_clear.kicad_pcb step3_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.11 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V --power-nets-widths 0.3 0.3 0.3
