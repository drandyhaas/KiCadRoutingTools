#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/schoko
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/schoko
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/schoko.kicad_pcb step1_planes.kicad_pcb --nets GND /PWR3V3 --plane-layers In1.Cu In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/schoko
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!/PWR3V3' --diff-pairs '/DVI_CK*' '/DVI_D0*' '/DVI_D1*' '/DVI_D2*' '/USB0_D*' '/USB1_D*' --diff-pair-gap 0.15 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance 0.125 --track-width 0.125 --via-size 0.45 --via-drill 0.2 --plane-net-layers GND:In1.Cu /PWR3V3:In2.Cu --output step1b_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/schoko
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!/PWR3V3' --diff-pairs '/DVI_CK*' '/DVI_D0*' '/DVI_D1*' '/DVI_D2*' '/USB0_D*' '/USB1_D*' --diff-pair-gap 0.15 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance 0.125 --track-width 0.1 --via-size 0.4 --via-drill 0.2 --plane-net-layers GND:In1.Cu /PWR3V3:In2.Cu --output step1b_fanout_u1_v2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/schoko
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!/PWR3V3' --diff-pairs '/DVI_CK*' '/DVI_D0*' '/DVI_D1*' '/DVI_D2*' '/USB0_D*' '/USB1_D*' --diff-pair-gap 0.15 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance 0.125 --track-width 0.0762 --via-size 0.4 --via-drill 0.2 --plane-net-layers GND:In1.Cu /PWR3V3:In2.Cu --output step1b_fanout_u1_v3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/schoko
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_fanout_u1_v2.kicad_pcb step1c_capclear.kicad_pcb --clearance 0.125
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/schoko
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1c_capclear.kicad_pcb step2a_diffpairs.kicad_pcb --nets '/CK*' '/D0*' '/D1*' '/D2*' '/DVI_*' '/USB0*' '/USB1*' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.4 --via-drill 0.2 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/schoko
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --clearance-ceiling 0.125 --track-width 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND /PWR3V3 /PWR1V1 /PWR5V0 --power-nets-widths 0.3 0.3 0.3 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 6.0 2.5 1.0
