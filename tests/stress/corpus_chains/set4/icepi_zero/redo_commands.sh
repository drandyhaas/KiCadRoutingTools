#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set4/icepi_zero.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.14 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +5V +1V1 +2V5 --power-nets-widths 0.3 0.3 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --component U1 --output step2a_fanout_u1.kicad_pcb --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+3V3' --diff-pairs /GPDI/GPDI_CLK /GPDI/GPDI_D0 /GPDI/GPDI_D1 /GPDI/GPDI_D2 /USB/D0 /USB/D1 /USB/USB0_PULL_D /USB/USB1_PULL_D --diff-pair-gap 0.1 --track-width 0.15 --clearance 0.14 --via-size 0.32 --via-drill 0.15 --escape-method auto --plane-net-layers GND:In1.Cu +3V3:In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --component U1 --output step2a_fanout_u1_v2.kicad_pcb --layers F.Cu In1.Cu In2.Cu B.Cu --nets '*' '!GND' '!+3V3' --diff-pairs /GPDI/GPDI_CLK /GPDI/GPDI_D0 /GPDI/GPDI_D1 /GPDI/GPDI_D2 /USB/D0 /USB/D1 /USB/USB0_PULL_D /USB/USB1_PULL_D --diff-pair-gap 0.1 --track-width 0.0762 --clearance 0.09 --via-size 0.25 --via-drill 0.15 --escape-method auto --plane-net-layers GND:In1.Cu +3V3:In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2a_fanout_u1_v2.kicad_pcb step2b_capclear_u1.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_capclear_u1.kicad_pcb --component U9 --output step2c_fanout_u9.kicad_pcb --width 0.15 --clearance 0.14
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_fanout_u9.kicad_pcb step3_diffpairs.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_diffpairs.kicad_pcb step3b_diffpairs_retry.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.25 --no-gnd-vias --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3b_diffpairs_retry.kicad_pcb step4_signal.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.127 --clearance-ceiling 0.14 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.25 --power-nets GND +3V3 +5V +1V1 +2V5 --power-nets-widths 0.3 0.3 0.3 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3b_diffpairs_retry.kicad_pcb step4_signal.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.127 --clearance-ceiling 0.14 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.25 --power-nets GND +3V3 +5V +1V1 +2V5 --power-nets-widths 0.3 0.3 0.3 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set4/icepi_zero
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_signal.kicad_pcb step5_retry.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.127 --clearance-ceiling 0.14 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.25 --power-nets GND +3V3 +5V +1V1 +2V5 --power-nets-widths 0.3 0.3 0.3 0.3 0.3 --max-ripup 5
