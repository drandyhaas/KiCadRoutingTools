#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/fusb302_pd_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set9/fusb302_pd_dev.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.1524 --zone-clearance 0.1524 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/fusb302_pd_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2a_fan_u302.kicad_pcb -c U302 --clearance 0.1524 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/fusb302_pd_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fan_u302.kicad_pcb -o step2b_fan_u1.kicad_pcb -c U1 --clearance 0.1524 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/fusb302_pd_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fan_u1.kicad_pcb -o step2c_fan_u2.kicad_pcb -c U2 --clearance 0.1524 --grid-step 0.05
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2c_fan_u2.kicad_pcb step2d_capclear.kicad_pcb --clearance 0.1524
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/fusb302_pd_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2c_fan_u2.kicad_pcb step2d_capclear.kicad_pcb --clearance 0.1524
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/fusb302_pd_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/fusb302_pd_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2d_capclear.kicad_pcb step3_diff.kicad_pcb --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --grid-step 0.05 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/fusb302_pd_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/fusb302_pd_dev
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.1524 --track-width 0.1524 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --grid-step 0.05 --max-ripup 5 --no-bga-zone --power-nets GND +3V3 '/USB-C + PP/VBUS' 'Net-(J1-CC1)' 'Net-(J1-CC2)' 'Net-(U2-BOOT)' 'Net-(U2-NC{slash}SW)' 'Net-(U2-VCC)' --power-nets-widths 0.3 0.3 0.5 0.254 0.254 0.254 0.254 0.254
