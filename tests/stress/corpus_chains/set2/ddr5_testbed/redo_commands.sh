#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ddr5_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ddr5_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ddr5_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ddr5_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/ddr5_testbed.kicad_pcb step1_planes.kicad_pcb --nets GND VDDQ --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.25 --power-nets VDD 1V8_SYS 1V1_SYS VPP /EEPROM/VDD_LED --power-nets-widths 0.3 0.3 0.3 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ddr5_testbed
# [2026-08-19] removed dead command: '--timeout' is redo_stress_test.py's flag, never a routing-tool flag -- it exited rc=2 at record time too, and under 'set -e' it killed the whole replay at this line. Superseded by a clean command in this manifest.
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ddr5_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' '!VDDQ' --diff-pairs 'CLK_*' 'DQS_*' 'TDQS_*' --diff-pair-gap 0.1 --track-width 0.12 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --plane-net-layers GND:In1.Cu VDDQ:In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ddr5_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1.kicad_pcb step3_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ddr5_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ddr5_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpairs.kicad_pcb --nets CLK_P CLK_N DQS_P DQS_N TDQS_P TDQS_N --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --impedance 85 --via-size 0.25 --via-drill 0.15 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ddr5_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ddr5_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5_signal.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.12 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND VDDQ VDD 1V8_SYS 1V1_SYS VPP /EEPROM/VDD_LED --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.2 0.15 --length-match-group 'CA*' 'DQ*' --length-match-tolerance 0.5 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ddr5_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_signal.kicad_pcb step5b_retry.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.0762 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND VDDQ VDD 1V8_SYS 1V1_SYS VPP /EEPROM/VDD_LED --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.2 0.15 --length-match-group 'CA*' 'DQ*' --length-match-tolerance 0.5 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ddr5_testbed
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_signal.kicad_pcb step5b_retry.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.0762 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND VDDQ VDD 1V8_SYS 1V1_SYS VPP /EEPROM/VDD_LED --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.2 0.15 --length-match-group 'CA*' 'DQ*' --length-match-tolerance 0.5 --max-ripup 5 --no-bga-zone
