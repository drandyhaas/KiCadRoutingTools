#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set9/usbc_liion_charger.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.127
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_u2.kicad_pcb --component U2 --clearance 0.127 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u2.kicad_pcb --component U2 --clearance 0.127 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u2.kicad_pcb --output step2b_fanout_u3.kicad_pcb --component U3 --clearance 0.127 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_u3.kicad_pcb step2d_capclear.kicad_pcb --clearance 0.127
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2d_capclear.kicad_pcb --output step3_diffpairs.kicad_pcb --nets 'Net-(U3-Sense+)' 'Net-(U3-Sense-)' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.127 --no-gnd-vias --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb --output step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.25 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --hole-to-hole-clearance 0.2 --power-nets GND /Charge-IC/VIN /Charge-IC/BAT '/Boost-Converter/V_{IN}' '/Boost-Converter/V_{OUT}' '/Boost-Converter/INTV_{CC}' 'Net-(Q4-D)' --power-nets-widths 0.3 0.6 0.6 0.6 0.6 0.3 0.6
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set9/usbc_liion_charger
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_route.kicad_pcb --output step5_fix_sense.kicad_pcb --nets 'Net-(U3-Sense+)' 'Net-(U3-Sense-)' --force-reroute --clearance-ceiling 0.127 --track-width 0.1 --routing-clearance-margin 1.6 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --hole-to-hole-clearance 0.2
