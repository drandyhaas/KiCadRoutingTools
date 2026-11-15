#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/fsae_bms_slave
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/fsae_bms_slave
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/fsae_bms_slave
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set20/fsae_bms_slave.kicad_pcb step1_planes.kicad_pcb --nets Cell0- Cell0- Cell0- --plane-layers B.Cu In1.Cu In2.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/fsae_bms_slave
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_ic120.kicad_pcb --component IC120 --nets '*' '!Cell0-' --clearance 0.1 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/fsae_bms_slave
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/fsae_bms_slave
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_ic120.kicad_pcb step3_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/fsae_bms_slave
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/fsae_bms_slave
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/fsae_bms_slave
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpairs.kicad_pcb --nets IN_P1 IN_N1 OUT_P1 OUT_N1 'Net-(TR101-RD+)' 'Net-(TR101-RD-)' 'Net-(TR101-TD+)' 'Net-(TR101-TD-)' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/fsae_bms_slave
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/fsae_bms_slave
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets Cell0- VREG VREF2 V+ /Power/VIN 'Net-(IC201-INTVCC)' --power-nets-widths 0.3 0.3 0.25 0.3 0.4 0.25 --grid-step 0.1
