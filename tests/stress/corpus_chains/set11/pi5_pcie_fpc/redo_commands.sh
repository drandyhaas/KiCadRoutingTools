#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/pi5_pcie_fpc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/pi5_pcie_fpc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set11/pi5_pcie_fpc.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/pi5_pcie_fpc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/pi5_pcie_fpc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb --nets 'PCIE_CLK*' 'PCIE_RX0*' 'PCIE_TX0*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2205 --diff-pair-gap 0.2032 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/pi5_pcie_fpc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/pi5_pcie_fpc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.25 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +12V +3.3VA +5V --power-nets-widths 0.4 0.4 0.5 0.3 0.4
