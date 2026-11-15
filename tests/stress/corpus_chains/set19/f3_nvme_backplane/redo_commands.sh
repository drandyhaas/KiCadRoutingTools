#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/f3_nvme_backplane
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/f3_nvme_backplane
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/f3_nvme_backplane
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set19/f3_nvme_backplane.kicad_pcb step1_planes.kicad_pcb --nets GND +5V --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.3 --track-width 0.3493 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/f3_nvme_backplane
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diffpairs.kicad_pcb '/PCIE_*' '/SATA1_*' '/SATA2_*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1516 --diff-pair-gap 0.127 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/f3_nvme_backplane
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/f3_nvme_backplane
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set19/f3_nvme_backplane
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_diffpairs.kicad_pcb step3_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.3493 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.3 --power-nets GND +5V +12V +3.3V /SATA1/SATA_12V /SATA1/SATA_5V /SATA2/SATA_12V /SATA2/SATA_5V --power-nets-widths 0.5 0.5 1.5 0.8 1.0 1.2 1.0 1.2
