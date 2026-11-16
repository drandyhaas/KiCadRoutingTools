#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/openmux_potentio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/openmux_potentio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set13/openmux_potentio.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/openmux_potentio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/openmux_potentio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VDD VSS AGND DGND --power-nets-widths 0.4 0.4 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/openmux_potentio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VDD VSS AGND DGND --power-nets-widths 0.4 0.4 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/openmux_potentio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VDD VSS AGND DGND --power-nets-widths 0.4 0.4 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/openmux_potentio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VDD VSS AGND DGND --power-nets-widths 0.4 0.4 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/openmux_potentio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VDD VSS AGND DGND --power-nets-widths 0.4 0.4 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/openmux_potentio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_planes.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND VDD VSS AGND DGND --power-nets-widths 0.4 0.4 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/openmux_potentio
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step2b_retry.kicad_pcb --nets DGND AGND 'CON1_1{slash}D3 -> A0' 'CON1_2{slash}D2 -> A1' 'CON1_3{slash}D3 -> A3' CE_Pin RE_Pin WE_Pin --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.3 --via-drill 0.15 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 5
