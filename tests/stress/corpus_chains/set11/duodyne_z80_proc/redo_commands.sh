#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/duodyne_z80_proc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set11/duodyne_z80_proc.kicad_pcb step1_pour.kicad_pcb --nets GND --plane-layers B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/duodyne_z80_proc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND VCC +12V -12V /DMA/GND-DMA1 /DMA/GND-DMA2 --power-nets-widths 1.0 1.0 1.0 1.0 0.7 0.7 --layers F.Cu B.Cu --layer-costs 1.0 3.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/duodyne_z80_proc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND VCC +12V -12V /DMA/GND-DMA1 /DMA/GND-DMA2 --power-nets-widths 1.0 1.0 1.0 1.0 0.7 0.7 --layers F.Cu B.Cu --layer-costs 1.0 3.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/duodyne_z80_proc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND VCC +12V -12V /DMA/GND-DMA1 /DMA/GND-DMA2 --power-nets-widths 1.0 1.0 1.0 1.0 0.7 0.7 --layers F.Cu B.Cu --layer-costs 1.0 3.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/duodyne_z80_proc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND VCC +12V -12V /DMA/GND-DMA1 /DMA/GND-DMA2 --power-nets-widths 1.0 1.0 1.0 1.0 0.7 0.7 --layers F.Cu B.Cu --layer-costs 1.0 3.0 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/duodyne_z80_proc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' '!CPU-D1' --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND VCC +12V -12V /DMA/GND-DMA1 /DMA/GND-DMA2 --power-nets-widths 1.0 1.0 1.0 1.0 0.7 0.7 --layers F.Cu B.Cu --layer-costs 1.0 3.0 --max-ripup 3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/duodyne_z80_proc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' '!CPU-D1' --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND VCC +12V -12V /DMA/GND-DMA1 /DMA/GND-DMA2 --power-nets-widths 1.0 1.0 1.0 1.0 0.7 0.7 --layers F.Cu B.Cu --layer-costs 1.0 3.0 --max-ripup 3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/duodyne_z80_proc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step1_pour.kicad_pcb step2_route.kicad_pcb --nets '*' '!CPU-D1' --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND VCC +12V -12V /DMA/GND-DMA1 /DMA/GND-DMA2 --power-nets-widths 1.0 1.0 1.0 1.0 0.7 0.7 --layers F.Cu B.Cu --layer-costs 1.0 3.0 --max-ripup 1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/duodyne_z80_proc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step3_cpud1.kicad_pcb --nets CPU-D1 --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND VCC +12V -12V /DMA/GND-DMA1 /DMA/GND-DMA2 --power-nets-widths 1.0 1.0 1.0 1.0 0.7 0.7 --layers F.Cu B.Cu --layer-costs 1.0 3.0 --max-ripup 1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/duodyne_z80_proc
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_cpud1.kicad_pcb step4_gnd_fix.kicad_pcb --nets GND --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND VCC +12V -12V /DMA/GND-DMA1 /DMA/GND-DMA2 --power-nets-widths 1.0 1.0 1.0 1.0 0.7 0.7 --layers F.Cu B.Cu --layer-costs 1.0 3.0 --max-ripup 1
