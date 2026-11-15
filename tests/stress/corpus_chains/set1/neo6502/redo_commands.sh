#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set1/neo6502.kicad_pcb -c U2 --clearance 0.1 --width 0.15 --grid-step 0.05 -o step1_fanout_U2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_fanout_U2.kicad_pcb step2_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2_clearance.kicad_pcb step3_diffpairs.kicad_pcb '*' --track-width 0.2 --diff-pair-gap 0.508 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step3_diffpairs.kicad_pcb step4_planes.kicad_pcb --nets GND --plane-layers B.Cu --via-size 0.7 --via-drill 0.4 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb '*' --clearance-ceiling 0.2 --track-width 0.254 --via-size 0.7 --via-drill 0.4 --hole-to-hole-clearance 0.25 --max-ripup 5 --power-nets GND +3.3V +5V_USB /+1V1 --power-nets-widths 0.5 0.4 0.5 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb '*' --clearance-ceiling 0.2 --track-width 0.254 --via-size 0.7 --via-drill 0.4 --hole-to-hole-clearance 0.25 --power-nets GND +3.3V +5V_USB /+1V1 --power-nets-widths 0.5 0.4 0.5 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5_route.kicad_pcb '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.7 --via-drill 0.4 --hole-to-hole-clearance 0.25 --power-nets GND +3.3V +5V_USB /+1V1 --power-nets-widths 0.4 0.3 0.4 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_planes.kicad_pcb step5a_gnd.kicad_pcb GND --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.7 --via-drill 0.4 --hole-to-hole-clearance 0.25 --power-nets GND --power-nets-widths 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5a_gnd.kicad_pcb step5b_route.kicad_pcb '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.7 --via-drill 0.4 --hole-to-hole-clearance 0.25 --power-nets GND +5V_USB --power-nets-widths 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5a_gnd.kicad_pcb step5b1_route.kicad_pcb 'Net-(C17-Pad1)' 'Net-(AUDIO1-Pad1)' /BUZZER /SPK 'Net-(D1-Pad2)' 'Net-(PWR_LED1-Pad1)' 'Net-(C15-Pad1)' +5V_USB /RESB '/~{BOOTSEL}' 'Net-(R10-Pad2)' 'Net-(R12-Pad2)' 'Net-(R15-Pad1)' /IRQB /NMIB /BE /SOB /GPIO3 /GPIO2 /GPIO1 --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.7 --via-drill 0.4 --hole-to-hole-clearance 0.25 --power-nets +5V_USB --power-nets-widths 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5b1_route.kicad_pcb step5b2_route.kicad_pcb /GPIO0 /GPIO7 /GPIO6 /GPIO5 /GPIO4 'Net-(L1-Pad1)' /A8 /A9 /A10 /A11 /A12 /A13 /A14 /A15 /D7 /D6 /D5 /D4 /D3 /D2 --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.7 --via-drill 0.4 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5b2_route.kicad_pcb step5b3_route.kicad_pcb /D1 /D0 /A0 /A1 /A2 /A3 /A4 /A5 /A6 /A7 /RDY /SYNC /VPB /MLB 'Net-(DVI1-Pad18)' 'Net-(D1-Pad1)' 'Net-(DVI1-Pad13)' 'Net-(DVI1-Pad15)' 'Net-(DVI1-Pad16)' 'Net-(DVI1-Pad19)' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.7 --via-drill 0.4 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5b2_route.kicad_pcb step5b3a_route.kicad_pcb /A0 /A1 /A2 /A3 /A4 /A5 /A6 /A7 /RDY /SYNC /VPB /MLB 'Net-(DVI1-Pad18)' 'Net-(D1-Pad1)' 'Net-(DVI1-Pad13)' 'Net-(DVI1-Pad15)' 'Net-(DVI1-Pad16)' 'Net-(DVI1-Pad19)' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.7 --via-drill 0.4 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5b3a_route.kicad_pcb step5b3b_route.kicad_pcb /D0 /D1 --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.7 --via-drill 0.4 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5b3b_route.kicad_pcb step5c_gnd.kicad_pcb GND --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.7 --via-drill 0.4 --hole-to-hole-clearance 0.25 --power-nets GND --power-nets-widths 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1/neo6502
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5c_gnd.kicad_pcb step5d_fix.kicad_pcb '*' --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.25 --rip-existing-nets '*' --power-nets GND --power-nets-widths 0.4
