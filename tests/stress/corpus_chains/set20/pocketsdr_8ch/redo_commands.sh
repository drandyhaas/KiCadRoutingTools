#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set20/pocketsdr_8ch.kicad_pcb --output step1_bga_u10.kicad_pcb --component U10 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance 0.09 --track-width 0.127 --via-size 0.45 --via-drill 0.3 --nets '*' '!GND' '!+3.3V' '!/+1.2V' --diff-pairs /USB2_DP /USB2_DM --diff-pair-gap 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1_bga_u10.kicad_pcb step1b_capopt_u10.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1b_capopt_u10.kicad_pcb --output step2_1_qfn_u1.kicad_pcb --component U1 --clearance 0.09 --width 0.127 --nets '*' '!GND' '!+3.3V'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_1_qfn_u1.kicad_pcb --output step2_2_qfn_u2.kicad_pcb --component U2 --clearance 0.09 --width 0.127 --nets '*' '!GND' '!+3.3V'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_2_qfn_u2.kicad_pcb --output step2_3_qfn_u3.kicad_pcb --component U3 --clearance 0.09 --width 0.127 --nets '*' '!GND' '!+3.3V'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_3_qfn_u3.kicad_pcb --output step2_4_qfn_u4.kicad_pcb --component U4 --clearance 0.09 --width 0.127 --nets '*' '!GND' '!+3.3V'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_4_qfn_u4.kicad_pcb --output step2_5_qfn_u5.kicad_pcb --component U5 --clearance 0.09 --width 0.127 --nets '*' '!GND' '!+3.3V'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_5_qfn_u5.kicad_pcb --output step2_6_qfn_u6.kicad_pcb --component U6 --clearance 0.09 --width 0.127 --nets '*' '!GND' '!+3.3V'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_6_qfn_u6.kicad_pcb --output step2_7_qfn_u7.kicad_pcb --component U7 --clearance 0.09 --width 0.127 --nets '*' '!GND' '!+3.3V'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_7_qfn_u7.kicad_pcb --output step2_8_qfn_u8.kicad_pcb --component U8 --clearance 0.09 --width 0.127 --nets '*' '!GND' '!+3.3V'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_8_qfn_u8.kicad_pcb --output step3_qfn_u16.kicad_pcb --component U16 --clearance 0.09 --width 0.127 --nets '*' '!GND' '!+3.3V'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_qfn_u16.kicad_pcb step4_diffpairs.kicad_pcb /USB2_DP /USB2_DM 'Net-(U16-TX+)' 'Net-(U16-TX-)' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step4_diffpairs.kicad_pcb step5_planes.kicad_pcb --nets GND GND --plane-layers In1.Cu In2.Cu --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.45 --via-drill 0.3 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --stitch-vias --power-nets GND --power-nets-widths 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.09 --track-width 0.127 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V /+1.2V /VBUS --power-nets-widths 0.25 0.3 0.25 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set20/pocketsdr_8ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_planes.kicad_pcb step6_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.09 --track-width 0.127 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V /+1.2V /VBUS --power-nets-widths 0.25 0.3 0.25 0.3
