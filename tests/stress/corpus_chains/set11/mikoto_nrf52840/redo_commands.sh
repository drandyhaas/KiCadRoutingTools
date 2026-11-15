#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/mikoto_nrf52840
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/mikoto_nrf52840
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/mikoto_nrf52840
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set11/mikoto_nrf52840.kicad_pcb step1_planes.kicad_pcb --nets GND VDD_NRF --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.12 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/mikoto_nrf52840
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!VDD_NRF' --escape-method underpad --allow-via-in-pad --clearance 0.12 --output step2_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/mikoto_nrf52840
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!VDD_NRF' --escape-method underpad --allow-via-in-pad --clearance 0.1 --via-size 0.25 --via-drill 0.15 --width 0.1 --output step2_fanout_u1b.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/mikoto_nrf52840
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/mikoto_nrf52840
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1b.kicad_pcb step3_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/mikoto_nrf52840
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/mikoto_nrf52840
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diff.kicad_pcb --nets USB_D+ USB_D- --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.15 --track-width 0.3 --diff-pair-gap 0.15 --via-size 0.45 --via-drill 0.3 --no-gnd-vias --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/mikoto_nrf52840
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3_capclear.kicad_pcb step4_diff_b.kicad_pcb --nets USB_D+ USB_D- --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1 --track-width 0.1 --diff-pair-gap 0.1 --via-size 0.25 --via-drill 0.15 --no-gnd-vias --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/mikoto_nrf52840
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set11/mikoto_nrf52840
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diff_b.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.12 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --power-nets GND VDD_NRF VDDH VBAT VBUS PPH_VCC --power-nets-widths 0.3 0.3 0.3 0.4 0.4 0.3 --grid-step 0.05 --no-bga-zone --max-ripup 5 --hole-to-hole-clearance 0.2
