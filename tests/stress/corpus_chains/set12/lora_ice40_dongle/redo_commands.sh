#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/lora_ice40_dongle
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set12/lora_ice40_dongle.kicad_pcb step1_planes.kicad_pcb --nets GND 3V3 --plane-layers In1.Cu In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/lora_ice40_dongle
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U2 --nets '*' '!GND' '!3V3' --output step2a_fanout_U2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/lora_ice40_dongle
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_U2.kicad_pcb --component U3 --nets '*' '!GND' '!3V3' --output step2b_fanout_U3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/lora_ice40_dongle
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_U3.kicad_pcb --component U6 --nets '*' '!GND' '!3V3' --output step2c_fanout_U6.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/lora_ice40_dongle
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_fanout_U6.kicad_pcb --component U1 --nets '*' '!GND' '!3V3' --output step2d_fanout_U1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/lora_ice40_dongle
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2d_fanout_U1.kicad_pcb step2e_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/lora_ice40_dongle
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/lora_ice40_dongle
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2e_capclear.kicad_pcb step3_diffpairs.kicad_pcb --nets /D+ /D- '/RF Circuitry/RFI_P' '/RF Circuitry/RFI_N' /USB_Interface/USB_D+ /USB_Interface/USB_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/lora_ice40_dongle
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND 3V3 +5V 1V8 1V2 1V2FPGA 3V3FPGA /VBUS --power-nets-widths 0.3 0.3 0.4 0.3 0.3 0.3 0.3 0.4 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/lora_ice40_dongle
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --power-nets GND 3V3 +5V 1V8 1V2 1V2FPGA 3V3FPGA /VBUS --power-nets-widths 0.3 0.3 0.4 0.3 0.3 0.3 0.3 0.4 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/lora_ice40_dongle
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_route.kicad_pcb step5_shield.kicad_pcb --nets 'unconnected-(J1-Pad5)' --clearance-ceiling 0.1 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
