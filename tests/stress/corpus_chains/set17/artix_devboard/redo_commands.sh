#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set17/artix_devboard.kicad_pcb step1_planes.kicad_pcb --nets GND +3.3V --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --component U301 --nets '*' '!GND' '!+3.3V' --diff-pairs 'PCIe_RX_*' 'PCIe_TX_*' 'PCIe_CLOCK_*' 'HDMI_RX_*_TMDS_*' 'HDMI_TX_*_TMDS_*' --diff-pair-gap 0.11 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance 0.1 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --output step2_u301_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_u301_fanout.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_capclear.kicad_pcb --component U102 --nets '*' '!GND' '!+3.3V' --clearance 0.1 --width 0.15 --output step3a_u102_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3a_u102_fanout.kicad_pcb --component U201 --nets '*' '!GND' '!+3.3V' --clearance 0.1 --width 0.15 --output step3b_u201_fanout.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3a_u102_fanout.kicad_pcb --component U201 --nets '*' '!GND' '!+3.3V' --clearance 0.09 --width 0.1 --grid-step 0.05 --output step3b_u201_fanout_retry.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3a_u102_fanout.kicad_pcb --component U201 --nets '*' '!GND' '!+3.3V' --clearance 0.1 --width 0.15 --escape-method underpad --via-size 0.45 --via-drill 0.25 --allow-via-in-pad --output step3b_u201_fanout_underpad.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3b_u201_fanout_underpad.kicad_pcb step4a_diff_pcie.kicad_pcb --nets 'PCIe_RX_*' 'PCIe_TX_*' 'PCIe_CLOCK_*' '/PCIe/PCIe_*' --track-width 0.3 --diff-pair-gap 0.11 --clearance-ceiling 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias --polarity-swap-nets 'PCIe_*' '/PCIe/*'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4a_diff_pcie.kicad_pcb step4b_diff_hdmi.kicad_pcb --nets 'HDMI_RX_*_TMDS_*' 'HDMI_TX_*_TMDS_*' --track-width 0.18 --diff-pair-gap 0.11 --clearance-ceiling 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4a_diff_pcie.kicad_pcb step4b_diff_hdmi.kicad_pcb --nets 'HDMI_RX_*_TMDS_*' 'HDMI_TX_*_TMDS_*' --track-width 0.18 --diff-pair-gap 0.11 --clearance-ceiling 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4a_diff_pcie.kicad_pcb step4b_diff_hdmi.kicad_pcb --nets 'HDMI_RX_*_TMDS_*' 'HDMI_TX_*_TMDS_*' --track-width 0.18 --diff-pair-gap 0.11 --clearance-ceiling 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4a_diff_pcie.kicad_pcb step4b_diff_hdmi.kicad_pcb --nets 'HDMI_RX_*_TMDS_*' 'HDMI_TX_*_TMDS_*' --track-width 0.18 --diff-pair-gap 0.11 --clearance-ceiling 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4a_diff_pcie.kicad_pcb step4b_diff_hdmi.kicad_pcb --nets 'HDMI_RX_*_TMDS_*' 'HDMI_TX_*_TMDS_*' --track-width 0.18 --diff-pair-gap 0.11 --clearance-ceiling 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4a_diff_pcie.kicad_pcb step4b_diff_hdmi.kicad_pcb --nets 'HDMI_RX_*_TMDS_*' 'HDMI_TX_*_TMDS_*' --track-width 0.18 --diff-pair-gap 0.11 --clearance-ceiling 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4a_diff_pcie.kicad_pcb step4b_diff_hdmi.kicad_pcb --nets 'HDMI_RX_*_TMDS_*' 'HDMI_TX_*_TMDS_*' --track-width 0.18 --diff-pair-gap 0.11 --clearance-ceiling 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4a_diff_pcie.kicad_pcb step4b_diff_hdmi.kicad_pcb --nets 'HDMI_RX_*_TMDS_*' 'HDMI_TX_*_TMDS_*' --track-width 0.18 --diff-pair-gap 0.11 --clearance-ceiling 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4b_diff_hdmi.kicad_pcb step4c_diff_usb.kicad_pcb --nets '/USB-C_D*' 'Net-(U102-D*)' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4c_diff_usb.kicad_pcb step5_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +12V +1V0 +1V8 +1V2 +5V +5V_USB 'Net-(J101-+5V)' 'Net-(J103-+5V)' 'Net-(J104-+5V)' 'Net-(J105-+5V)' 'Net-(U201-VDD)' 'Net-(U301G-VCCBATT_0)' --power-nets-widths 0.3 0.4 0.5 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4c_diff_usb.kicad_pcb step5_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +12V +1V0 +1V8 +1V2 +5V +5V_USB 'Net-(J101-+5V)' 'Net-(J103-+5V)' 'Net-(J104-+5V)' 'Net-(J105-+5V)' 'Net-(U201-VDD)' 'Net-(U301G-VCCBATT_0)' --power-nets-widths 0.3 0.4 0.5 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route.kicad_pcb step5b_retry.kicad_pcb --nets +12V +3.3V +1V0 --no-bga-zone --max-ripup 5 --grid-step 0.05 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +12V +1V0 +1V8 +1V2 +5V +5V_USB 'Net-(J101-+5V)' 'Net-(J103-+5V)' 'Net-(J104-+5V)' 'Net-(J105-+5V)' 'Net-(U201-VDD)' 'Net-(U301G-VCCBATT_0)' --power-nets-widths 0.3 0.4 0.5 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set17/artix_devboard
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5b_retry.kicad_pcb step5c_1v8fix.kicad_pcb --nets +1V8 --force-reroute --no-bga-zone --max-ripup 5 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +12V +1V0 +1V8 +1V2 +5V +5V_USB 'Net-(J101-+5V)' 'Net-(J103-+5V)' 'Net-(J104-+5V)' 'Net-(J105-+5V)' 'Net-(U201-VDD)' 'Net-(U301G-VCCBATT_0)' --power-nets-widths 0.3 0.4 0.5 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.25 0.25
