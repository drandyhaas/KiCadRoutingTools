#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/core1106_cam
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/core1106_cam
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/core1106_cam
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/core1106_cam.kicad_pcb step1_planes.kicad_pcb --nets GND DOVDD_1V8 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2 --zone-clearance 0.2 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets 5V_IN DVDD_1V5 AVDD_REG --power-nets-widths 0.5 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/core1106_cam
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/core1106_cam.kicad_pcb step1_planes.kicad_pcb --nets /GND /DOVDD_1V8 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2 --zone-clearance 0.2 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets /5V_IN /DVDD_1V5 /AVDD_REG --power-nets-widths 0.5 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/core1106_cam
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diff_mipi.kicad_pcb --pairs /MIPI_CSI_RX_CLK0P /MIPI_CSI_RX_CLK0N /MIPI_CSI_RX_CLK1P /MIPI_CSI_RX_CLK1N /MIPI_CSI_RX_D0P /MIPI_CSI_RX_D0N /MIPI_CSI_RX_D1P /MIPI_CSI_RX_D1N /MIPI_CSI_RX_D2P /MIPI_CSI_RX_D2N /MIPI_CSI_RX_D3P /MIPI_CSI_RX_D3N --track-width 0.13 --diff-pair-gap 0.1 --clearance-ceiling 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/core1106_cam
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/core1106_cam
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1_planes.kicad_pcb step2_diff_mipi.kicad_pcb --nets '/MIPI_CSI_RX_*' --track-width 0.13 --diff-pair-gap 0.1 --clearance-ceiling 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/core1106_cam
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2_diff_mipi.kicad_pcb step3_diff_usb.kicad_pcb --nets /USB_DP /USB_DM --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.25 --layers F.Cu In1.Cu In2.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/core1106_cam
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff_usb.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets /GND /DOVDD_1V8 /5V_IN /DVDD_1V5 /AVDD_REG --power-nets-widths 0.3 0.3 0.5 0.3 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/core1106_cam
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_route.kicad_pcb step5_retry.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.6 --via-drill 0.3 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5 --power-nets /GND /DOVDD_1V8 /5V_IN /DVDD_1V5 /AVDD_REG --power-nets-widths 0.3 0.3 0.5 0.3 0.3 --max-ripup 5
