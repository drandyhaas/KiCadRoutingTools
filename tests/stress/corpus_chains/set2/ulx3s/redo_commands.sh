#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/ulx3s.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --escape-method channel --diff-pairs 'SRCLK*' 'FPDI_CLK*' 'FPDI_D0*' 'FPDI_D1*' 'FPDI_D2*' 'USB_FPGA_D*' 'USB_FPGA_PULL_D*' --diff-pair-gap 0.25 --track-width 0.2 --clearance 0.2 --via-size 0.5 --via-drill 0.3 --output step1_U1_diffchannel.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set2/ulx3s.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --escape-method channel --nets SRCLKN SRCLKP FPDI_CLK+ FPDI_CLK- FPDI_D0+ FPDI_D0- FPDI_D1+ FPDI_D1- FPDI_D2+ FPDI_D2- USB_FPGA_D+ USB_FPGA_D- USB_FPGA_PULL_D+ USB_FPGA_PULL_D- --diff-pairs 'SRCLK*' 'FPDI_CLK*' 'FPDI_D0*' 'FPDI_D1*' 'FPDI_D2*' 'USB_FPGA_D*' 'USB_FPGA_PULL_D*' --diff-pair-gap 0.2 --track-width 0.15 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --output step1_U1_diffchannel.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_U1_diffchannel.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --escape-method channel --check-for-previous --nets FPDI_CLK+ FPDI_CLK- USB_FPGA_PULL_D+ USB_FPGA_PULL_D- --diff-pairs 'FPDI_CLK*' 'USB_FPGA_PULL_D*' --diff-pair-gap 0.2 --track-width 0.12 --clearance 0.09 --via-size 0.2 --via-drill 0.12 --output step1b_U1_diffchannel_retry.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1b_U1_diffchannel_retry.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu B.Cu --escape-method underpad --check-for-previous --nets '*' '!GND' '!+1V1' '!+2V5' '!+3V3' '!2V5_3V3' --track-width 0.12 --clearance 0.1 --via-size 0.35 --via-drill 0.2 --output step2_U1_underpad.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_U1_underpad.kicad_pcb step3_placeclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3_placeclear.kicad_pcb --component U8 --width 0.15 --clearance 0.1 --via-size 0.35 --via-drill 0.2 --output step4_U8.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step4_U8.kicad_pcb --component U6 --width 0.15 --clearance 0.1 --via-size 0.35 --via-drill 0.2 --output step5_U6.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step5_U6.kicad_pcb step6_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.5 --via-drill 0.3 --track-width 0.2 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +2V5 +5V --power-nets-widths 0.3 0.3 0.25 0.25 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step6_planes.kicad_pcb step7_diffpairs.kicad_pcb --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_diffpairs.kicad_pcb step8_signal.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.15 --clearance-ceiling 0.15 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +2V5 +5V '*VBAT*' '*RTCVDD*' '*US2VBUS*' --power-nets-widths 0.3 0.3 0.25 0.25 0.3 0.25 0.25 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_diffpairs.kicad_pcb step8_signal.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.15 --clearance-ceiling 0.15 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +2V5 +5V '*VBAT*' '*RTCVDD*' '*US2VBUS*' --power-nets-widths 0.3 0.3 0.25 0.25 0.3 0.25 0.25 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_diffpairs.kicad_pcb step8_signal.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.15 --clearance-ceiling 0.15 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +2V5 +5V '*VBAT*' '*RTCVDD*' '*US2VBUS*' --power-nets-widths 0.3 0.3 0.25 0.25 0.3 0.25 0.25 0.2 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_diffpairs.kicad_pcb step8_signal.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.15 --clearance-ceiling 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +2V5 +5V '*VBAT*' '*RTCVDD*' '*US2VBUS*' --power-nets-widths 0.3 0.3 0.25 0.25 0.3 0.25 0.25 0.2 --max-ripup 3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_diffpairs.kicad_pcb step8_signal.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.15 --clearance-ceiling 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +2V5 +5V '*VBAT*' '*RTCVDD*' '*US2VBUS*' --power-nets-widths 0.3 0.3 0.25 0.25 0.3 0.25 0.25 0.2 --max-ripup 3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set2/ulx3s
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step8_signal.kicad_pcb step8b_retry.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.1 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 +1V1 +2V5 +5V '*VBAT*' '*RTCVDD*' '*US2VBUS*' --power-nets-widths 0.3 0.3 0.25 0.25 0.3 0.25 0.25 0.2 --max-ripup 3
