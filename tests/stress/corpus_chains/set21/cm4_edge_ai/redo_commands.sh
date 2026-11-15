#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set21/cm4_edge_ai.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In4.Cu --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_module1.kicad_pcb --component Module1 --nets '*' '!GND' '!+3V3' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.15 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout_module1.kicad_pcb step2b_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_clearance.kicad_pcb --output step3_fanout_u2.kicad_pcb --component U2 --nets '*' '!GND' --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_clearance.kicad_pcb --output step3_fanout_u2.kicad_pcb --component U2 --nets '*' '!GND' --clearance 0.1 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step3_fanout_u2.kicad_pcb step3b_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3b_clearance.kicad_pcb --output step4a_diff_cam.kicad_pcb --nets '*CAM0*' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --no-gnd-vias --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step4a_diff_cam.kicad_pcb --output step4b_diff_eth.kicad_pcb --nets '*Ethernet_Pair*' --track-width 0.16 --diff-pair-gap 0.15 --clearance-ceiling 0.15 --via-size 0.45 --via-drill 0.2 --no-gnd-vias --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step4b_diff_eth.kicad_pcb --output step4c_diff_pcie.kicad_pcb --nets '*PCIe_CLK*' '*PCIe_RX*' '*PCIe_TX*' '*PCIe_M2_TX*' --track-width 0.21 --diff-pair-gap 0.15 --clearance-ceiling 0.15 --via-size 0.45 --via-drill 0.2 --no-gnd-vias --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step4c_diff_pcie.kicad_pcb --output step4d_diff_usb.kicad_pcb --nets '*USB_D*' --track-width 0.2 --diff-pair-gap 0.15 --clearance-ceiling 0.15 --via-size 0.45 --via-drill 0.2 --no-gnd-vias --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/cm4_edge_ai
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4d_diff_usb.kicad_pcb step5_signal.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5V --power-nets-widths 0.5 0.5 0.5 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --grid-step 0.05 --max-ripup 5
