#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set14/xerxes_cm4_blade.kicad_pcb step1_planes.kicad_pcb --nets GND GND /CM5_3V3 --plane-layers In1.Cu In4.Cu In2.Cu --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.127 --via-size 0.45 --via-drill 0.2 --track-width 0.127 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_conn1.kicad_pcb --component Conn1 --nets '*' '!GND' '!/CM5_3V3' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.1 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_conn1.kicad_pcb --output step2b_fanout_conn2.kicad_pcb --component Conn2 --nets '*' '!GND' '!/CM5_3V3' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.1 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_conn2.kicad_pcb --output step2c_fanout_conn2_retry.kicad_pcb --component Conn2 --nets /HDMI0_CEC /HDMI0_HOTPLUG --escape-method underpad --allow-via-in-pad --via-size 0.2 --via-drill 0.1 --width 0.08 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_conn2.kicad_pcb --output step2c_fanout_conn2_retry.kicad_pcb --component Conn2 --nets /HDMI0_CEC /HDMI0_HOTPLUG --escape-method stub --width 0.1 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_fanout_conn2_retry.kicad_pcb --output step2d_fanout_j10.kicad_pcb --component J10 --nets '*' '!GND' '!/CM5_3V3' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.1 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2d_fanout_j10.kicad_pcb step2e_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2e_capclear.kicad_pcb step3_diffpairs.kicad_pcb '/DPHY1_*' '/HDMI0_*' '/MIPI0_*' '/PCIe_*' '/TRD*' '/USB2_*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --no-gnd-vias --polarity-swap-nets '/PCIe_RX_*' '/PCIe_TX_*'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2e_capclear.kicad_pcb step3_diffpairs.kicad_pcb '/DPHY1_*' '/HDMI0_*' '/MIPI0_*' '/PCIe_*' '/TRD*' '/USB2_*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --no-gnd-vias --polarity-swap-nets '/PCIe_RX_*' '/PCIe_TX_*'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2e_capclear.kicad_pcb step3_diffpairs.kicad_pcb '/DPHY1_*' '/HDMI0_*' '/MIPI0_*' '/PCIe_*' '/TRD*' '/USB2_*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --no-gnd-vias --polarity-swap-nets '/PCIe_RX_*' '/PCIe_TX_*'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/xerxes_cm4_blade
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_signal.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --power-nets GND /CM5_3V3 /CM5_5V /M2_3v3 --power-nets-widths 0.3 0.3 0.4 0.3 --max-ripup 5
