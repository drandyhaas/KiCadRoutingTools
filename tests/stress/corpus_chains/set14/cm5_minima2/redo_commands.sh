#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set14/cm5_minima2.kicad_pcb step1_planes.kicad_pcb --nets GND GND GND GND GND +5V +5V M2_3V3 M2_3V3 +3V3_PI +3V3_PI --plane-layers F.Cu B.Cu In1.Cu In2.Cu In4.Cu In3.Cu B.Cu F.Cu In3.Cu F.Cu B.Cu --clearance-ceiling 0.13 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set14/cm5_minima2.kicad_pcb step1_planes_v2.kicad_pcb --nets GND GND GND GND GND +5V M2_3V3 +3V3_PI +3V3_PI --plane-layers F.Cu B.Cu In1.Cu In2.Cu In4.Cu B.Cu F.Cu F.Cu B.Cu --clearance-ceiling 0.13 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes_v2.kicad_pcb step2a_fanout_m301.kicad_pcb --component Module301 --nets '*' '!GND' '!+5V' '!+3V3_PI' '!M2_3V3' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --clearance 0.09 --track-width 0.1 --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes_v2.kicad_pcb --output step2a_fanout_m301.kicad_pcb --component Module301 --nets '*' '!GND' '!+5V' '!+3V3_PI' '!M2_3V3' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --clearance 0.09 --width 0.1 --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes_v2.kicad_pcb --output step2a_fanout_m301b.kicad_pcb --component Module301 --nets '*' '!GND' '!+5V' '!+3V3_PI' '!M2_3V3' --escape-method underpad --allow-via-in-pad --via-size 0.2 --via-drill 0.12 --clearance 0.09 --width 0.0889 --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_m301b.kicad_pcb --output step2b_fanout_m302.kicad_pcb --component Module302 --nets '*' '!GND' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --clearance 0.09 --width 0.0889 --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2b_fanout_m302.kicad_pcb step2c_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_capclear.kicad_pcb step3a_diff100.kicad_pcb '/CM5/DPHY0_P.*' '/CM5/DPHY1_P.*' '/CM5/ETH_PI.TRD*' '/CM5/HDMI1_CK_*' '/CM5/HDMI1_D*' '/CM5/HDMI_PI.CK_*' '/CM5/HDMI_PI.D*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --impedance 100 --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-gnd-vias --no-bga-zones
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2c_capclear.kicad_pcb step3a_diff100.kicad_pcb '/CM5/DPHY0_P.*' '/CM5/DPHY1_P.*' '/CM5/ETH_PI.TRD*' '/CM5/HDMI1_CK_*' '/CM5/HDMI1_D*' '/CM5/HDMI_PI.CK_*' '/CM5/HDMI_PI.D*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --impedance 100 --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-gnd-vias --no-bga-zones --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3a_diff100.kicad_pcb step3b_diff90.kicad_pcb '/CM5/PCIE_PI.CLK_*' '/CM5/PCIE_PI.RX_*' '/CM5/PCIE_PI.TX_*' '/CM5/USB2P.D_*' '/CM5/USB3-0-RX_*' '/CM5/USB3-0-TX_*' '/CM5/USB3-1-D_*' '/CM5/USB3-1-RX_*' '/CM5/USB3-1-TX_*' '/USB_C.D_*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --impedance 90 --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-gnd-vias --no-bga-zones --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3b_diff90.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.13 --track-width 0.1 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +5V M2_3V3 +3V3_PI VBUS_USB2.0 HDMI_+5V +1V8 /CM5/VBUS_EN 'Net-(T401-VCC)' /CM5/VBAT --power-nets-widths 0.3 0.4 0.3 0.3 0.3 0.3 0.25 0.25 0.25 0.25 --no-bga-zones --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set14/cm5_minima2
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_route.kicad_pcb step5_retry.kicad_pcb --nets GND /CM5/ETH_PI.TRD1_N --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.13 --track-width 0.1 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND --power-nets-widths 0.3 --no-bga-zones --max-ripup 5
