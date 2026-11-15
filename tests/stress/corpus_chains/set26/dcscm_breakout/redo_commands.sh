#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/dcscm_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/dcscm_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/dcscm_breakout.kicad_pcb step1_planes.kicad_pcb --nets GND GND +3V3 --plane-layers In1.Cu In4.Cu In3.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.1 --track-width 0.125
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/dcscm_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/dcscm_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u3.kicad_pcb --component U3 --width 0.15 --clearance 0.1 --extension 0.3 --nets '*' '!GND' '!+3V3' --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/dcscm_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout_u3.kicad_pcb step3_fanout_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/dcscm_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/dcscm_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3_fanout_clearance.kicad_pcb step4a_diff_85ohm.kicad_pcb '*DC-SCM_CLK_100M_0.CLK*' '*DC-SCM_CLK_100M_1.CLK*' '*DC-SCM_PCIE0_SGMII.RX*' '*DC-SCM_PCIE0_SGMII.TX*' '*DC-SCM_PCIE1.RX0*' '*DC-SCM_PCIE1.TX0*' '*DC-SCM_PCIE_USB_SCMROOT.RX0*' '*DC-SCM_PCIE_USB_SCMROOT.TX0*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.203 --diff-pair-gap 0.189 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/dcscm_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step4a_diff_85ohm.kicad_pcb step4b_diff_90ohm.kicad_pcb '*DC_SCM_USB1.D*' '*DC_SCM_USB2.D*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.182 --diff-pair-gap 0.196 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/dcscm_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step4b_diff_90ohm.kicad_pcb step4c_diff_100ohm.kicad_pcb '*DC_SCM_LTPI.RXCK*' '*DC_SCM_LTPI.RXD*' '*DC_SCM_LTPI.TXCK*' '*DC_SCM_LTPI.TXD*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.155 --diff-pair-gap 0.205 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/dcscm_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step4c_diff_100ohm.kicad_pcb step4d_diff_default.kicad_pcb '*FTDI_D*' '*FTDI_USB_D*' '*PCIE/VSP*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/dcscm_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/dcscm_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4d_diff_default.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.125 --via-size 0.45 --via-drill 0.2 --power-nets GND +3V3 +12V --power-nets-widths 0.4 0.4 0.4 --max-ripup 5 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/dcscm_breakout
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4d_diff_default.kicad_pcb step5b_route_thin.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.0889 --via-size 0.45 --via-drill 0.2 --power-nets GND +3V3 +12V --power-nets-widths 0.4 0.4 0.4 --max-ripup 5 --hole-to-hole-clearance 0.2
