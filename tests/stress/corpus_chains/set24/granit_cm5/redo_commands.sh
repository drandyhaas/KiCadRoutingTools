#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set24/granit_cm5.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.2 --power-nets GND +3V3 --power-nets-widths 0.2 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_cm1_fanout.kicad_pcb --component CM1 --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.1 --clearance 0.09 --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_cm1_fanout.kicad_pcb --component CM1 --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.1 --clearance 0.09 --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2a_cm1_fanout.kicad_pcb step2b_cm1_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_cm1_clearance.kicad_pcb --output step2c_u3_fanout.kicad_pcb --component U3 --clearance 0.09 --width 0.1 --nets '*' '!GND' '!+3V3'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2c_u3_fanout.kicad_pcb step3_diffpairs.kicad_pcb --nets '/SATA Bridge/PCIE_D_RX_P' '/SATA Bridge/PCIE_D_RX_N' '/SATA Bridge/SATA_A_RX_P' '/SATA Bridge/SATA_A_RX_N' '/SATA Bridge/SATA_A_TX_P' '/SATA Bridge/SATA_A_TX_N' '/SATA Bridge/SATA_D_RX_P' '/SATA Bridge/SATA_D_RX_N' '/SATA Bridge/SATA_D_TX_P' '/SATA Bridge/SATA_D_TX_N' ETH_P0_P ETH_P0_N ETH_P1_P ETH_P1_N ETH_P2_P ETH_P2_N ETH_P3_P ETH_P3_N PCIE_CLK_P PCIE_CLK_N PCIE_RX_P PCIE_RX_N PCIE_TX_P PCIE_TX_N USB_DP USB_DN --layers F.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.45 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5VP +12VP 'Net-(U3-VCC12_1)' 'Net-(U1-VBUS)' '/SATA Bridge/+5V_S' '/SATA Bridge/+12V_S' --power-nets-widths 0.3 1.0 2.5 3.0 1.0 2.5 1.5 1.5 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5VP +12VP 'Net-(U3-VCC12_1)' 'Net-(U1-VBUS)' '/SATA Bridge/+5V_S' '/SATA Bridge/+12V_S' --power-nets-widths 0.3 1.0 2.5 3.0 1.0 2.5 1.5 1.5 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/granit_cm5
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +5VP +12VP 'Net-(U3-VCC12_1)' 'Net-(U1-VBUS)' '/SATA Bridge/+5V_S' '/SATA Bridge/+12V_S' --power-nets-widths 0.3 1.0 2.5 3.0 1.0 2.5 1.5 1.5 --max-ripup 5
