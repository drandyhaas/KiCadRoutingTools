#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/daisho.kicad_pcb --component U1 --output /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step1_fanout_U1.kicad_pcb --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --nets '*' '!GND' '!V1P8' --diff-pairs '*fpga_ddr2/CLK_P*' '*fpga_ddr2/CLK_N*' --diff-pair-gap 0.25 --clearance 0.127 --track-width 0.1651 --via-size 0.45 --via-drill 0.2032
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/daisho.kicad_pcb --component U1 --output /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step1_fanout_U1.kicad_pcb --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --nets '*' '!GND' '!V1P8' --diff-pairs '*fpga_ddr2/CLK_P*' '*fpga_ddr2/CLK_N*' --diff-pair-gap 0.25 --escape-method channel --clearance 0.127 --track-width 0.1651 --via-size 0.45 --via-drill 0.2032
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/daisho.kicad_pcb --component U1 --output /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step1b_fanout_U1_floor.kicad_pcb --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --nets '*' '!GND' '!V1P8' --diff-pairs '*fpga_ddr2/CLK_P*' '*fpga_ddr2/CLK_N*' --diff-pair-gap 0.25 --escape-method channel --clearance 0.09 --track-width 0.1651 --via-size 0.45 --via-drill 0.2032
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step1b_fanout_U1_floor.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step2_capopt_U1.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step2_capopt_U1.kicad_pcb --component U2 --output /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step3_fanout_U2.kicad_pcb --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --nets '*' '!GND' '!V1P8' --diff-pairs '*usb0_interfaces/DP' '*usb0_interfaces/DM' --diff-pair-gap 0.25 --escape-method channel --clearance 0.09 --track-width 0.1651 --via-size 0.45 --via-drill 0.2032
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step2_capopt_U1.kicad_pcb --component U2 --output /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step3b_fanout_U2_fine.kicad_pcb --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --nets '*' '!GND' '!V1P8' --diff-pairs '*usb0_interfaces/DP' '*usb0_interfaces/DM' --diff-pair-gap 0.25 --escape-method channel --clearance 0.09 --track-width 0.1651 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step3b_fanout_U2_fine.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step4_capopt_U2.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step4_capopt_U2.kicad_pcb --component U3 --output /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step5_fanout_U3.kicad_pcb --nets '*' '!GND' '!V3P3D' --width 0.1651 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step5_fanout_U3.kicad_pcb --component U4 --output /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step6_fanout_U4.kicad_pcb --nets '*' '!GND' '!V3P3A' --width 0.1651 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step6_fanout_U4.kicad_pcb --output /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step7_route_diff.kicad_pcb --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --nets /clock_generator/FE_CLK_P0 /clock_generator/FE_CLK_N0 /clock_generator/FE_CLK_P1 /clock_generator/FE_CLK_N1 /fpga_ddr2/CLK_P0 /fpga_ddr2/CLK_N0 /fpga_ddr2/CLK_P1 /fpga_ddr2/CLK_N1 /usb0_interfaces/DP /usb0_interfaces/DM --clearance-ceiling 0.127 --track-width 0.2 --diff-pair-gap 0.25 --via-size 0.45 --via-drill 0.2032 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step7_route_diff.kicad_pcb /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step8_planes.kicad_pcb --nets GND GND GND V1P8 --plane-layers In1.Cu In3.Cu In6.Cu In4.Cu --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --via-size 0.45 --via-drill 0.2032 --track-width 0.1651 --clearance-ceiling 0.127 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/daisho
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step8_planes.kicad_pcb --output /Users/andy/Documents/kicad_stress_test/runs_set3/daisho/step9_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --layer-costs 1.0 3.0 1.0 3.0 3.0 1.0 3.0 1.0 --clearance-ceiling 0.127 --track-width 0.1651 --via-size 0.45 --via-drill 0.2032 --hole-to-hole-clearance 0.25 --power-nets GND V1P8 V1P2 V1P1 V3P3D VRAW /front_end/FE_VCCIO_B /front_end/FE_VCCIO_C /front_end/FE_VCCIO_A /fpga_power/VCCD_PLL /fpga_front_end_bank_b/VCCIO_DEFAULT_EN /usb0_interfaces/VBUS --power-nets-widths 0.4 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.25 0.25 0.25 --max-ripup 5 --no-bga-zone
