#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set28/mez_rx.kicad_pcb step1_planes.kicad_pcb --nets GND GND V1P2 --plane-layers In1.Cu In6.Cu In3.Cu --via-size 0.45 --via-drill 0.2 --track-width 0.5 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1.kicad_pcb --component U1 --layers F.Cu In2.Cu In4.Cu In5.Cu B.Cu --track-width 0.127 --clearance 0.1 --via-size 0.4 --via-drill 0.2 --nets '*' '!GND' '!V1P2' --plane-net-layers GND:In1.Cu,In6.Cu V1P2:In3.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1.kicad_pcb --component U1 --layers F.Cu In2.Cu In4.Cu In5.Cu B.Cu --track-width 0.127 --clearance 0.09 --via-size 0.35 --via-drill 0.15 --nets '*' '!GND' '!V1P2' --plane-net-layers GND:In1.Cu,In6.Cu V1P2:In3.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2a_fanout_u1.kicad_pcb step2b_clr_u1.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_clr_u1.kicad_pcb --output step2c_fanout_u3.kicad_pcb --component U3 --width 0.1 --clearance 0.09 --via-size 0.25 --via-drill 0.15 --extension 0.3 --nets '*' '!GND' '!V1P2'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_clr_u1.kicad_pcb --output step2c_fanout_u3.kicad_pcb --component U3 --width 0.1 --clearance 0.09 --via-size 0.25 --via-drill 0.15 --extension 0.55 --nets '*' '!GND' '!V1P2'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_clr_u1.kicad_pcb --output step2c_fanout_u3.kicad_pcb --component U3 --escape-method underpad --allow-via-in-pad --width 0.1 --clearance 0.09 --via-size 0.25 --via-drill 0.15 --extension 0.3 --nets '*' '!GND' '!V1P2'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step2c_fanout_u3.kicad_pcb --output step2d_fanout_u4.kicad_pcb --component U4 --layers F.Cu In2.Cu In4.Cu In5.Cu B.Cu --track-width 0.127 --clearance 0.09 --via-size 0.35 --via-drill 0.15 --nets '*' '!GND' '!V1P2' --plane-net-layers GND:In1.Cu,In6.Cu V1P2:In3.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2d_fanout_u4.kicad_pcb --output step2e_fanout_jh1.kicad_pcb --component JH1 --escape-method underpad --allow-via-in-pad --width 0.127 --clearance 0.09 --via-size 0.35 --via-drill 0.15 --nets '*' '!GND' '!V1P2'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2e_fanout_jh1.kicad_pcb --output step2f_fanout_jh2.kicad_pcb --component JH2 --escape-method underpad --allow-via-in-pad --width 0.127 --clearance 0.09 --via-size 0.35 --via-drill 0.15 --nets '*' '!GND' '!V1P2'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2f_fanout_jh2.kicad_pcb step2g_clr_all.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2g_clr_all.kicad_pcb step3_diff.kicad_pcb 'PORT1_T*' 'PORT2_T*' --layers F.Cu In2.Cu In4.Cu In5.Cu B.Cu --track-width 0.127 --diff-pair-gap 0.15 --impedance 100 --clearance-ceiling 0.127 --hole-to-hole-clearance 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In2.Cu In4.Cu In5.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.1 --grid-step 0.05 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND V1P2 V1P8 V2P5 V3P3 V2P5_VPP D5V_IN KSZ_AVDDL_1V2 KSZ_AVDDH_2V5 V2P5_PLL_NW V2P5_PLL_NE V2P5_PLL_SW ETH_SHIELD VDDI5_UNUSED --power-nets-widths 0.5 0.5 0.4 0.4 0.4 0.3 0.5 0.25 0.25 0.2 0.2 0.2 0.3 0.2 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In2.Cu In4.Cu In5.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.1 --grid-step 0.05 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND V1P2 V1P8 V2P5 V3P3 V2P5_VPP D5V_IN KSZ_AVDDL_1V2 KSZ_AVDDH_2V5 V2P5_PLL_NW V2P5_PLL_NE V2P5_PLL_SW ETH_SHIELD VDDI5_UNUSED --power-nets-widths 0.5 0.5 0.4 0.4 0.4 0.3 0.5 0.25 0.25 0.2 0.2 0.2 0.3 0.2 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In2.Cu In4.Cu In5.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.1 --grid-step 0.05 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND V1P2 V1P8 V2P5 V3P3 V2P5_VPP D5V_IN KSZ_AVDDL_1V2 KSZ_AVDDH_2V5 V2P5_PLL_NW V2P5_PLL_NE V2P5_PLL_SW ETH_SHIELD VDDI5_UNUSED --power-nets-widths 0.5 0.5 0.4 0.4 0.4 0.3 0.5 0.25 0.25 0.2 0.2 0.2 0.3 0.2 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In2.Cu In4.Cu In5.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.1 --grid-step 0.05 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND V1P2 V1P8 V2P5 V3P3 V2P5_VPP D5V_IN KSZ_AVDDL_1V2 KSZ_AVDDH_2V5 V2P5_PLL_NW V2P5_PLL_NE V2P5_PLL_SW ETH_SHIELD VDDI5_UNUSED --power-nets-widths 0.5 0.5 0.4 0.4 0.4 0.3 0.5 0.25 0.25 0.2 0.2 0.2 0.3 0.2 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In2.Cu In4.Cu In5.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.1 --grid-step 0.05 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND V1P2 V1P8 V2P5 V3P3 V2P5_VPP D5V_IN KSZ_AVDDL_1V2 KSZ_AVDDH_2V5 V2P5_PLL_NW V2P5_PLL_NE V2P5_PLL_SW ETH_SHIELD VDDI5_UNUSED --power-nets-widths 0.5 0.5 0.4 0.4 0.4 0.3 0.5 0.25 0.25 0.2 0.2 0.2 0.3 0.2 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In2.Cu In4.Cu In5.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.1 --grid-step 0.05 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND V1P2 V1P8 V2P5 V3P3 V2P5_VPP D5V_IN KSZ_AVDDL_1V2 KSZ_AVDDH_2V5 V2P5_PLL_NW V2P5_PLL_NE V2P5_PLL_SW ETH_SHIELD VDDI5_UNUSED --power-nets-widths 0.5 0.5 0.4 0.4 0.4 0.3 0.5 0.25 0.25 0.2 0.2 0.2 0.3 0.2 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set28/mez_rx
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In2.Cu In4.Cu In5.Cu B.Cu --clearance-ceiling 0.1 --track-width 0.1 --grid-step 0.05 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --power-nets GND V1P2 V1P8 V2P5 V3P3 V2P5_VPP D5V_IN KSZ_AVDDL_1V2 KSZ_AVDDH_2V5 V2P5_PLL_NW V2P5_PLL_NE V2P5_PLL_SW ETH_SHIELD VDDI5_UNUSED --power-nets-widths 0.5 0.5 0.4 0.4 0.4 0.3 0.5 0.25 0.25 0.2 0.2 0.2 0.3 0.2 --max-ripup 5 --no-bga-zone
