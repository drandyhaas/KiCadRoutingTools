#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set13/sband_transceiver.kicad_pcb step1_planes.kicad_pcb --nets GND +3.3VA --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.150114 --hole-to-hole-clearance 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!+3.3VA' --clearance 0.150114 --output step2a_fanout_U1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_U1.kicad_pcb --component U2 --nets '*' '!GND' '!+3.3VA' --clearance 0.150114 --output step2b_fanout_U2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_U2.kicad_pcb --component U4 --nets '*' '!GND' '!+3.3VA' --clearance 0.150114 --output step2c_fanout_U4.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2c_fanout_U4.kicad_pcb --component U7 --nets '*' '!GND' '!+3.3VA' --clearance 0.150114 --output step2d_fanout_U7.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2d_fanout_U7.kicad_pcb --component U8 --nets '*' '!GND' '!+3.3VA' --clearance 0.150114 --output step2e_fanout_U8.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2e_fanout_U8.kicad_pcb --component U12 --nets '*' '!GND' '!+3.3VA' --clearance 0.150114 --output step2f_fanout_U12.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2f_fanout_U12.kicad_pcb --component U15 --nets '*' '!GND' '!+3.3VA' --clearance 0.150114 --output step2g_fanout_U15.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2g_fanout_U15.kicad_pcb --component U20 --nets '*' '!GND' '!+3.3VA' --clearance 0.150114 --output step2h_fanout_U20.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2h_fanout_U20.kicad_pcb step2i_capclear.kicad_pcb --clearance 0.150114
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2i_capclear.kicad_pcb step3_diffpairs.kicad_pcb --nets LO+ LO- 'Net-(U1-RF_OUT_A+)' 'Net-(U1-RF_OUT_A-)' 'Net-(U1-RF_OUT_B+)' 'Net-(U1-RF_OUT_B-)' 'Net-(U12-RFIN+)' 'Net-(U12-RFIN-)' 'Net-(U15-RFIP)' 'Net-(U15-RFIN)' 'Net-(U20-RFIN+)' 'Net-(U20-RFIN-)' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.10 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3.3VA +5V +3V3 -5V VBUS 'Net-(U10-Vdd)' 'Net-(U16-Vdd)' --power-nets-widths 0.5 0.4 0.4 0.4 0.3 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.10 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3.3VA +5V +3V3 -5V VBUS 'Net-(U10-Vdd)' 'Net-(U16-Vdd)' --power-nets-widths 0.5 0.4 0.4 0.4 0.3 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.10 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3.3VA +5V +3V3 -5V VBUS 'Net-(U10-Vdd)' 'Net-(U16-Vdd)' --power-nets-widths 0.5 0.4 0.4 0.4 0.3 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.10 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3.3VA +5V +3V3 -5V VBUS 'Net-(U10-Vdd)' 'Net-(U16-Vdd)' --power-nets-widths 0.5 0.4 0.4 0.4 0.3 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.10 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3.3VA +5V +3V3 -5V VBUS 'Net-(U10-Vdd)' 'Net-(U16-Vdd)' --power-nets-widths 0.5 0.4 0.4 0.4 0.3 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.10 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3.3VA +5V +3V3 -5V VBUS 'Net-(U10-Vdd)' 'Net-(U16-Vdd)' --power-nets-widths 0.5 0.4 0.4 0.4 0.3 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.10 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3.3VA +5V +3V3 -5V VBUS 'Net-(U10-Vdd)' 'Net-(U16-Vdd)' --power-nets-widths 0.5 0.4 0.4 0.4 0.3 0.4 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/sband_transceiver
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.10 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.25 --power-nets GND +3.3VA +5V +3V3 -5V VBUS 'Net-(U10-Vdd)' 'Net-(U16-Vdd)' --power-nets-widths 0.5 0.4 0.4 0.4 0.3 0.4 0.3 0.3
