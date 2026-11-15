#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set3/orangecrab.kicad_pcb step1_planes.kicad_pcb --nets GND VSYS GND VTT GND P3.3V P1.1V P1.35V ECP5_VREF --plane-layers F.Cu F.Cu B.Cu B.Cu In1.Cu In3.Cu In3.Cu In4.Cu In4.Cu --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.0889 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --component U3 --nets '*' '!GND' '!VSYS' '!VTT' '!P3.3V' '!P1.1V' '!P1.35V' '!ECP5_VREF' --diff-pairs 'EXT_PLL*' --diff-pair-gap 0.1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --escape-method dogbone --clearance 0.09 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --output step1b_U3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1b_U3.kicad_pcb --component U4 --nets '*' '!GND' '!ECP5_VREF' --diff-pairs 'RAM_CK*' 'RAM_LDQS*' 'RAM_UDQS*' --diff-pair-gap 0.1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance 0.09 --track-width 0.13 --via-size 0.4 --via-drill 0.2 --output step1b_U4.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1b_U4.kicad_pcb --component U5 --nets '*' '!GND' '!VSYS' '!VTT' '!P3.3V' '!P1.1V' '!P1.35V' '!ECP5_VREF' --clearance 0.09 --width 0.1 --output step1b_U5.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1b_U5.kicad_pcb --component U8 --nets '*' '!GND' '!VSYS' '!VTT' '!P3.3V' '!P1.1V' '!P1.35V' '!ECP5_VREF' --clearance 0.09 --width 0.1 --output step1b_U8.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1b_U8.kicad_pcb --component U9 --nets '*' '!GND' '!VSYS' '!VTT' '!P3.3V' '!P1.1V' '!P1.35V' '!ECP5_VREF' --clearance 0.09 --width 0.1 --output step1b_U9.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1b_U9.kicad_pcb --component U1 --nets '*' '!GND' '!VSYS' '!VTT' '!P3.3V' '!P1.1V' '!P1.35V' '!ECP5_VREF' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --escape-method underpad --allow-via-in-pad --clearance 0.09 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --output step1b_U1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1b_U9.kicad_pcb --component U1 --nets '*' '!GND' '!VSYS' '!VTT' '!P3.3V' '!P1.1V' '!P1.35V' '!ECP5_VREF' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --escape-method underpad --same-net-pad-clearance -1 --clearance 0.09 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --output step1b_U1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1b_U1.kicad_pcb --component U2 --nets '*' '!GND' '!VSYS' '!VTT' '!P3.3V' '!P1.1V' '!P1.35V' '!ECP5_VREF' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --escape-method underpad --same-net-pad-clearance -1 --clearance 0.09 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --output step1b_U2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1b_U2.kicad_pcb --component U7 --nets '*' '!GND' '!VSYS' '!VTT' '!P3.3V' '!P1.1V' '!P1.35V' '!ECP5_VREF' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --escape-method underpad --same-net-pad-clearance -1 --clearance 0.09 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --output step1b_U7.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_U7.kicad_pcb step1c_clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1c_clearance.kicad_pcb step2a_diffpairs.kicad_pcb --nets 'EXT_PLL*' 'RAM_CK*' 'RAM_LDQS*' 'RAM_UDQS*' 'USB_D*' '/sheetIO/_USB_D_*' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.25 --via-size 0.25 --via-drill 0.15 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --no-gnd-vias --diff-pair-intra-match
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --track-width 0.0762 --clearance-ceiling 0.0889 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.25 --direction-preference-cost 5 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 6.0 1.0 2.5 2.5 1.0 --power-nets GND VSYS VTT P3.3V P1.1V P1.35V ECP5_VREF /sheetHyperRAM/RAM_VDDQ /sheetHyperRAM/RAM_VDD P2.5V VBATT PUSB_VBUS --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 0.2 0.25 0.25 0.2 0.2 0.2 --length-match-group auto --length-match-tolerance 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step_iter1.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --track-width 0.0762 --clearance-ceiling 0.0889 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.25 --direction-preference-cost 5 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 6.0 1.0 2.5 2.5 1.0 --power-nets GND VSYS VTT P3.3V P1.1V P1.35V ECP5_VREF /sheetHyperRAM/RAM_VDDQ /sheetHyperRAM/RAM_VDD P2.5V VBATT PUSB_VBUS --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 0.2 0.25 0.25 0.2 0.2 0.2 --length-match-group auto --length-match-tolerance 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step_iter1.kicad_pcb step_iter2.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --track-width 0.0762 --clearance-ceiling 0.0889 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.25 --direction-preference-cost 5 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 6.0 1.0 2.5 2.5 1.0 --power-nets GND VSYS VTT P3.3V P1.1V P1.35V ECP5_VREF /sheetHyperRAM/RAM_VDDQ /sheetHyperRAM/RAM_VDD P2.5V VBATT PUSB_VBUS --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 0.2 0.25 0.25 0.2 0.2 0.2 --length-match-group auto --length-match-tolerance 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step_iter1.kicad_pcb step3_planes_final.kicad_pcb --nets GND VSYS GND VTT GND P3.3V P1.1V P1.35V ECP5_VREF --plane-layers F.Cu F.Cu B.Cu B.Cu In1.Cu In3.Cu In3.Cu In4.Cu In4.Cu --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.09 --hole-to-hole-clearance 0.25 --via-size 0.45 --via-drill 0.2 --add-gnd-vias --gnd-via-net GND --gnd-via-distance 2.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set3/orangecrab
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_planes_final.kicad_pcb step4_reconnect.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --track-width 0.0762 --clearance-ceiling 0.0889 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.25 --direction-preference-cost 5 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 6.0 1.0 2.5 2.5 1.0 --power-nets GND VSYS VTT P3.3V P1.1V P1.35V ECP5_VREF /sheetHyperRAM/RAM_VDDQ /sheetHyperRAM/RAM_VDD P2.5V VBATT PUSB_VBUS --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 0.2 0.25 0.25 0.2 0.2 0.2
