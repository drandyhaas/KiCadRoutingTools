#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/private/tmp/wt_baseline_0809/py_router
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/lwdo_sdr
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/lwdo_sdr
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set25/lwdo_sdr.kicad_pcb step1_planes.kicad_pcb --nets AGND --plane-layers B.Cu --clearance-ceiling 0.175 --via-size 0.65 --via-drill 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/lwdo_sdr
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb -c U201 -o step2a_fanout_u201.kicad_pcb --clearance 0.1 --grid-step 0.05 --nets '!AGND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/lwdo_sdr
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2a_fanout_u201.kicad_pcb -c U1302 -o step2b_fanout_u1302.kicad_pcb --clearance 0.1 --grid-step 0.05 --nets '!AGND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/lwdo_sdr
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/lwdo_sdr
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2b_fanout_u1302.kicad_pcb step3_diffpairs.kicad_pcb /ADC1/INA+ /ADC1/INB+ /ADC2/INA+ /ADC2/INB+ /FTDI/USB_D+ --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/lwdo_sdr
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/lwdo_sdr
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.175 --track-width 0.2 --via-size 0.65 --via-drill 0.25 --power-nets AGND +3.3VD +3.3VA +5VMAIN +1.2VD +2.5VA '/Power Supply/VBUS_CLEAN' '/Power Supply/+5VFTDI' '/Clock Output/VDD_3V3' '/Extension Connector/VDDIO' '/Extension Connector/EXT_VDDIO' /VCXO/VCXO_VCC '/Extension Connector/EXT_VDD3V3' '/Power Supply/VBUS' --power-nets-widths 0.4 0.3 0.3 0.3 0.3 0.25 0.3 0.25 0.25 0.25 0.25 0.25 0.25 0.3 --max-ripup 5
