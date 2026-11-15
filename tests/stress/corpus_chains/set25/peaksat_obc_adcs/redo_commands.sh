#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set25/peaksat_obc_adcs.kicad_pcb step1_planes.kicad_pcb --nets GND GND GND /ADCS_VDD '/OBC module/3V3_PROT' --plane-layers In1.Cu In3.Cu In6.Cu In4.Cu In4.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U16 --nets '*' '!GND' '!/OBC module/3V3_PROT' --clearance 0.09 --grid-step 0.05 --output step1b_fanout_u16.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1b_fanout_u16.kicad_pcb --component U14 --nets '*' '!GND' '!/ADCS_VDD' --clearance 0.09 --grid-step 0.05 --output step1b_fanout_u14.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1b_fanout_u14.kicad_pcb --component IC1 --nets '*' '!GND' --clearance 0.09 --grid-step 0.05 --output step1b_fanout_ic1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step1b_fanout_ic1.kicad_pcb step1c_capopt.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step1c_capopt.kicad_pcb step2a_diffpairs.kicad_pcb --nets CAN1+ CAN1- CAN2+ CAN2- '/OBC module/LCL/2IN+' '/OBC module/LCL/2IN-' '/OBC module/LCL/3IN+' '/OBC module/LCL/3IN-' '/OBC module/LCL/4IN+' '/OBC module/LCL/4IN-' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --track-width 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND /ADCS_VDD '/OBC module/3V3_PROT' '/OBC module/MCU_VDDCORE' '/ADCS module/Microcontroller/MCU_VDDCORE' '/OBC module/MCU_VDDPLL' '/OBC module/MCU_VDDUTMIC' '/ADCS module/Microcontroller/MCU_VDDPLL' '/ADCS module/Microcontroller/MCU_VDDUTMIC' 'EPS VBAT' 'Net-(U19-VDD)' --power-nets-widths 0.4 0.4 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.4 0.3 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --layer-costs 1.0 3.0 1.0 3.0 3.0 1.0 3.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/peaksat_obc_adcs
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --track-width 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND /ADCS_VDD '/OBC module/3V3_PROT' '/OBC module/MCU_VDDCORE' '/ADCS module/Microcontroller/MCU_VDDCORE' '/OBC module/MCU_VDDPLL' '/OBC module/MCU_VDDUTMIC' '/ADCS module/Microcontroller/MCU_VDDPLL' '/ADCS module/Microcontroller/MCU_VDDUTMIC' 'EPS VBAT' 'Net-(U19-VDD)' --power-nets-widths 0.4 0.4 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.4 0.3 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --layer-costs 1.0 3.0 1.0 3.0 3.0 1.0 3.0 1.0
