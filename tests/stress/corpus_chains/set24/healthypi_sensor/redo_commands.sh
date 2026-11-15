#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set24/healthypi_sensor.kicad_pcb step1_planes.kicad_pcb --nets GND GND 1.8V --plane-layers In1.Cu In4.Cu In2.Cu --clearance-ceiling 0.1016 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --component 'U$3' --output step2a_qfn_us3.kicad_pcb --width 0.15 --clearance 0.1016 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2a_qfn_us3.kicad_pcb --component U2 --output step2b_qfn_u2.kicad_pcb --width 0.15 --clearance 0.1016 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_qfn_u2.kicad_pcb --component U5 --output step2c_qfn_u5.kicad_pcb --width 0.15 --clearance 0.1016 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step2c_qfn_u5.kicad_pcb --component 'U$30' --output step2d_bga_u30.kicad_pcb --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' '!1.8V' --diff-pairs '*/EP' '*/EN' 'Net-(U$30-BIP)' 'Net-(U$30-BIN)' 'Net-(U$30-CAPP)' 'Net-(U$30-CAPN)' 'Net-(U$30-DRVP)' 'Net-(U$30-DRVN)' --diff-pair-gap 0.1 --clearance 0.09 --track-width 0.15 --via-size 0.25 --via-drill 0.15 --plane-net-layers GND:In1.Cu,In4.Cu 1.8V:In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step2d_bga_u30.kicad_pcb --component U3 --output step2e_bga_u3.kicad_pcb --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' '!1.8V' '!+5V' --clearance 0.09 --track-width 0.1 --via-size 0.25 --via-drill 0.15 --plane-net-layers GND:In1.Cu,In4.Cu 1.8V:In2.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2e_bga_u3.kicad_pcb step2f_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2f_capclear.kicad_pcb step3_diffpairs.kicad_pcb '*/ECGP' '*/ECGN' '*/EP' '*/EN' 'Net-(U$30-BIP)' 'Net-(U$30-BIN)' 'Net-(U$30-CAPP)' 'Net-(U$30-CAPN)' 'Net-(U$30-DRVP)' 'Net-(U$30-DRVN)' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.09 --via-size 0.25 --via-drill 0.15 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.1016 --track-width 0.2 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND 1.8V +5V 3V0 --power-nets-widths 0.3 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/healthypi_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step5_retry.kicad_pcb --nets /pc_healthypi_move_sensor_v7_1/RESET /pc_healthypi_move_sensor_v7_2/W_SEN_MISO --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.1016 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 5 --power-nets GND 1.8V +5V 3V0 --power-nets-widths 0.3 0.3 0.3 0.3
