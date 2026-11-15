#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/private/tmp/wt_baseline_0809
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set23/miniscope_v4_flex.kicad_pcb step1_planes.kicad_pcb --nets GND GND +3V3 +1V8 --plane-layers B.Cu In1.Cu In4.Cu In4.Cu --clearance-ceiling 0.1016 --via-size 0.45 --via-drill 0.2 --power-nets GND +3V3 +1V8 --power-nets-widths 0.25 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2a_fanout_U6.kicad_pcb --component U6 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' --clearance 0.1016 --track-width 0.1016 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2a_fanout_U6.kicad_pcb step2a_fanout_U6_capcleared.kicad_pcb --clearance 0.1016
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2a_fanout_U6.kicad_pcb step2a_fanout_U6_capcleared.kicad_pcb --clearance 0.1016 --verbose
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2a_fanout_U6.kicad_pcb step2a_fanout_U6_capcleared_test.kicad_pcb --clearance 0.1016 --verbose
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step2a_fanout_U6.kicad_pcb --output step2b_fanout_U4.kicad_pcb --component U4 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' --clearance 0.1016 --track-width 0.1016 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step2b_fanout_U4.kicad_pcb --output step2c_fanout_U12.kicad_pcb --component U12 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --nets '*' '!GND' --clearance 0.1016 --track-width 0.1016 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2c_fanout_U12.kicad_pcb --output step2d_fanout_U9.kicad_pcb --component U9 --nets '*' '!GND' --width 0.1016 --clearance 0.1016 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2c_fanout_U12.kicad_pcb --output step2d_fanout_U9.kicad_pcb --component U9 --nets '*' '!GND' --escape-method underpad --allow-via-in-pad --width 0.1016 --clearance 0.09 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2d_fanout_U9.kicad_pcb --output step2e_fanout_U1.kicad_pcb --component U1 --nets '*' '!GND' --escape-method underpad --allow-via-in-pad --width 0.1016 --clearance 0.09 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2e_fanout_U1.kicad_pcb --output step2f_fanout_U3.kicad_pcb --component U3 --nets '*' '!GND' --escape-method underpad --allow-via-in-pad --width 0.09 --clearance 0.09 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2f_fanout_U3.kicad_pcb step3_capcleared.kicad_pcb --clearance 0.1016
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2f_fanout_U3.kicad_pcb step3_capcleared.kicad_pcb --clearance 0.1016
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_capcleared.kicad_pcb step4_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 1.2 3.0 1.0 1.0 1.0 --clearance-ceiling 0.1016 --track-width 0.1016 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +1V8 +5V /PYTHON480/VDD_PIX /Ser-Pow/VDDPLL /Ser-Pow/VDDCML /PYTHON480/VDD_PIX_3V3 --power-nets-widths 0.25 0.3 0.3 0.3 0.2 0.2 0.2 0.2 --grid-step 0.05 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step5_retry_i2c.kicad_pcb --nets /IMU/I2C_SDA --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 1.2 3.0 1.0 1.0 1.0 --clearance-ceiling 0.09 --track-width 0.0889 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --grid-step 0.025 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step5_retry_i2c.kicad_pcb --nets /IMU/I2C_SDA --rip-existing-nets +1V8 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 1.2 3.0 1.0 1.0 1.0 --clearance-ceiling 0.09 --track-width 0.0889 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.2 --power-nets +1V8 --power-nets-widths 0.2 --grid-step 0.025 --no-bga-zone --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/miniscope_v4_flex
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step5_retry_i2c.kicad_pcb step6_fix_gnd.kicad_pcb --nets GND --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 1.2 3.0 1.0 1.0 1.0 --clearance-ceiling 0.09 --track-width 0.1016 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND --power-nets-widths 0.25 --grid-step 0.05 --no-bga-zone --max-ripup 5
