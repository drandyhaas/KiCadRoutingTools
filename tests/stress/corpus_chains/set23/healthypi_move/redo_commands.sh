#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set23/healthypi_move.kicad_pcb step1_planes.kicad_pcb --nets GND 3V --plane-layers In2.Cu In3.Cu --clearance-ceiling 0.1524 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --track-width 0.2 --power-nets GND 3V --power-nets-widths 0.3 0.3 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2a_fanout_u5.kicad_pcb --component 'U$5' --nets '*' '!GND' '!3V' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2a_fanout_u5.kicad_pcb step2a_fanout_u5_pfc.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2a_fanout_u5_pfc.kicad_pcb -o step2b_fanout_u4.kicad_pcb --component U4 --nets '*' '!GND' '!3V' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2b_fanout_u4.kicad_pcb step2b_fanout_u4_pfc.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_fanout_u4_pfc.kicad_pcb -o step2c_fanout_u3.kicad_pcb --component U3 --nets '*' '!GND' '!3V' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2c_fanout_u3.kicad_pcb step2c_fanout_u3_pfc.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2c_fanout_u3_pfc.kicad_pcb -o step2d_fanout_mdbt1.kicad_pcb --component MDBT1 --nets '*' '!GND' '!3V' --clearance 0.09 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2d_fanout_mdbt1.kicad_pcb step2d_fanout_mdbt1_pfc.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2d_fanout_mdbt1_pfc.kicad_pcb step3_diff.kicad_pcb --nets USB_D+ USB_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --hole-to-hole-clearance 0.2 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --power-nets GND 3V --power-nets-widths 0.3 0.3 --grid-step 0.05 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --hole-to-hole-clearance 0.2 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --power-nets GND 3V --power-nets-widths 0.3 0.3 --grid-step 0.05 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --hole-to-hole-clearance 0.2 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --power-nets GND 3V --power-nets-widths 0.3 0.3 --grid-step 0.05 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --hole-to-hole-clearance 0.2 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --power-nets GND 3V --power-nets-widths 0.3 0.3 --grid-step 0.05 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1524 --hole-to-hole-clearance 0.2 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --power-nets GND 3V --power-nets-widths 0.3 0.3 --grid-step 0.05 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/healthypi_move
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step4b_retry.kicad_pcb --nets I2C1_SCL 3V I2C1_SDA --rip-existing-nets '*' --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --track-width 0.0762 --via-size 0.5 --via-drill 0.3 --power-nets 3V --power-nets-widths 0.3 --grid-step 0.05 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --max-ripup 5
