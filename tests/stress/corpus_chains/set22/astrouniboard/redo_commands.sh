#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set22/astrouniboard.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.127 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2a_fanout_U2.kicad_pcb --component U2 --nets '*' '!GND' --clearance 0.1 --width 0.1 --extension 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2a_fanout_U2.kicad_pcb -o step2b_fanout_U3.kicad_pcb --component U3 --nets '*' '!GND' --clearance 0.1 --width 0.1 --extension 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_fanout_U3.kicad_pcb -o step2c_fanout_U5.kicad_pcb --component U5 --nets '*' '!GND' --clearance 0.1 --width 0.1 --extension 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2c_fanout_U5.kicad_pcb -o step2d_fanout_U6.kicad_pcb --component U6 --nets '*' '!GND' --clearance 0.1 --width 0.1 --extension 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2c_fanout_U5.kicad_pcb -o step2d_fanout_U6.kicad_pcb --component U6 --nets '*' '!GND' --clearance 0.1 --width 0.1 --extension 0.1 --escape-method underpad --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2c_fanout_U5.kicad_pcb -o step2d_fanout_U6.kicad_pcb --component U6 --nets '*' '!GND' --clearance 0.1 --width 0.1 --extension 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2d_fanout_U6.kicad_pcb -o step2e_fanout_U10.kicad_pcb --component U10 --nets '*' '!GND' --clearance 0.1 --width 0.1 --extension 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2e_fanout_U10.kicad_pcb step2f_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2f_capclear.kicad_pcb step3_diff.kicad_pcb --nets /DP /DM --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.5 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +3.3V +5V +12V +5V3 Vdrive1 --power-nets-widths 0.3 0.25 0.4 0.6 0.4 0.6
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +5V +12V +5V3 Vdrive1 --power-nets-widths 0.3 0.25 0.4 0.6 0.4 0.6
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +5V +12V +5V3 Vdrive1 --power-nets-widths 0.3 0.25 0.4 0.6 0.4 0.6
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +5V +12V +5V3 Vdrive1 --power-nets-widths 0.3 0.25 0.4 0.6 0.4 0.6
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.127 --track-width 0.1 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +5V +12V +5V3 Vdrive1 --power-nets-widths 0.3 0.25 0.4 0.6 0.4 0.6
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step5_retry.kicad_pcb --nets 'Net-(U5-PB9)' /HALL 'Net-(U12-FB)' --force-reroute --rip-existing-nets 'Net-(U5-PB9)' /HALL 'Net-(U12-FB)' --clearance-ceiling 0.127 --track-width 0.1 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5 --power-nets GND +3.3V +5V +12V +5V3 Vdrive1 --power-nets-widths 0.3 0.25 0.4 0.6 0.4 0.6
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set22/astrouniboard
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step5_retry.kicad_pcb step6_gndfix.kicad_pcb --nets GND --clearance-ceiling 0.127 --track-width 0.1 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND --power-nets-widths 0.3
