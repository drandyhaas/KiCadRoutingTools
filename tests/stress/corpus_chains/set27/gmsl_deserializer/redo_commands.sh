#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set27/gmsl_deserializer.kicad_pcb --output step1_fanout_U7.kicad_pcb --component U7 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step1_fanout_U7.kicad_pcb step1b_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py step1b_capclear.kicad_pcb step2_planes.kicad_pcb --nets GND +12V --plane-layers In1.Cu In2.Cu --layers F.Cu In1.Cu In2.Cu B.Cu --via-size 0.6 --via-drill 0.25 --track-width 0.125 --clearance-ceiling 0.125 --hole-to-hole-clearance 0.2 --power-nets GND +12V --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2_planes.kicad_pcb step3_diff_csi.kicad_pcb --nets '/Connectors/CSI0.*' '/Connectors/CSI1.*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.155 --diff-pair-gap 0.125 --clearance-ceiling 0.125 --via-size 0.6 --via-drill 0.25 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3_diff_csi.kicad_pcb step3b_diff_csi_retry.kicad_pcb --nets '/Connectors/CSI1.D2*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.155 --diff-pair-gap 0.125 --clearance-ceiling 0.125 --via-size 0.6 --via-drill 0.25 --hole-to-hole-clearance 0.2 --no-gnd-vias --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3b_diff_csi_retry.kicad_pcb step4_diff_sio.kicad_pcb --nets '/Deserializer/SIO*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.125 --diff-pair-gap 0.125 --clearance-ceiling 0.125 --via-size 0.6 --via-drill 0.25 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/gmsl_deserializer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diff_sio.kicad_pcb step5_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.125 --clearance-ceiling 0.125 --via-size 0.6 --via-drill 0.25 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND +12V +1V8 +5V VDDIO +1V2 /Deserializer/VDD /Deserializer/VDD_1.8V /Deserializer/CAP_VDD --power-nets-widths 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3
