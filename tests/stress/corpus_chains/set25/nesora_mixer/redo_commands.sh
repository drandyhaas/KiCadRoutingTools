#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/nesora_mixer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/nesora_mixer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/nesora_mixer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/nesora_mixer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/nesora_mixer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/nesora_mixer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set25/nesora_mixer.kicad_pcb step1_planes.kicad_pcb --nets GNDD --plane-layers B.Cu --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/nesora_mixer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u4.kicad_pcb --component U4 --width 0.1 --clearance 0.1 --via-size 0.25 --via-drill 0.15 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/nesora_mixer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout_u4.kicad_pcb step3_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/nesora_mixer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpairs.kicad_pcb --nets 'Net-(U13-D+)' 'Net-(U13-D-)' USB2_Audio_D+ USB2_Audio_D- --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/nesora_mixer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpairs.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GNDD /PCM2706_VDD +5VD +3.3VA VCC +3.3V 'Net-(U13-VCCR)' 'Net-(U13-VCCL)' 'Net-(U13-VCCP)' -5V +5V --power-nets-widths 0.4 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 --max-ripup 5
