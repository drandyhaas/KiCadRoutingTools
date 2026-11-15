#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/private/tmp/wt_baseline_0809
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set27/audio_latency_tester.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.125 --zone-clearance 0.09 --hole-to-hole-clearance 0.25 --power-nets GND +3V3 --power-nets-widths 0.4 0.4
# cwd=/private/tmp/wt_baseline_0809
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_U9.kicad_pcb --component U9 --nets '!GND' '!+3V3' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.1 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_U9.kicad_pcb --component U9 --nets '!GND' '!+3V3' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.1 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2a_fanout_U9.kicad_pcb --output step2b_fanout_U12.kicad_pcb --component U12 --nets '!GND' '!+3V3' --escape-method underpad --allow-via-in-pad --via-size 0.25 --via-drill 0.15 --width 0.1 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_fanout_U12.kicad_pcb --output step2c_fanout_U21.kicad_pcb --component U21 --nets '!GND' '!+3V3' --clearance 0.09 --width 0.125 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2c_fanout_U21.kicad_pcb --output step2d_fanout_U17.kicad_pcb --component U17 --nets '!GND' '!+3V3' --clearance 0.09 --width 0.125 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2d_fanout_U17.kicad_pcb step2e_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2e_capclear.kicad_pcb --output step3a_diff_usb.kicad_pcb --nets '*USB1.D_P' '*USB1.D_N' '*USB2.D_P' '*USB2.D_N' 'Net-(U12-USB_DP)' 'Net-(U12-USB_DM)' 'Net-(U9-USB_DP)' 'Net-(U9-USB_DM)' --impedance 90 --clearance-ceiling 0.15 --diff-pair-gap 0.15 --track-width 0.182 --via-size 0.25 --via-drill 0.15 --hole-to-hole-clearance 0.25 --layers F.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3a_diff_usb.kicad_pcb --output step3b_diff_spk.kicad_pcb --nets '*SPK_P' '*SPK_N' --clearance-ceiling 0.125 --diff-pair-gap 0.125 --track-width 0.15 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --layers F.Cu B.Cu --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set27/audio_latency_tester
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3b_diff_spk.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.125 --track-width 0.127 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --grid-step 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --power-nets GND +3V3 +5V '/2xUSB + Power/USB1_VBUS' '/2xUSB + Power/USB2_VBUS' +1V8 --power-nets-widths 0.4 0.4 0.4 0.4 0.4 0.3 --max-ripup 5
