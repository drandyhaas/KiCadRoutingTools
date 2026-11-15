#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set1arc/video_latency_probe_arc.kicad_pcb step1_planes.kicad_pcb --nets GND GND +3V3 --plane-layers In1.Cu In4.Cu In2.Cu --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --via-size 0.45 --via-drill 0.2 --track-width 0.125 --clearance-ceiling 0.125 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --clearance 0.125 --width 0.125 --grid-step 0.05 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --clearance 0.125 --width 0.125 --grid-step 0.05 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.1 --grid-step 0.05 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u1_underpad.kicad_pcb --component U1 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.1 --grid-step 0.05 --escape-method underpad --via-size 0.25 --via-drill 0.15 --allow-via-in-pad
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2a_fanout_u1.kicad_pcb step2a_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2a_capclear.kicad_pcb --output step2b_fanout_u2.kicad_pcb --component U2 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.1 --grid-step 0.05 --via-size 0.25 --via-drill 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2b_fanout_u2.kicad_pcb step2b_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2b_capclear.kicad_pcb step3_diffpairs.kicad_pcb --nets /D+ /D- /USB_DP /USB_DN --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.125 --track-width 0.15 --diff-pair-gap 0.125 --impedance 90 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_qspi.kicad_pcb --nets /QSPI_CSn /QSPI_SCLK /QSPI_SD0 /QSPI_SD1 /QSPI_SD2 /QSPI_SD3 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --impedance 50 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_qspi.kicad_pcb step5_signal.kicad_pcb --nets '*' '!/QSPI_CSn' '!/QSPI_SCLK' '!/QSPI_SD0' '!/QSPI_SD1' '!/QSPI_SD2' '!/QSPI_SD3' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.125 --track-width 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 +1V8 /VBUS +1V1 /VBUS_Q --power-nets-widths 0.3 0.3 0.25 0.4 0.25 0.3 --max-ripup 5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set1arc/video_latency_probe_arc
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step5_signal.kicad_pcb step6_retry.kicad_pcb --nets /SCL /ICM_INT1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --clearance-ceiling 0.09 --track-width 0.1 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --rip-existing-nets '*' --max-ripup 5 --power-nets GND +3V3 +1V8 /VBUS +1V1 /VBUS_Q --power-nets-widths 0.3 0.3 0.25 0.4 0.25 0.3
