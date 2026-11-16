#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/sdi_fiber_adapter.kicad_pcb step1_planes.kicad_pcb --nets GND /Fiber/SH +2V5 --plane-layers In1.Cu In1.Cu In2.Cu --clearance-ceiling 0.125 --via-size 0.55 --via-drill 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --output step2a_fanout_u1.kicad_pcb --nets '*' '!GND' '!+2V5' --width 0.125 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2a_fanout_u1.kicad_pcb --component U2 --output step2b_fanout_u2.kicad_pcb --nets '*' '!GND' '!+2V5' --width 0.125 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2b_fanout_u2.kicad_pcb --component U7 --output step2c_fanout_u7.kicad_pcb --nets '*' '!GND' '!+2V5' --width 0.125 --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2c_fanout_u7.kicad_pcb step2d_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2d_capclear.kicad_pcb step3_diffpairs.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --impedance 100 --via-size 0.55 --via-drill 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/sdi_fiber_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu B.Cu --clearance-ceiling 0.125 --track-width 0.125 --via-size 0.55 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --power-nets GND /Fiber/SH +2V5 +3V3 +5V /Power/USBC_VBUS 'Net-(U2-VDD_LDO)' 'Net-(U1-VDD_LDO)' --power-nets-widths 0.3 0.3 0.3 0.3 0.4 0.5 0.25 0.25 --max-ripup 5
