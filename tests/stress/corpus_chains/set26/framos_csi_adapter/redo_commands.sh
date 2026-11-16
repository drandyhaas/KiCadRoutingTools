#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/private/tmp/wt_baseline_0809/py_router
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/framos_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/framos_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set26/framos_csi_adapter.kicad_pcb step1_planes.kicad_pcb --nets GND GND --plane-layers In1.Cu In2.Cu --via-size 0.6 --via-drill 0.25 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/framos_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_j1fanout.kicad_pcb --component J1 --nets '*' '!GND' --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/framos_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_j1fanout.kicad_pcb --component J1 --nets '*' '!GND' --clearance 0.09 --width 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/framos_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_j1fanout.kicad_pcb step3_j1clearance.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/framos_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/framos_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3_j1clearance.kicad_pcb step4_diffpairs.kicad_pcb --nets 'CSI_CLK_*' 'CSI_D0_*' 'CSI_D1_*' 'CSI_D2_*' 'CSI_D3_*' --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1 -1 -1 1 --track-width 0.15 --diff-pair-gap 0.19 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2 --via-size 0.6 --via-drill 0.25 --impedance 100 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/framos_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step4_diffpairs.kicad_pcb step4b_diffretry.kicad_pcb --nets 'CSI_D1_*' 'CSI_D3_*' --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1 -1 -1 1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.25 --via-drill 0.15 --impedance 100 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/framos_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set26/framos_csi_adapter
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4b_diffretry.kicad_pcb step5_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1 -1 -1 1 --track-width 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.6 --via-drill 0.25 --power-nets GND 1V8_CAM 3V8_CAM 5V0_SYS 3V3FFC --power-nets-widths 0.3 0.3 0.3 0.4 0.3 --no-bga-zone --max-ripup 5
