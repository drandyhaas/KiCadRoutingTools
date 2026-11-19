#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/dual_ipex_csi_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/dual_ipex_csi_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set32/dual_ipex_csi_interposer.kicad_pcb step1_planes.kicad_pcb --nets GND 5V0_SYS --plane-layers In1.Cu In2.Cu --via-size 0.6 --via-drill 0.25 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2 --power-nets GND 5V0_SYS 3V3_FFC --power-nets-widths 0.4 0.4 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/dual_ipex_csi_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/dual_ipex_csi_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/dual_ipex_csi_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb --nets CAM1_CSI_CLK CAM1_CSI_D0 CAM1_CSI_D1 CAM1_CSI_D2 CAM1_CSI_D3 CAM2_CSI_CLK CAM2_CSI_D0 CAM2_CSI_D1 CAM2_CSI_D2 CAM2_CSI_D3 --track-width 0.15 --diff-pair-gap 0.19 --clearance-ceiling 0.15 --via-size 0.6 --via-drill 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/dual_ipex_csi_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/dual_ipex_csi_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step1_planes.kicad_pcb step2_diff.kicad_pcb --nets 'CAM1_CSI_CLK_*' 'CAM1_CSI_D0_*' 'CAM1_CSI_D1_*' 'CAM1_CSI_D2_*' 'CAM1_CSI_D3_*' 'CAM2_CSI_CLK_*' 'CAM2_CSI_D0_*' 'CAM2_CSI_D1_*' 'CAM2_CSI_D2_*' 'CAM2_CSI_D3_*' --track-width 0.15 --diff-pair-gap 0.19 --clearance-ceiling 0.15 --via-size 0.6 --via-drill 0.25 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/dual_ipex_csi_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step1_planes.kicad_pcb step2b_diff_clk_retry.kicad_pcb --nets 'CAM1_CSI_CLK_*' --track-width 0.15 --diff-pair-gap 0.19 --clearance-ceiling 0.15 --via-size 0.6 --via-drill 0.25 --no-gnd-vias --direction backward
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/dual_ipex_csi_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step1_planes.kicad_pcb step2_diff_v2.kicad_pcb --nets 'CAM1_CSI_CLK_*' 'CAM1_CSI_D0_*' 'CAM1_CSI_D1_*' 'CAM1_CSI_D2_*' 'CAM1_CSI_D3_*' 'CAM2_CSI_CLK_*' 'CAM2_CSI_D0_*' 'CAM2_CSI_D1_*' 'CAM2_CSI_D2_*' 'CAM2_CSI_D3_*' --track-width 0.15 --diff-pair-gap 0.19 --clearance-ceiling 0.15 --via-size 0.6 --via-drill 0.25 --no-gnd-vias --direction backward
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/dual_ipex_csi_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set32/dual_ipex_csi_interposer
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_diff.kicad_pcb step3_route.kicad_pcb --nets '*' --layers F.Cu B.Cu --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.6 --via-drill 0.25 --hole-to-hole-clearance 0.2 --power-nets GND 5V0_SYS 3V3_FFC --power-nets-widths 0.4 0.4 0.3
