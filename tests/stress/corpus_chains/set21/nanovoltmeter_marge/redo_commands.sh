#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set21/nanovoltmeter_marge.kicad_pcb step1_planes.kicad_pcb --nets GND GNDREF /nvm_pmain_fpsu/FGND_A /nvm_pmain_fpsu/FGND_D --plane-layers B.Cu B.Cu B.Cu B.Cu --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u9.kicad_pcb --component U9 --nets '*' '!GND' --clearance 0.1 --width 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout_u9.kicad_pcb step3_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step3_capclear.kicad_pcb step4_diffpair.kicad_pcb --nets /nvm_pmain_amp/OUT_P /nvm_pmain_amp/OUT_N --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpair.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND GNDREF /nvm_pmain_fpsu/FGND_A /nvm_pmain_fpsu/FGND_D +3V3 +12V +5V +3.3VADC +9VA --power-nets-widths 0.4 0.4 0.4 0.4 0.3 0.4 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpair.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND GNDREF /nvm_pmain_fpsu/FGND_A /nvm_pmain_fpsu/FGND_D +3V3 +12V +5V +3.3VADC +9VA --power-nets-widths 0.4 0.4 0.4 0.4 0.3 0.4 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpair.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND GNDREF /nvm_pmain_fpsu/FGND_A /nvm_pmain_fpsu/FGND_D +3V3 +12V +5V +3.3VADC +9VA --power-nets-widths 0.4 0.4 0.4 0.4 0.3 0.4 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpair.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND GNDREF /nvm_pmain_fpsu/FGND_A /nvm_pmain_fpsu/FGND_D +3V3 +12V +5V +3.3VADC +9VA --power-nets-widths 0.4 0.4 0.4 0.4 0.3 0.4 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpair.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND GNDREF /nvm_pmain_fpsu/FGND_A /nvm_pmain_fpsu/FGND_D +3V3 +12V +5V +3.3VADC +9VA --power-nets-widths 0.4 0.4 0.4 0.4 0.3 0.4 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpair.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND GNDREF /nvm_pmain_fpsu/FGND_A /nvm_pmain_fpsu/FGND_D +3V3 +12V +5V +3.3VADC +9VA --power-nets-widths 0.4 0.4 0.4 0.4 0.3 0.4 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/nanovoltmeter_marge
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_diffpair.kicad_pcb step5_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND GNDREF /nvm_pmain_fpsu/FGND_A /nvm_pmain_fpsu/FGND_D +3V3 +12V +5V +3.3VADC +9VA --power-nets-widths 0.4 0.4 0.4 0.4 0.3 0.4 0.3 0.3 0.3
