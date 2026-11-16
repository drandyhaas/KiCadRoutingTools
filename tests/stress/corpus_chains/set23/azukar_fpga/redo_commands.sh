#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/azukar_fpga
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set23/azukar_fpga.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/azukar_fpga
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2a_fanout_u4.kicad_pcb --component U4 --nets '*' '!GND' --width 0.15 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/azukar_fpga
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2a_fanout_u4.kicad_pcb -o step2b_fanout_u6.kicad_pcb --component U6 --nets '*' '!GND' --width 0.15 --clearance 0.1 --grid-step 0.05
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/azukar_fpga
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2b_fanout_u6.kicad_pcb step2c_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/azukar_fpga
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2c_clearance.kicad_pcb step3_diff.kicad_pcb --nets '/Power Supply/D+' '/Power Supply/D-' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/azukar_fpga
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.1 --grid-step 0.05 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-bga-zone --max-ripup 5 --layer-costs 1.0 1.5 --power-nets GND +3V3 +5V +1V2 +1V8 'Net-(U3-VBUS)' '/Memorias y PLL/VCCPLL1' '/Memorias y PLL/VCCPLL2' '/Memorias y PLL/GNDPLL1' '/Memorias y PLL/GNDPLL2' --power-nets-widths 0.3 0.5 1.0 0.3 0.3 0.3 0.3 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/azukar_fpga
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.1 --grid-step 0.05 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-bga-zone --max-ripup 5 --layer-costs 1.0 1.5 --power-nets GND +3V3 +5V +1V2 +1V8 'Net-(U3-VBUS)' '/Memorias y PLL/VCCPLL1' '/Memorias y PLL/VCCPLL2' '/Memorias y PLL/GNDPLL1' '/Memorias y PLL/GNDPLL2' --power-nets-widths 0.3 0.5 1.0 0.3 0.3 0.3 0.3 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/azukar_fpga
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.1 --grid-step 0.05 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-bga-zone --max-ripup 5 --layer-costs 1.0 1.5 --power-nets GND +3V3 +5V +1V2 +1V8 'Net-(U3-VBUS)' '/Memorias y PLL/VCCPLL1' '/Memorias y PLL/VCCPLL2' '/Memorias y PLL/GNDPLL1' '/Memorias y PLL/GNDPLL2' --power-nets-widths 0.3 0.5 1.0 0.3 0.3 0.3 0.3 0.3 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set23/azukar_fpga
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.1 --grid-step 0.05 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --no-bga-zone --max-ripup 5 --layer-costs 1.0 1.5 --power-nets GND +3V3 +5V +1V2 +1V8 'Net-(U3-VBUS)' '/Memorias y PLL/VCCPLL1' '/Memorias y PLL/VCCPLL2' '/Memorias y PLL/GNDPLL1' '/Memorias y PLL/GNDPLL2' --power-nets-widths 0.3 0.5 1.0 0.3 0.3 0.3 0.3 0.3 0.3 0.3
