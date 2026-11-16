#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set21/muzy_zynq2.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --component U2 --output step2_fanout.kicad_pcb --layers F.Cu B.Cu --nets '*' '!GND' --via-size 0.25 --via-drill 0.15 --track-width 0.127 --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/bga_fanout.py step1_planes.kicad_pcb --component U2 --output step2_fanout_retry.kicad_pcb --layers F.Cu B.Cu --nets '*' '!GND' --via-size 0.25 --via-drill 0.15 --track-width 0.1 --clearance 0.1 --escape-method underpad
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step3_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_capclear.kicad_pcb step4_route.kicad_pcb --nets '*' --power-nets GND +3V3 +1V8 +1V0 +5V --power-nets-widths 0.3048 0.3048 0.3048 0.3048 0.3048 --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.500024 --via-drill 0.3048 --hole-to-hole-clearance 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_capclear.kicad_pcb step4_route.kicad_pcb --nets '*' --power-nets GND +3V3 +1V8 +1V0 +5V --power-nets-widths 0.3048 0.3048 0.3048 0.3048 0.3048 --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.500024 --via-drill 0.3048 --hole-to-hole-clearance 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_capclear.kicad_pcb step4_route.kicad_pcb --nets '*' --power-nets GND +3V3 +1V8 +1V0 +5V --power-nets-widths 0.3048 0.3048 0.3048 0.3048 0.3048 --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.500024 --via-drill 0.3048 --hole-to-hole-clearance 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_capclear.kicad_pcb step4_route.kicad_pcb --nets '*' --power-nets GND +3V3 +1V8 +1V0 +5V --power-nets-widths 0.3048 0.3048 0.3048 0.3048 0.3048 --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.500024 --via-drill 0.3048 --hole-to-hole-clearance 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_capclear.kicad_pcb step4_route.kicad_pcb --nets '*' --power-nets GND +3V3 +1V8 +1V0 +5V --power-nets-widths 0.3048 0.3048 0.3048 0.3048 0.3048 --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.500024 --via-drill 0.3048 --hole-to-hole-clearance 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_capclear.kicad_pcb step4_route.kicad_pcb --nets '*' --power-nets GND +3V3 +1V8 +1V0 +5V --power-nets-widths 0.3048 0.3048 0.3048 0.3048 0.3048 --clearance-ceiling 0.127 --track-width 0.127 --via-size 0.500024 --via-drill 0.3048 --hole-to-hole-clearance 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/muzy_zynq2
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step5_retry.kicad_pcb --nets /SDRAM_nRAS /TX1 /TX0 /DQ13 /MDIO SD1_D3 SD1_CMD FPGA_RX +3V3 --power-nets +3V3 --power-nets-widths 0.3048 --clearance-ceiling 0.1 --track-width 0.1 --via-size 0.500024 --via-drill 0.3048 --hole-to-hole-clearance 0.25 --max-ripup 5 --no-bga-zone --grid-step 0.05
