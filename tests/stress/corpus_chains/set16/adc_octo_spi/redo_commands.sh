#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set16/adc_octo_spi.kicad_pcb step1_planes.kicad_pcb --nets GND GNDA --plane-layers B.Cu B.Cu --via-size 0.45 --via-drill 0.2 --track-width 0.3 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb step2_fanout.kicad_pcb --component U1 --nets '*' '!GND' '!GNDA' --clearance 0.2 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout.kicad_pcb --component U1 --nets '*' '!GND' '!GNDA' --clearance 0.2 --width 0.15 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout.kicad_pcb step2b_capclear.kicad_pcb --clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2b_capclear.kicad_pcb step3_diff.kicad_pcb --nets '*_P' --track-width 0.15 --diff-pair-gap 0.15 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --power-nets GND GNDA DVDD3V3 DVDD VCC AVDD AVDD3V3 --power-nets-widths 0.3 0.3 0.4 0.4 0.4 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3_diff.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --power-nets GND GNDA DVDD3V3 DVDD VCC AVDD AVDD3V3 --power-nets-widths 0.3 0.3 0.4 0.4 0.4 0.4 0.4
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4_route.kicad_pcb step5_fclkfix.kicad_pcb --nets /FCLK_P --track-width 0.15 --diff-pair-gap 0.18 --clearance-ceiling 0.15 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set16/adc_octo_spi
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_route.kicad_pcb step5_fclkfix.kicad_pcb --nets /FCLK_P /FCLK_N --rip-existing-nets /FCLK_P /FCLK_N --force-reroute --clearance-ceiling 0.15 --track-width 0.15 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.2 --grid-step 0.05
