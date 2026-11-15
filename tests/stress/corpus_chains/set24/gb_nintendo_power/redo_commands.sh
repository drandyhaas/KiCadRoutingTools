#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/gb_nintendo_power
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/gb_nintendo_power
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set24/gb_nintendo_power.kicad_pcb step1_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/gb_nintendo_power
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set24/gb_nintendo_power.kicad_pcb step1_planes.kicad_pcb --nets /GND --plane-layers B.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/gb_nintendo_power
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/gb_nintendo_power
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --clearance 0.1 --nets '*' '!/GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/gb_nintendo_power
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/gb_nintendo_power
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/gb_nintendo_power
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_fanout_u1.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.13 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets /GND /VCC /SRAM.VCC /Batt.VCC --power-nets-widths 0.5 0.5 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/gb_nintendo_power
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_fanout_u1.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.1 --track-width 0.13 --grid-step 0.05 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets /GND /VCC /SRAM.VCC /Batt.VCC --power-nets-widths 0.5 0.5 0.3 0.3 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/gb_nintendo_power
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_fanout_u1.kicad_pcb step3a_route_u1.kicad_pcb --nets '' --clearance-ceiling 0.1 --track-width 0.13 --grid-step 0.05 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --power-nets /VCC /SRAM.VCC /Batt.VCC --power-nets-widths 0.3 0.25 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/gb_nintendo_power
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_fanout_u1.kicad_pcb step3a_route_u1.kicad_pcb --nets /A0.Out /A0.Sys /A1.Out /A1.Sys /A10.Out /A10.Sys /A11.Out /A11.Sys /A12.Out /A12.Sys /A13.Out /A13.Sys /A14.Out /A14.Sys /A15.Out /A15.Sys /A16.Out /A17.Out /A18.Out /A19.Out /A2.Out /A2.Sys /A3.Out /A3.Sys /A4.Out /A4.Sys /A5.Out /A5.Sys /A6.Out /A6.Sys /A7.Out /A7.Sys /A8.Out /A8.Sys /A9.Out /A9.Sys /Batt.VCC /D0 /D1 /D2 /D3 /D4 /D5 /D6 /D7 /P10.R6 /SRAM.VCC '/SRAM.~{CE1}' /VCC '/~{CS.Sys}' '/~{Flash.CE}' '/~{Flash.Reset}' '/~{Flash.WP}' '/~{OE}.Out' '/~{Read.Sys}' '/~{Reset.Sys}' '/~{WE}.Out' '/~{Write.Sys}' 'Net-(C2-Pad1)' 'Net-(R4-Pad2)' 'Net-(U1-Clk.In)' 'Net-(U1-Clk.Out)' 'Net-(U1-Pad54)' --clearance-ceiling 0.1 --track-width 0.13 --grid-step 0.05 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --power-nets /VCC /SRAM.VCC /Batt.VCC --power-nets-widths 0.3 0.25 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/gb_nintendo_power
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step2_fanout_u1.kicad_pcb step3_route.kicad_pcb --nets '*' --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets /GND /VCC /SRAM.VCC /Batt.VCC --power-nets-widths 0.4 0.3 0.25 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set24/gb_nintendo_power
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_route.kicad_pcb step4_retry_a7out.kicad_pcb --nets /A7.Out --clearance-ceiling 0.1 --track-width 0.1 --grid-step 0.05 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --max-ripup 8 --no-bga-zone
