#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kishoof_components
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kishoof_components
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kishoof_components
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set15/kishoof_components.kicad_pcb step1_planes.kicad_pcb --nets GND +3.3V +3.3VA +12V -12V --plane-layers In1.Cu In2.Cu In2.Cu In2.Cu In2.Cu --clearance-ceiling 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kishoof_components
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --component U1 --nets '*' '!GND' '!+3.3V' '!+3.3VA' '!+12V' '!-12V' --width 0.15 --clearance 0.09 --grid-step 0.1 --output step1b_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kishoof_components
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_fanout_u1.kicad_pcb step1c_capopt.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kishoof_components
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kishoof_components
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kishoof_components
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1c_capopt.kicad_pcb step2a_diffpairs.kicad_pcb --nets '/USB_D*' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kishoof_components
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --grid-step 0.05 --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +3.3VA +12V -12V /+12V_IN /USB_VBUS --power-nets-widths 0.3 0.3 0.3 0.5 0.5 0.5 0.4 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kishoof_components
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --grid-step 0.05 --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +3.3VA +12V -12V /+12V_IN /USB_VBUS --power-nets-widths 0.3 0.3 0.3 0.5 0.5 0.5 0.4 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kishoof_components
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --grid-step 0.05 --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +3.3VA +12V -12V /+12V_IN /USB_VBUS --power-nets-widths 0.3 0.3 0.3 0.5 0.5 0.5 0.4 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kishoof_components
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --grid-step 0.05 --clearance-ceiling 0.15 --track-width 0.15 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3.3V +3.3VA +12V -12V /+12V_IN /USB_VBUS --power-nets-widths 0.3 0.3 0.3 0.5 0.5 0.5 0.4 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 1.5 1.0
