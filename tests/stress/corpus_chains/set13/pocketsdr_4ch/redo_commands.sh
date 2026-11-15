#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set13/pocketsdr_4ch.kicad_pcb step1_planes.kicad_pcb --nets GND +3.3V --plane-layers In1.Cu In2.Cu --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --component U7 --nets '*' '!GND' '!+3.3V' --diff-pairs 'Net-(U11-RX*)' 'Net-(U7-SSTX*)' --diff-pair-gap 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance 0.09 --via-size 0.45 --via-drill 0.3 --track-width 0.1 --escape-method dogbone --plane-net-layers GND:In1.Cu +3.3V:In2.Cu --output step1b_fanout_u7.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --component U7 --nets '*' '!GND' '!+3.3V' --diff-pairs 'Net-(U11-RX*)' 'Net-(U7-SSTX*)' --diff-pair-gap 0.1 --layers F.Cu In1.Cu In2.Cu B.Cu --clearance 0.09 --via-size 0.45 --via-drill 0.3 --track-width 0.08 --escape-method dogbone --plane-net-layers GND:In1.Cu +3.3V:In2.Cu --output step1b_fanout_u7_try2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1b_fanout_u7_try2.kicad_pcb step1c_cap_opt.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1c_cap_opt.kicad_pcb --component U1 --nets '*' '!GND' '!+3.3V' --width 0.15 --clearance 0.09 --grid-step 0.05 --output step1d_fanout_u1.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1d_fanout_u1.kicad_pcb --component U2 --nets '*' '!GND' '!+3.3V' --width 0.15 --clearance 0.09 --grid-step 0.05 --output step1e_fanout_u2.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1e_fanout_u2.kicad_pcb --component U3 --nets '*' '!GND' '!+3.3V' --width 0.15 --clearance 0.09 --grid-step 0.05 --output step1f_fanout_u3.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1f_fanout_u3.kicad_pcb --component U4 --nets '*' '!GND' '!+3.3V' --width 0.15 --clearance 0.09 --grid-step 0.05 --output step1g_fanout_u4.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1g_fanout_u4.kicad_pcb --component U11 --nets '*' '!GND' '!+3.3V' --width 0.1 --clearance 0.09 --grid-step 0.05 --output step1h_fanout_u11.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step1h_fanout_u11.kicad_pcb step1i_cap_opt2.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step1i_cap_opt2.kicad_pcb step2a_diffpairs.kicad_pcb --nets /USB2_DP /USB2_DM 'Net-(U11-RX+)' 'Net-(U11-RX-)' 'Net-(U11-TX+)' 'Net-(U11-TX-)' 'Net-(U7-SSTXP)' 'Net-(U7-SSTXM)' /SSTXP1 /SSTXN1 /SSTXP2 /SSTXN2 /SSRXP1 /SSRXN1 /SSRXP2 /SSRXN2 --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --impedance 90 --via-size 0.45 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --grid-step 0.05 --track-width 0.127 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.3 --power-nets GND +3.3V /+1.2V /VBUS --power-nets-widths 0.4 0.4 0.4 0.5 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2a_diffpairs.kicad_pcb step2_route.kicad_pcb --nets '*' --no-bga-zone --max-ripup 5 --grid-step 0.05 --track-width 0.127 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.3 --power-nets GND +3.3V /+1.2V /VBUS --power-nets-widths 0.4 0.4 0.4 0.5 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step2_fix.kicad_pcb --rip-existing-nets 'Net-(U11-RX+)' 'Net-(U7-SSTXM)' 'Net-(U7-SSTXP)' /USB2_DM --nets 'Net-(U11-RX+)' 'Net-(U7-SSTXM)' 'Net-(U7-SSTXP)' /USB2_DM --no-bga-zone --max-ripup 5 --grid-step 0.05 --track-width 0.127 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.3 --power-nets GND +3.3V /+1.2V /VBUS --power-nets-widths 0.4 0.4 0.4 0.5 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_route.kicad_pcb step2_fix.kicad_pcb --rip-existing-nets 'Net-(U11-RX+)' 'Net-(U7-SSTXM)' 'Net-(U7-SSTXP)' /USB2_DM --force-reroute --nets 'Net-(U11-RX+)' 'Net-(U7-SSTXM)' 'Net-(U7-SSTXP)' /USB2_DM --no-bga-zone --max-ripup 5 --grid-step 0.05 --track-width 0.127 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.3 --power-nets GND +3.3V /+1.2V /VBUS --power-nets-widths 0.4 0.4 0.4 0.5 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set13/pocketsdr_4ch
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step2_fix.kicad_pcb step2_fix2.kicad_pcb --nets GND +3.3V --no-bga-zone --max-ripup 5 --grid-step 0.05 --track-width 0.127 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.3 --power-nets GND +3.3V /+1.2V /VBUS --power-nets-widths 0.4 0.4 0.4 0.5 --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0
