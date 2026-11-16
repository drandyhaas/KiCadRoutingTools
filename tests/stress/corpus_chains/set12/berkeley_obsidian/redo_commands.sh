#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_planes.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set12/berkeley_obsidian.kicad_pcb step1_planes.kicad_pcb --nets GND GND +3V3 --plane-layers In1.Cu In6.Cu In4.Cu --clearance-ceiling 0.127 --hole-to-hole-clearance 0.2 --via-size 0.4 --via-drill 0.2 --track-width 0.127
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --nets '*' '!GND' '!+3V3' --diff-pairs 'AD0_*' 'AD1_*' 'AD8_*' 'AD9_*' 'MGT_*' 'REF_CLK0_*' 'WR_CLK0_*' --diff-pair-gap 0.127 --clearance 0.127 --track-width 0.09 --via-size 0.4 --via-drill 0.2 --plane-net-layers GND:In1.Cu,In6.Cu +3V3:In4.Cu --escape-method auto
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1_v2.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --nets '*' '!GND' '!+3V3' --diff-pairs 'AD0_*' 'AD1_*' 'AD8_*' 'AD9_*' 'MGT_*' 'REF_CLK0_*' 'WR_CLK0_*' --diff-pair-gap 0.127 --clearance 0.127 --track-width 0.09 --via-size 0.3 --via-drill 0.15 --plane-net-layers GND:In1.Cu,In6.Cu +3V3:In4.Cu --escape-method auto
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1_v3.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --nets '*' '!GND' '!+3V3' --diff-pairs 'AD0_*' 'AD1_*' 'AD8_*' 'AD9_*' 'MGT_*' 'REF_CLK0_*' 'WR_CLK0_*' --diff-pair-gap 0.09 --clearance 0.09 --track-width 0.1 --via-size 0.35 --via-drill 0.18 --plane-net-layers GND:In1.Cu,In6.Cu +3V3:In4.Cu --escape-method auto
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1_v4.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --nets '*' '!GND' '!+3V3' --diff-pairs 'AD0_*' 'AD1_*' 'AD8_*' 'AD9_*' 'MGT_*' 'REF_CLK0_*' 'WR_CLK0_*' --diff-pair-gap 0.09 --clearance 0.09 --track-width 0.1 --via-size 0.3 --via-drill 0.15 --plane-net-layers GND:In1.Cu,In6.Cu +3V3:In4.Cu --escape-method underpad --allow-via-in-pad
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1_v4.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --nets '*' '!GND' '!+3V3' --diff-pairs 'AD0_*' 'AD1_*' 'AD8_*' 'AD9_*' 'MGT_*' 'REF_CLK0_*' 'WR_CLK0_*' --diff-pair-gap 0.09 --clearance 0.09 --track-width 0.1 --via-size 0.3 --via-drill 0.15 --plane-net-layers GND:In1.Cu,In6.Cu +3V3:In4.Cu --escape-method underpad
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1_v5.kicad_pcb --component U1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --nets '*' '!GND' '!+3V3' --diff-pairs 'AD0_*' 'AD1_*' 'AD8_*' 'AD9_*' 'MGT_*' 'REF_CLK0_*' 'WR_CLK0_*' --diff-pair-gap 0.09 --clearance 0.09 --track-width 0.1 --via-size 0.3 --via-drill 0.15 --plane-net-layers GND:In1.Cu,In6.Cu +3V3:In4.Cu --escape-method auto
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1_v5.kicad_pcb step2b_capclear_u1.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step2b_capclear_u1.kicad_pcb --output step3_fanout_u2.kicad_pcb --component U2 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --nets '*' '!GND' '!+3V3' --diff-pairs 'DDR3_CLK_*' 'DDR3_DQS0_*' 'DDR3_DQS1_*' --diff-pair-gap 0.09 --clearance 0.09 --track-width 0.1 --via-size 0.3 --via-drill 0.15 --plane-net-layers GND:In1.Cu,In6.Cu +3V3:In4.Cu --escape-method auto
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3_fanout_u2.kicad_pcb step3b_capclear_u2.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3b_capclear_u2.kicad_pcb --output step4_fanout_u21.kicad_pcb --component U21 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step4_fanout_u21.kicad_pcb --output step5_fanout_u13.kicad_pcb --component U13 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step5_fanout_u13.kicad_pcb --output step6_fanout_u7.kicad_pcb --component U7 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.08
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step6_fanout_u7.kicad_pcb --output step7_fanout_u19.kicad_pcb --component U19 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step7_fanout_u19.kicad_pcb --output step8_fanout_u17.kicad_pcb --component U17 --nets '*' '!GND' '!+3V3' --clearance 0.09 --width 0.1
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step8_fanout_u17.kicad_pcb step9_diffpairs.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.3 --via-drill 0.15 --no-gnd-vias
# cwd=/Users/andy/Documents/KiCadRoutingTools
python3 -X utf8 py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set12/berkeley_obsidian
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step9_diffpairs.kicad_pcb step10_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --clearance-ceiling 0.09 --track-width 0.08 --hole-to-hole-clearance 0.2 --via-size 0.3 --via-drill 0.15 --grid-step 0.05 --power-nets GND +3V3 +1V0 DDRVCC VCC_IN +1V8 +12V DDRVTT +3V3_P +3V3_CLEAN +1V2 +3V3_USB XADCGND GND_CRYSTAL VCC_USB 'Net-(J*-VCC*)' 'Net-(U1F-VCCADC_0)' 'Net-(U12-VIN)' DDRVREF --power-nets-widths 0.3 0.3 0.4 0.35 0.4 0.3 0.3 0.25 0.25 0.2 0.25 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.15 --max-ripup 5 --no-bga-zone
