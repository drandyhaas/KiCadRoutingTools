#!/bin/bash
# Minimal self-contained manifest fixture for test_manifest_plan_parity.py.
# The converter only PARSES commands (never runs them), so the referenced
# boards need not exist. Exercises every flag the parity gate asserts:
# clearance/track-width/via/grid/max-ripup/hole-to-hole/no-bga-zones/
# no-gnd-vias/escape-method/diff-pair-gap/plane-layers.
set -e
# cwd=/repo
python3 -X utf8 bga_fanout.py board.kicad_pcb step1.kicad_pcb --component U1 --nets '*' '!GND' --clearance 0.09 --track-width 0.0762 --via-size 0.25 --via-drill 0.15 --grid-step 0.05 --escape-method auto
# cwd=/repo
python3 -X utf8 route_diff.py step1.kicad_pcb step2.kicad_pcb --nets /USB/D+ /USB/D- --clearance 0.10 --diff-pair-gap 0.1 --via-size 0.45 --via-drill 0.2 --grid-step 0.05 --no-gnd-vias
# cwd=/repo
python3 -X utf8 route.py step2.kicad_pcb step3.kicad_pcb --nets '*' '!GND' '!+3V3' --no-bga-zones --clearance 0.09 --track-width 0.0762 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 10 --max-iterations 1000000
# cwd=/repo
python3 -X utf8 route_planes.py step3.kicad_pcb step4.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In4.Cu --via-size 0.45 --via-drill 0.2 --track-width 0.09 --clearance 0.10 --hole-to-hole-clearance 0.2 --grid-step 0.05
# cwd=/repo
python3 -X utf8 route_disconnected_planes.py step4.kicad_pcb step5.kicad_pcb --nets GND +3V3 --clearance 0.09 --via-size 0.25 --via-drill 0.15 --track-width 0.0762 --grid-step 0.025 --hole-to-hole-clearance 0.2 --rip-blocker-nets
