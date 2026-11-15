#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/sata_sniffer.kicad_pcb step1_planes.kicad_pcb --nets GND GND GND /1V35 --plane-layers In1.Cu In3.Cu In6.Cu In4.Cu --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/sata_sniffer.kicad_pcb step1_planes.kicad_pcb --nets /GND /GND /GND /1V35 --plane-layers In1.Cu In3.Cu In6.Cu In4.Cu --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --via-size 0.45 --via-drill 0.25
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 bga_fanout.py --help
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u5.kicad_pcb --component U5 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --track-width 0.1 --clearance 0.09 --via-size 0.45 --via-drill 0.25 --nets '*' '!/GND' '!/1V35' --diff-pairs '*_P' '*_N' --diff-pair-gap 0.09
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u5.kicad_pcb step2b_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_capclear.kicad_pcb --output step3_fanout_u10.kicad_pcb --component U10 --width 0.1 --clearance 0.09 --nets '*' '!/GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3_fanout_u10.kicad_pcb --output step4_fanout_u2.kicad_pcb --component U2 --width 0.1 --clearance 0.09 --nets '*' '!/GND'
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4_fanout_u2.kicad_pcb step5_diff.kicad_pcb '*_P' '*_N' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --track-width 0.09 --diff-pair-gap 0.09 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --no-gnd-vias --max-ripup 5
# cwd=/Users/andy/Documents/KiCadRoutingTools/py_router
python3 -X utf8 route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diff.kicad_pcb step6_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone --power-nets /GND /1V35 /3V3 /1V8 /1V0 /5V0 /1V2 /12V0 '/SERDES interfaces/SFP_VDD_RX' '/SERDES interfaces/SFP_VDD_TX' '/Power Supply/VDDQ_SENSE' --power-nets-widths 0.3 0.3 0.3 0.25 0.35 0.3 0.25 0.4 0.2 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diff.kicad_pcb step6_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone --power-nets /GND /1V35 /3V3 /1V8 /1V0 /5V0 /1V2 /12V0 '/SERDES interfaces/SFP_VDD_RX' '/SERDES interfaces/SFP_VDD_TX' '/Power Supply/VDDQ_SENSE' --power-nets-widths 0.3 0.3 0.3 0.25 0.35 0.3 0.25 0.4 0.2 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diff.kicad_pcb step6_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone --power-nets /GND /1V35 /3V3 /1V8 /1V0 /5V0 /1V2 /12V0 '/SERDES interfaces/SFP_VDD_RX' '/SERDES interfaces/SFP_VDD_TX' '/Power Supply/VDDQ_SENSE' --power-nets-widths 0.3 0.3 0.3 0.25 0.35 0.3 0.25 0.4 0.2 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diff.kicad_pcb step6_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone --power-nets /GND /1V35 /3V3 /1V8 /1V0 /5V0 /1V2 /12V0 '/SERDES interfaces/SFP_VDD_RX' '/SERDES interfaces/SFP_VDD_TX' '/Power Supply/VDDQ_SENSE' --power-nets-widths 0.3 0.3 0.3 0.25 0.35 0.3 0.25 0.4 0.2 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diff.kicad_pcb step6_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone --power-nets /GND /1V35 /3V3 /1V8 /1V0 /5V0 /1V2 /12V0 '/SERDES interfaces/SFP_VDD_RX' '/SERDES interfaces/SFP_VDD_TX' '/Power Supply/VDDQ_SENSE' --power-nets-widths 0.3 0.3 0.3 0.25 0.35 0.3 0.25 0.4 0.2 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diff.kicad_pcb step6_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone --power-nets /GND /1V35 /3V3 /1V8 /1V0 /5V0 /1V2 /12V0 '/SERDES interfaces/SFP_VDD_RX' '/SERDES interfaces/SFP_VDD_TX' '/Power Supply/VDDQ_SENSE' --power-nets-widths 0.3 0.3 0.3 0.25 0.35 0.3 0.25 0.4 0.2 0.2 0.15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/sata_sniffer
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_diff.kicad_pcb step6_route.kicad_pcb --nets '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu In5.Cu In6.Cu B.Cu --track-width 0.1 --clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.25 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone --power-nets /GND /1V35 /3V3 /1V8 /1V0 /5V0 /1V2 /12V0 '/SERDES interfaces/SFP_VDD_RX' '/SERDES interfaces/SFP_VDD_TX' '/Power Supply/VDDQ_SENSE' --power-nets-widths 0.3 0.3 0.3 0.25 0.35 0.3 0.25 0.4 0.2 0.2 0.15
