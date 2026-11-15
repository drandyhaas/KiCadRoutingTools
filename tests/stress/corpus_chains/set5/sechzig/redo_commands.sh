#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set5/sechzig.kicad_pcb step1_planes.kicad_pcb --nets GND GND --plane-layers F.Cu In1.Cu --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --via-size 0.45 --via-drill 0.2 --track-width 0.125 --clearance-ceiling 0.125 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --stitch-vias --stitch-pitch 15
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1.kicad_pcb --component U1 --nets '*' '!GND' --diff-pairs 'DDMI_CK_*' 'DDMI_D0_*' 'DDMI_D1_*' 'DDMI_D2_*' 'DRAM_CK_*' 'DRAM_LDQS_*' 'DRAM_UDQS_*' 'DS0_*' 'DS1_*' 'DS2_*' 'USBD_*' 'USBH0_*' 'USBH1_*' --diff-pair-gap 0.1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 -1 1.0 1.0 1.0 1.0 --track-width 0.125 --clearance 0.1 --via-size 0.4 --via-drill 0.2 --plane-net-layers GND:F.Cu,In1.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1_v2.kicad_pcb --component U1 --nets '*' '!GND' --diff-pairs 'DDMI_CK_*' 'DDMI_D0_*' 'DDMI_D1_*' 'DDMI_D2_*' 'DRAM_CK_*' 'DRAM_LDQS_*' 'DRAM_UDQS_*' 'DS0_*' 'DS1_*' 'DS2_*' 'USBD_*' 'USBH0_*' 'USBH1_*' --diff-pair-gap 0.1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 -1 1.0 1.0 1.0 1.0 --track-width 0.125 --clearance 0.1 --via-size 0.35 --via-drill 0.2 --plane-net-layers GND:F.Cu,In1.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1_v3.kicad_pcb --component U1 --nets '*' '!GND' --diff-pairs 'DDMI_CK_*' 'DDMI_D0_*' 'DDMI_D1_*' 'DDMI_D2_*' 'DRAM_CK_*' 'DRAM_LDQS_*' 'DRAM_UDQS_*' 'DS0_*' 'DS1_*' 'DS2_*' 'USBD_*' 'USBH0_*' 'USBH1_*' --diff-pair-gap 0.1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 -1 1.0 1.0 1.0 1.0 --track-width 0.1 --clearance 0.1 --via-size 0.35 --via-drill 0.2 --plane-net-layers GND:F.Cu,In1.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1_v2.kicad_pcb step2c_clearance_u1.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step2c_clearance_u1.kicad_pcb --output step3_fanout_u3.kicad_pcb --component U3 --nets '*' '!GND' --diff-pairs 'DRAM_CK_*' 'DRAM_LDQS_*' 'DRAM_UDQS_*' --diff-pair-gap 0.1 --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 -1 1.0 1.0 1.0 1.0 --track-width 0.125 --clearance 0.1 --via-size 0.35 --via-drill 0.2 --plane-net-layers GND:F.Cu,In1.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3_fanout_u3.kicad_pcb step3c_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3c_clearance.kicad_pcb step4_diffpairs.kicad_pcb --nets 'DDMI_CK_*' 'DDMI_D0_*' 'DDMI_D1_*' 'DDMI_D2_*' 'DDMIC_CK_*' 'DDMIC_D0_*' 'DDMIC_D1_*' 'DDMIC_D2_*' 'DRAM_CK_*' 'DRAM_LDQS_*' 'DRAM_UDQS_*' 'DS0_*' 'DS1_*' 'DS2_*' 'USBD_*' 'USBDC_*' 'USBH0_*' 'USBH1_*' 'USBHC0_*' 'USBHC1_*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 -1 1.0 1.0 1.0 1.0 --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.4 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step1_planes.kicad_pcb --output step2_fanout_u1_v4.kicad_pcb --component U1 --nets '*' '!GND' --diff-pairs 'DDMI_CK_*' 'DDMI_D0_*' 'DDMI_D1_*' 'DDMI_D2_*' 'DRAM_CK_*' 'DRAM_LDQS_*' 'DRAM_UDQS_*' 'DS0_*' 'DS1_*' 'DS2_*' 'USBD_*' 'USBH0_*' 'USBH1_*' --diff-pair-gap 0.15 --rebalance-escape --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 -1 1.0 1.0 1.0 1.0 --track-width 0.125 --clearance 0.1 --via-size 0.35 --via-drill 0.2 --plane-net-layers GND:F.Cu,In1.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2_fanout_u1_v4.kicad_pcb step2c_clearance_u1_v4.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py step2c_clearance_u1_v4.kicad_pcb --output step3_fanout_u3_v2.kicad_pcb --component U3 --nets '*' '!GND' --diff-pairs 'DRAM_CK_*' 'DRAM_LDQS_*' 'DRAM_UDQS_*' --diff-pair-gap 0.15 --rebalance-escape --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 -1 1.0 1.0 1.0 1.0 --track-width 0.125 --clearance 0.1 --via-size 0.35 --via-drill 0.2 --plane-net-layers GND:F.Cu,In1.Cu
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step3_fanout_u3_v2.kicad_pcb step3c_clearance_v2.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3c_clearance_v2.kicad_pcb step4_diffpairs_v2.kicad_pcb --nets 'DDMI_CK_*' 'DDMI_D0_*' 'DDMI_D1_*' 'DDMI_D2_*' 'DDMIC_CK_*' 'DDMIC_D0_*' 'DDMIC_D1_*' 'DDMIC_D2_*' 'DRAM_CK_*' 'DRAM_LDQS_*' 'DRAM_UDQS_*' 'DS0_*' 'DS1_*' 'DS2_*' 'USBD_*' 'USBDC_*' 'USBH0_*' 'USBH1_*' 'USBHC0_*' 'USBHC1_*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 -1 1.0 1.0 1.0 1.0 --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --via-size 0.4 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step4_diffpairs_v2.kicad_pcb step5_route.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 6.0 1.0 1.0 1.0 1.0 --track-width 0.125 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --power-nets GND /PWR3V3 /PWR1V1 /DRAM_VDD /DRAM_VDDQ /PWR1V35 /PWR2V5 /PWR5V0 --power-nets-widths 0.5 0.5 0.4 0.4 0.4 0.4 0.3 0.3 --length-match-group auto --length-match-tolerance 0.5
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set5/sechzig
python3 -u -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step5_route.kicad_pcb step6_retry.kicad_pcb '*' --layers F.Cu In1.Cu In2.Cu In3.Cu In4.Cu B.Cu --layer-costs 1.0 6.0 1.0 1.0 1.0 1.0 --track-width 0.125 --clearance-ceiling 0.125 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.25 --board-edge-clearance 0.2 --power-nets GND /PWR3V3 /PWR1V1 /DRAM_VDD /DRAM_VDDQ /PWR1V35 /PWR2V5 /PWR5V0 --power-nets-widths 0.5 0.5 0.4 0.4 0.4 0.4 0.3 0.3 --grid-step 0.05 --max-ripup 5
