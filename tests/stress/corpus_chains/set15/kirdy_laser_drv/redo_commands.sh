#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set15/kirdy_laser_drv.kicad_pcb step1_planes.kicad_pcb --nets GND +3V3 --plane-layers In1.Cu In2.Cu --via-size 0.45 --via-drill 0.2 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_planes.kicad_pcb --output step2a_fanout_u9.kicad_pcb --component U9 --clearance 0.09 --width 0.15 --via-size 0.25 --via-drill 0.15 --grid-step 0.05 --nets '*' '!GND' '!+3V3' --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2a_fanout_u9.kicad_pcb --output step2b_fanout_u23.kicad_pcb --component U23 --clearance 0.09 --width 0.15 --via-size 0.25 --via-drill 0.15 --grid-step 0.05 --nets '*' '!GND' '!+3V3' --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2b_fanout_u23.kicad_pcb --output step2c_fanout_u28.kicad_pcb --component U28 --clearance 0.09 --width 0.15 --via-size 0.25 --via-drill 0.15 --grid-step 0.05 --nets '*' '!GND' '!+3V3' --fab-tier advanced
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py step2c_fanout_u28.kicad_pcb step2d_capclear.kicad_pcb --clearance 0.09
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step2d_capclear.kicad_pcb step3a_diff_eth.kicad_pcb /Ehternet/PHY_RD_P /Ehternet/PHY_RD_N /Ehternet/PHY_TD_P /Ehternet/PHY_TD_N /Ehternet/RJ45_RD_P /Ehternet/RJ45_RD_N /Ehternet/RJ45_TD_P /Ehternet/RJ45_TD_N --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --impedance 100 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3a_diff_eth.kicad_pcb step3b_diff_usb.kicad_pcb /MCU/USB_DP /MCU/USB_DN --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --impedance 90 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step3b_diff_usb.kicad_pcb step3c_diff_analog.kicad_pcb /MCU/PWM_MAXIP /MCU/PWM_MAXIN /thermostat/MAXIP /thermostat/MAXIN 'Net-(U16-C+)' 'Net-(U16-C-)' 'Net-(U23-AIN1{slash}REF2+)' 'Net-(U23-AIN0{slash}REF2-)' --layers F.Cu In1.Cu In2.Cu B.Cu --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.09 --hole-to-hole-clearance 0.2 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3c_diff_analog.kicad_pcb step4_route.kicad_pcb '*' --clearance-ceiling 0.09 --track-width 0.25 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 --power-nets GND +3V3 12Vin '/digital supply/+12V_Barrel_Jack' +12V +15V +9V +9VA +5V +5VA +8V -6V -9V 'Net-(U26-VIN)' +3.3VA /Ehternet/AVDDT_PHY /MCU/VDDA /MCU/USB_VBUS 'Net-(U29-VBUS)' 'Net-(U28-VDDCR)' /thermostat/TEC+ /thermostat/TEC- /Ehternet/POE_VC1+ /Ehternet/POE_VC1- /Ehternet/POE_VC2+ /Ehternet/POE_VC2- --power-nets-widths 0.25 0.25 0.6 0.6 0.5 0.5 0.5 0.4 0.4 0.4 0.4 0.4 0.4 0.5 0.35 0.3 0.3 0.3 0.3 0.3 0.8 0.8 0.35 0.35 0.35 0.35
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set15/kirdy_laser_drv
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step3c_diff_analog.kicad_pcb step4_route.kicad_pcb '*' --clearance-ceiling 0.09 --track-width 0.25 --via-size 0.45 --via-drill 0.2 --hole-to-hole-clearance 0.2 --max-ripup 5 --power-nets GND +3V3 12Vin '/digital supply/+12V_Barrel_Jack' +12V +15V +9V +9VA +5V +5VA +8V -6V -9V 'Net-(U26-VIN)' +3.3VA /Ehternet/AVDDT_PHY /MCU/VDDA /MCU/USB_VBUS 'Net-(U29-VBUS)' 'Net-(U28-VDDCR)' /thermostat/TEC+ /thermostat/TEC- /Ehternet/POE_VC1+ /Ehternet/POE_VC1- /Ehternet/POE_VC2+ /Ehternet/POE_VC2- --power-nets-widths 0.25 0.25 0.6 0.6 0.5 0.5 0.5 0.4 0.4 0.4 0.4 0.4 0.4 0.5 0.35 0.3 0.3 0.3 0.3 0.3 0.8 0.8 0.35 0.35 0.35 0.35
