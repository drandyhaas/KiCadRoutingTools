#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set25/bms_sensor.kicad_pcb --component U106 --nets '*' '!GND' --clearance 0.1 --width 0.15 --grid-step 0.05 --output step1_fanout_u106.kicad_pcb
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step1_fanout_u106.kicad_pcb step2_capclear.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py step2_capclear.kicad_pcb step3_planes.kicad_pcb --nets GND /BMS.Gnd --plane-layers B.Cu B.Cu --clearance-ceiling 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND /BMS.Gnd --power-nets-widths 0.3 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_planes.kicad_pcb step4_route.kicad_pcb --nets '*' --power-nets GND /BMS.Gnd V_+5V /BMS.+5V --power-nets-widths 0.4 0.4 0.4 0.3 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 5 --no-bga-zone --net-clearances '{"/SPI.Miso":0.1,"/SPI.Clock":0.1,"V_+5V":0.1,"/CAN.Select":0.1,"/Analog.Reference":0.1,"/CAN.Interrupt":0.1,"Net-(U106-PA7)":0.1,"Net-(U105-OSC1)":0.1,"Net-(U106-PB4)":0.1,"/CAN.RX_Buf_0":0.1,"/CAN.RX_Buf_1":0.1,"/Analog.Input_10":0.1,"/Analog.Input_11":0.1,"/Balance.Out":0.1,"/Enable.Out":0.1,"/LED.Out":0.1,"/CAN.Start_Of_Frame":0.1,"/Debug.UPDI":0.1,"/SPI.Mosi":0.1}'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_planes.kicad_pcb step4_route.kicad_pcb --nets '*' --power-nets GND /BMS.Gnd V_+5V /BMS.+5V --power-nets-widths 0.4 0.4 0.4 0.3 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 5 --no-bga-zone --net-clearances net_clearances.json
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step5_retry.kicad_pcb --nets /CAN.Interrupt /SPI.Clock /SPI.Miso --rip-existing-nets '*' --power-nets GND /BMS.Gnd V_+5V /BMS.+5V --power-nets-widths 0.4 0.4 0.4 0.3 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.025 --max-ripup 5 --no-bga-zone --net-clearances net_clearances.json
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step5_retry.kicad_pcb --nets /CAN.Interrupt /SPI.Clock /SPI.Miso --rip-existing-nets '*' --power-nets GND /BMS.Gnd V_+5V /BMS.+5V --power-nets-widths 0.4 0.4 0.4 0.3 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.025 --max-ripup 5 --no-bga-zone --net-clearances net_clearances.json
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step5_retry.kicad_pcb --nets /CAN.Interrupt /SPI.Clock /SPI.Miso --rip-existing-nets '*' --power-nets GND /BMS.Gnd V_+5V /BMS.+5V --power-nets-widths 0.4 0.4 0.4 0.3 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.025 --max-ripup 5 --no-bga-zone --net-clearances net_clearances.json
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step5_retry.kicad_pcb --nets /CAN.Interrupt /SPI.Clock /SPI.Miso --rip-existing-nets '*' --power-nets GND /BMS.Gnd V_+5V /BMS.+5V --power-nets-widths 0.4 0.4 0.4 0.3 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.025 --max-ripup 5 --no-bga-zone --net-clearances net_clearances.json
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step5_retry.kicad_pcb --nets /CAN.Interrupt /SPI.Clock /SPI.Miso --rip-existing-nets '*' --power-nets GND /BMS.Gnd V_+5V /BMS.+5V --power-nets-widths 0.4 0.4 0.4 0.3 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.025 --max-ripup 5 --no-bga-zone --net-clearances net_clearances.json
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step4_route.kicad_pcb step5_retry.kicad_pcb --nets /CAN.Interrupt /SPI.Clock /SPI.Miso --rip-existing-nets '*' --power-nets GND /BMS.Gnd V_+5V /BMS.+5V --power-nets-widths 0.4 0.4 0.4 0.3 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.025 --max-ripup 5 --no-bga-zone --net-clearances net_clearances.json
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step5_retry.kicad_pcb step6_fix.kicad_pcb --nets /BMS.Gnd 'Net-(U101-EN)' V_+5V /BMS.Power_L /BMS.Can_H --power-nets GND /BMS.Gnd V_+5V /BMS.+5V --power-nets-widths 0.4 0.4 0.4 0.3 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 5 --no-bga-zone --net-clearances net_clearances.json
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step6_fix.kicad_pcb step7_gndfix.kicad_pcb --nets GND --power-nets GND /BMS.Gnd V_+5V /BMS.+5V --power-nets-widths 0.4 0.4 0.4 0.3 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.05 --max-ripup 5 --no-bga-zone --net-clearances net_clearances.json
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set25/bms_sensor
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step7_gndfix.kicad_pcb step8_final.kicad_pcb --nets 'Net-(U106-PA7)' --power-nets GND /BMS.Gnd V_+5V /BMS.+5V --power-nets-widths 0.4 0.4 0.4 0.3 --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --grid-step 0.025 --max-ripup 5 --no-bga-zone --net-clearances net_clearances.json
