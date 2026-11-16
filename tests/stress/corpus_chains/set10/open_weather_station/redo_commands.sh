#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/bga_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set10/open_weather_station.kicad_pcb -c U1 -o step1_fanout_U1.kicad_pcb --clearance 0.1 --grid-step 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step1_fanout_U1.kicad_pcb -c U8 -o step2_fanout_U8.kicad_pcb --clearance 0.1 --grid-step 0.1 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step2_fanout_U8.kicad_pcb -c U2 -o step3_fanout_U2.kicad_pcb --clearance 0.1 --grid-step 0.1 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/qfn_fanout.py step3_fanout_U2.kicad_pcb -c U5 -o step4_fanout_U5.kicad_pcb --clearance 0.1 --grid-step 0.1 --nets '*' '!GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/place_fanout_clearance.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4_fanout_U5.kicad_pcb -o step5_diff.kicad_pcb --nets /Ethernet/ETH_DATA.RD+ /Ethernet/ETH_DATA.RD- /Ethernet/ETH_DATA.TD+ /Ethernet/ETH_DATA.TD- --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_diff.py step4_fanout_U5.kicad_pcb --output step5_diff.kicad_pcb --nets /Ethernet/ETH_DATA.RD+ /Ethernet/ETH_DATA.RD- /Ethernet/ETH_DATA.TD+ /Ethernet/ETH_DATA.TD- --track-width 0.2 --diff-pair-gap 0.25 --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route_planes.py step5_diff.kicad_pcb --output step6_planes.kicad_pcb --nets GND --plane-layers B.Cu --clearance-ceiling 0.2 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 /PoE/VIN GNDPWR GNDA /PoE/VSS +3.3VA +3.3V 'Net-(U8-VDDCR)' --power-nets-widths 0.5 0.4 0.4 0.4 0.3 0.4 0.3 0.25 0.25
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step6_planes.kicad_pcb --output step7_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.25 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --power-nets GND +3V3 /PoE/VIN GNDPWR GNDA /PoE/VSS +3.3VA +3.3V 'Net-(U8-VDDCR)' --power-nets-widths 0.5 0.4 0.4 0.4 0.3 0.4 0.3 0.25 0.25 --max-ripup 5 --no-bga-zone
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set10/open_weather_station
python3 -X utf8 /Users/andy/Documents/KiCadRoutingTools/py_router/route.py step7_route.kicad_pcb --output step8_retry.kicad_pcb --nets /Connectors/ETH_PORT4 /Connectors/ETH_PORT5 /Connectors/ETH_PORT7 /Connectors/ETH_PORT8 'Net-(D1-Pad3)' --clearance-ceiling 0.1 --track-width 0.15 --via-size 0.45 --via-drill 0.3 --hole-to-hole-clearance 0.2 --max-ripup 5 --no-bga-zone --grid-step 0.05
