#!/bin/bash
# Auto-recorded stress-test command manifest (issue #132).
# Replay with redo_stress_test.py.
set -e
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/uncutgem_nv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/uncutgem_nv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/uncutgem_nv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_planes.py /Users/andy/Documents/kicad_stress_test/boards_unrouted_set21/uncutgem_nv.kicad_pcb step1_planes.kicad_pcb --nets GND '/Transimpedence Amp (page 2)/FAKE_GND' --plane-layers B.Cu B.Cu --via-size 0.5 --via-drill 0.3 --track-width 0.2 --clearance-ceiling 0.2 --hole-to-hole-clearance 0.2
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/uncutgem_nv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb step2a_fanout_u1.kicad_pcb --component U1 --clearance 0.1 --grid-step 0.05 --nets '*' '!GND' '!/Transimpedence Amp (page 2)/FAKE_GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/uncutgem_nv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step1_planes.kicad_pcb -o step2a_fanout_u1.kicad_pcb --component U1 --clearance 0.1 --grid-step 0.05 --nets '*' '!GND' '!/Transimpedence Amp (page 2)/FAKE_GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/uncutgem_nv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/qfn_fanout.py step2a_fanout_u1.kicad_pcb -o step2b_fanout_u8.kicad_pcb --component U8 --clearance 0.1 --grid-step 0.05 --nets '*' '!GND' '!/Transimpedence Amp (page 2)/FAKE_GND'
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/uncutgem_nv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/place_fanout_clearance.py step2b_fanout_u8.kicad_pcb step2c_clearance.kicad_pcb --clearance 0.1
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/uncutgem_nv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/uncutgem_nv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route_diff.py step2c_clearance.kicad_pcb step3_diffpairs.kicad_pcb /Microcontroller-ESP32/D+ /Microcontroller-ESP32/D- 'Net-(U1-RF_OUT_A+)' 'Net-(U1-RF_OUT_A-)' --track-width 0.1 --diff-pair-gap 0.1 --clearance-ceiling 0.1 --hole-to-hole-clearance 0.2 --via-size 0.5 --via-drill 0.3 --no-gnd-vias
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/uncutgem_nv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py --help
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/uncutgem_nv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --net-clearances '{
  "MUXOUT": 0.15,
  "Net-(U1-CLK)": 0.15,
  "Net-(U1-CPout)": 0.15,
  "Net-(U1-DATA)": 0.15,
  "Net-(U1-LD)": 0.15,
  "Net-(U1-REFIN)": 0.15,
  "Net-(U1-RF_OUT_A+)": 0.15,
  "Net-(U1-RF_OUT_A-)": 0.15,
  "Net-(U1-RSET)": 0.15,
  "Net-(U1-TEMP)": 0.15,
  "Net-(U1-VCOM)": 0.15,
  "Net-(U1-VREF)": 0.15,
  "Net-(U1-VTUNE)": 0.15,
  "SPICE": 0.15,
  "SPILE": 0.15,
  "/Microcontroller-ESP32/D+": 0.15,
  "/Microcontroller-ESP32/D-": 0.15,
  "/Microcontroller-ESP32/DTR": 0.15,
  "/Microcontroller-ESP32/RTS": 0.15,
  "/Microcontroller-ESP32/RX0": 0.15,
  "/Microcontroller-ESP32/TX0": 0.15,
  "5V_SUPP": 0.15,
  "Net-(U8-~{RST})": 0.15,
  "Net-(U8-~{SUSPEND})": 0.15
}' --grid-step 0.05 --power-nets GND '/Transimpedence Amp (page 2)/FAKE_GND' 3V3 --power-nets-widths 0.3 0.25 0.3
# cwd=/Users/andy/Documents/kicad_stress_test/runs_set21/uncutgem_nv
python3 -X utf8 /private/tmp/wt_baseline_0809/py_router/route.py step3_diffpairs.kicad_pcb step4_route.kicad_pcb --nets '*' --clearance-ceiling 0.2 --track-width 0.2 --via-size 0.5 --via-drill 0.3 --hole-to-hole-clearance 0.2 --net-clearances net_clearances.json --grid-step 0.05 --power-nets GND '/Transimpedence Amp (page 2)/FAKE_GND' 3V3 --power-nets-widths 0.3 0.25 0.3
