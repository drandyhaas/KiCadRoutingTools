# Test fanning out BGA chips

# Remaining FTDI nets (the DATA ones are already in the fanout_starting_point file, so we check-for-previous)
python3 bga_fanout.py fanout_starting_point.kicad_pcb --component U3 --output fanout_output1.kicad_pcb --nets "*U2A*" --primary-escape horizontal --check-for-previous --force-escape-direction

# LVDS from ADC
python3 bga_fanout.py fanout_output1.kicad_pcb --component IC1 --output fanout_output2.kicad_pcb --nets "*lvds_rx*" --diff-pairs "*lvds_rx*" --primary-escape vertical --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu

# LVDS on FPGA
python3 bga_fanout.py fanout_output2.kicad_pcb --component U3 --output fanout_output3.kicad_pcb --nets "*lvds_rx*" --diff-pairs "*lvds_rx*" --primary-escape vertical --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu

# DDR on FPGA
python3 bga_fanout.py fanout_output3.kicad_pcb --component U3 --output fanout_output4.kicad_pcb --nets "*U1A*" --primary-escape horizontal

# DDR on DDR chip
python3 bga_fanout.py fanout_output4.kicad_pcb --component U1 --output fanout_output.kicad_pcb --nets "*U1A*" --primary-escape horizontal

# Route the FTDI tracks, letting them sneak into the BGA for now
python3 route.py fanout_output.kicad_pcb routed_output.kicad_pcb "Net-(U2A-*)" --swappable-nets "Net-(U2A-DATA_*)" #--no-bga-zone

# Check for errors
python3 check_drc.py routed_output.kicad_pcb

# Check connections
python3 check_connected.py routed_output.kicad_pcb --nets "Net-(U2A-*)"

# Test
# python3 test_all_diffpairs.py --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu

