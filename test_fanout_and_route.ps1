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
python3 route.py fanout_output.kicad_pcb routed_output.kicad_pcb "Net-(U2A-*)" --swappable-nets "Net-(U2A-DATA_*)" --heuristic-weight 2.5

# Check for errors
python3 check_drc.py routed_output.kicad_pcb

# Check connections
python3 check_connected.py routed_output.kicad_pcb --nets "Net-(U2A-*)"

# Test
# python3 test_all_diffpairs.py --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu
# python3 test_diffpair.py "*rx1_*" --swappable-nets "*rx1_*" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu

# Route diff pairs
# python3 test_diffpair.py "*rx1_*" "*rx2_*" "*rx*clkin1*" "*rx*clkin2*" --swappable-nets "*rx1_*" "*rx2_*" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --output 12.pcb
# python3 test_diffpair.py "*rx3_*" "*rx4_*" "*rx*clkin3*" "*rx*clkin4*" --swappable-nets "*rx3_*" "*rx4_*" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --input 12.pcb

# Fanout remaining RAM chip
# python3 bga_fanout.py --component U1 --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --nets --net "*U1A*" "*U1B*" --check-for-previous --output test_diffpair.py test_diffpair.kicad_pcb

# Route RAM
# python3 route.py test_diffpair.kicad_pcb test_diffpair_ram.kicad_pcb "Net-(U1A-*)" "Net-(U1B-*)" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu

# check for errors
# python3 check_drc.py test_diffpair_ram.kicad_pcb

# Check connections
# python3 check_connected.py test_diffpair_ram.kicad_pcb --nets "Net-(U1A-*)" "Net-(U1B-*)"
