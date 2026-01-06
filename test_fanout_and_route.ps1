# Test fanning out BGA chips

# Remaining FTDI nets (the DATA ones are already in the fanout_starting_point file, so we check-for-previous)
python3 bga_fanout.py fanout_starting_point.kicad_pcb --component U3 --output fanout_output1.kicad_pcb --nets "*U2A*" --primary-escape horizontal --check-for-previous --force-escape-direction

# LVDS from ADC
python3 bga_fanout.py fanout_output1.kicad_pcb --component IC1 --output fanout_output2.kicad_pcb --nets "*lvds_rx*" --diff-pairs "*lvds_rx*" --primary-escape vertical --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu

# LVDS on FPGA
python3 bga_fanout.py fanout_output2.kicad_pcb --component U3 --output fanout_output3.kicad_pcb --nets "*lvds_rx*" --diff-pairs "*lvds_rx*" --primary-escape vertical --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu

# DDR on FPGA
python3 bga_fanout.py fanout_output3.kicad_pcb --component U3 --output fanout_output4.kicad_pcb --nets "Net-(U1*DQS*)" "Net-(U1*CK*)" --diff-pairs "Net-(U1*DQS*)" "Net-(U1*CK*)" --primary-escape horizontal
python3 bga_fanout.py fanout_output4.kicad_pcb --component U3 --output fanout_output5.kicad_pcb --nets "*U1A*" "*U1B*" --check-for-previous --primary-escape horizontal

# DDR on DDR chip
python3 bga_fanout.py fanout_output5.kicad_pcb --component U1 --output fanout_output6.kicad_pcb --nets "Net-(U1*DQS*)" "Net-(U1*CK*)" --diff-pairs "Net-(U1*DQS*)" "Net-(U1*CK*)" --primary-escape horizontal
python3 bga_fanout.py fanout_output6.kicad_pcb --component U1 --output fanout_output7.kicad_pcb --nets "*U1A*" --check-for-previous --primary-escape horizontal --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu
python3 bga_fanout.py fanout_output7.kicad_pcb --component U1 --output fanout_output.kicad_pcb --nets "*U1B*" --check-for-previous --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu

# Route the FTDI tracks, letting them sneak into the BGA for now
python3 route.py fanout_output.kicad_pcb routed_output.kicad_pcb "Net-(U2A-DATA_11*)" --swappable-nets "Net-(U2A-DATA_*)" # quick test
#python3 route.py fanout_output.kicad_pcb routed_output.kicad_pcb "Net-(U2A-*)" --swappable-nets "Net-(U2A-DATA_*)" --heuristic-weight 2.5

# Check for errors
#python3 check_drc.py routed_output.kicad_pcb --nets "Net-(U2A-*)"

# Check connections
#python3 check_connected.py routed_output.kicad_pcb --nets "Net-(U2A-*)"

# Route diff pairs
python3 test_diffpair.py "*rx1_1*" --swappable-nets "*rx1_1*" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu # quick test
#python3 test_diffpair.py "*rx1_*" "*rx2_*" "*rx*clkin1*" "*rx*clkin2*" --swappable-nets "*rx1_*" "*rx2_*" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --output routed_output_diff12.pcb
#python3 test_diffpair.py "*rx3_*" "*rx4_*" "*rx*clkin3*" "*rx*clkin4*" --swappable-nets "*rx3_*" "*rx4_*" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --input routed_output_diff12.pcb

# Route RAM
python3 route.py test_diffpair.kicad_pcb test_diffpair_ram.kicad_pcb "Net-(U1*DQS*)" "Net-(U1*CK_*)" --diff-pairs "Net-(U1*DQS*)" "Net-(U1*CK_*)" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu \
    --bga-proximity-radius 1 --stub-proximity-radius 1 --length-match-group "Net-(U1*DQS*)" "Net-(U1*CK_*)"
#python3 route.py test_diffpair.kicad_pcb test_diffpair_ram.kicad_pcb "Net-(U1*)" --swappable-nets "Net-(U1*DQ*)" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu \
#    --bga-proximity-radius 1 --stub-proximity-radius 1 --length-match-group auto

# Check for errors
python3 check_drc.py test_diffpair_ram.kicad_pcb --nets "Net-(U1*)"

# Check connections
python3 check_connected.py test_diffpair_ram.kicad_pcb --nets "Net-(U1*)"

