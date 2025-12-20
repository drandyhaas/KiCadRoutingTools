alias python=python3

# Test fanning out BGA chips
# Run in Windows powershell like: .\apply_fanouts.ps1

# Remaining FTDI nets (the DATA ones are already in the fanout_starting_point file, so we check-for-previous)
python bga_fanout.py fanout_starting_point.kicad_pcb --component U3 --output fanout_output.kicad_pcb --nets "*U2A*" --primary-escape horizontal --check-for-previous --force-escape-direction

# LVDS from ADC
python bga_fanout.py fanout_output.kicad_pcb --component IC1 --output fanout_output.kicad_pcb --nets "*lvds_rx*" --diff-pairs "*lvds_rx*" --primary-escape vertical

# LVDS on FPGA
python bga_fanout.py fanout_output.kicad_pcb --component U3 --output fanout_output.kicad_pcb --nets "*lvds_rx*" --diff-pairs "*lvds_rx*" --primary-escape vertical

# DDR on FPGA
python bga_fanout.py fanout_output.kicad_pcb --component U3 --output fanout_output.kicad_pcb --nets "*U1A*" --primary-escape horizontal

# DDR on DDR chip
python bga_fanout.py fanout_output.kicad_pcb --component U1 --output fanout_output.kicad_pcb --nets "*U1A*" --primary-escape horizontal

# Route the FTDI tracks, letting them sneak into the BGA for now
python route.py fanout_output.kicad_pcb routed_output.kicad_pcb "Net-(U2A-*)" --stub-proximity-radius 1.0 #--no-bga-zone #--max-iterations 500000

# Check for errors
python check_drc.py routed_output.kicad_pcb

# Check connections
python check_connected.py routed_output.kicad_pcb --nets "Net-(U2A-*)"

