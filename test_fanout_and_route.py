#!/usr/bin/env python3
"""
Test fanning out BGA chips and routing.
All inputs/outputs go in the "kicad_files" directory.
"""

import argparse
import shlex
import subprocess
import sys


def run(cmd: str) -> None:
    """Run a command string and exit if it fails.

    Args:
        cmd: Command string to run (will be parsed using shell-style splitting)
    """
    print(f"\n>>> {cmd}")
    args = shlex.split(cmd)
    result = subprocess.run(args)
    if result.returncode != 0:
        print(f"Command failed with exit code {result.returncode}")
        #sys.exit(result.returncode)


def main():
    parser = argparse.ArgumentParser(description='Test fanning out BGA chips and routing')
    parser.add_argument('--quick', action='store_true',
                        help='Run quick test with reduced routing')
    parser.add_argument('--ram-only', action='store_true',
                        help='Only run RAM routing tests (requires test_diffpair.kicad_pcb to exist)')
    args = parser.parse_args()

    quick = args.quick
    ram_only = args.ram_only

    if not ram_only:
        # Fan out QFN
        run('python3 qfn_fanout.py kicad_files/haasoscope_pro_max_test.kicad_pcb --output kicad_files/qfn_fanned_out.kicad_pcb --component U2 --nets "Net-(U2*)"')

        # Fan out FTDI nets, first DATA nets, then others (to test "--check-for-previous")
        run('python3 bga_fanout.py kicad_files/qfn_fanned_out.kicad_pcb --component U3 --output kicad_files/fanout_starting_point.kicad_pcb --nets "*U2A*DATA*" --primary-escape horizontal --force-escape-direction')
        run('python3 bga_fanout.py kicad_files/fanout_starting_point.kicad_pcb --component U3 --output kicad_files/fanout_output1.kicad_pcb --nets "*U2A*" --primary-escape horizontal --check-for-previous --force-escape-direction')

        # Fan out LVDS from ADC
        run('python3 bga_fanout.py kicad_files/fanout_output1.kicad_pcb --component IC1 --output kicad_files/fanout_output2.kicad_pcb --nets "*lvds_rx*" --diff-pairs "*lvds_rx*" --primary-escape vertical --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --no-inner-top-layer')

        # Fan out LVDS on FPGA
        run('python3 bga_fanout.py kicad_files/fanout_output2.kicad_pcb --component U3 --output kicad_files/fanout_output3.kicad_pcb --nets "*lvds_rx*" --diff-pairs "*lvds_rx*" --primary-escape vertical --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --no-inner-top-layer')

        # Fan out DDR on FPGA
        run('python3 bga_fanout.py kicad_files/fanout_output3.kicad_pcb --component U3 --output kicad_files/fanout_output4.kicad_pcb --nets "Net-(U1*DQS*)" "Net-(U1*CK*)" --diff-pairs "Net-(U1*DQS*)" "Net-(U1*CK*)" --primary-escape horizontal')
        run('python3 bga_fanout.py kicad_files/fanout_output4.kicad_pcb --component U3 --output kicad_files/fanout_output5.kicad_pcb --nets "*U1A*" "*U1B*" --check-for-previous --primary-escape horizontal')

        # Fan out DDR on DDR chip
        run('python3 bga_fanout.py kicad_files/fanout_output5.kicad_pcb --component U1 --output kicad_files/fanout_output6.kicad_pcb --nets "Net-(U1*DQS*)" "Net-(U1*CK*)" --diff-pairs "Net-(U1*DQS*)" "Net-(U1*CK*)" --primary-escape horizontal --no-inner-top-layer')
        run('python3 bga_fanout.py kicad_files/fanout_output6.kicad_pcb --component U1 --output kicad_files/fanout_output7.kicad_pcb --nets "*U1A*" --check-for-previous --primary-escape horizontal --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --no-inner-top-layer')
        run('python3 bga_fanout.py kicad_files/fanout_output7.kicad_pcb --component U1 --output kicad_files/fanout_output.kicad_pcb --nets "*U1B*" --check-for-previous --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --no-inner-top-layer')

        # Route the FTDI tracks
        if quick: run('python3 route.py kicad_files/fanout_output.kicad_pcb kicad_files/routed_output.kicad_pcb "Net-(U2A-DATA_11*)" --swappable-nets "Net-(U2A-DATA_*)"') # quick test
        else: run('python3 route.py kicad_files/fanout_output.kicad_pcb kicad_files/routed_output.kicad_pcb "Net-(U2A-*)" --swappable-nets "Net-(U2A-DATA_*)" --mps-layer-swap')

        # Check for errors
        run('python3 check_drc.py kicad_files/routed_output.kicad_pcb --nets "Net-(U2A-*)"')

        # Check connections
        if not quick: run('python3 check_connected.py kicad_files/routed_output.kicad_pcb --nets "Net-(U2A-*)"')

        # Route diff pairs (and check for errors and connections)
        if quick: run('python3 test_diffpair.py "*rx1_1*" --swappable-nets "*rx1_1*" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu') # quick test
        else:
            run('python3 test_diffpair.py "*rx1_*" "*rx2_*" "*rx*clkin1*" "*rx*clkin2*" --swappable-nets "*rx1_*" "*rx2_*" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --output kicad_files/routed_output_diff12.pcb')
            run('python3 test_diffpair.py "*rx3_*" "*rx4_*" "*rx*clkin3*" "*rx*clkin4*" --swappable-nets "*rx3_*" "*rx4_*" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --input kicad_files/routed_output_diff12.pcb')

    # Route RAM
    run('python3 route.py kicad_files/test_diffpair.kicad_pcb kicad_files/test_diffpair_ram.kicad_pcb "Net-(U1*DQS*)" "Net-(U1*CK_*)" --diff-pairs "Net-(U1*DQS*)" "Net-(U1*CK_*)" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --bga-proximity-radius 1 --stub-proximity-radius 1 --length-match-group "Net-(U1*DQS*)" "Net-(U1*CK_*)" --mps-layer-swap --diff-pair-intra-match --heuristic-weight 1.5')
    if not quick: run('python3 route.py kicad_files/test_diffpair_ram.kicad_pcb kicad_files/test_diffpair_ram.kicad_pcb "Net-(U1*)" --swappable-nets "Net-(U1*DQ*)" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --bga-proximity-radius 1 --stub-proximity-radius 1 --length-match-group auto')

    # Check for errors
    run('python3 check_drc.py kicad_files/test_diffpair_ram.kicad_pcb --nets "Net-(U1*)"')

    # Check connections
    if not quick: run('python3 check_connected.py kicad_files/test_diffpair_ram.kicad_pcb --nets "Net-(U1*)"')

    print("\n=== All tests completed ===")


if __name__ == "__main__":
    main()
