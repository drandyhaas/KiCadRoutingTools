#!/usr/bin/env python3
"""
Test fanning out BGA chips and routing.
All inputs/outputs go in the "kicad_files" directory.
"""

import argparse
import shlex
import subprocess
import os


def run(cmd: str, unbuffered: bool = False) -> None:
    """Run a command string and exit if it fails.

    Args:
        cmd: Command string to run (will be parsed using shell-style splitting)
        unbuffered: If True, add -u flag to python commands
    """
    if unbuffered and cmd.startswith('python3 '):
        cmd = 'python3 -u ' + cmd[8:]
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
    parser.add_argument('-u', '--unbuffered', action='store_true',
                        help='Run python commands with -u (unbuffered output)')
    args = parser.parse_args()

    quick = args.quick
    ram_only = args.ram_only
    unbuffered = args.unbuffered

    if not ram_only:
        # Fan out QFN
        run('python3 qfn_fanout.py kicad_files/haasoscope_pro_max_test.kicad_pcb --output kicad_files/qfn_fanned_out.kicad_pcb --component U2 --nets "Net-(U2*)"', unbuffered)

        # Fan out FTDI nets, first DATA nets, then others (to test "--check-for-previous")
        run('python3 bga_fanout.py kicad_files/qfn_fanned_out.kicad_pcb --component U3 --output kicad_files/fanout_starting_point.kicad_pcb --nets "*U2A*DATA*" --primary-escape horizontal --force-escape-direction', unbuffered)
        run('python3 bga_fanout.py kicad_files/fanout_starting_point.kicad_pcb --component U3 --output kicad_files/fanout_output1.kicad_pcb --nets "*U2A*" --primary-escape horizontal --check-for-previous --force-escape-direction', unbuffered)

        # Fan out LVDS from ADC
        run('python3 bga_fanout.py kicad_files/fanout_output1.kicad_pcb --component IC1 --output kicad_files/fanout_output2.kicad_pcb --nets "*lvds_rx*" --diff-pairs "*lvds_rx*" --primary-escape vertical --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --no-inner-top-layer', unbuffered)

        # Fan out LVDS on FPGA
        run('python3 bga_fanout.py kicad_files/fanout_output2.kicad_pcb --component U3 --output kicad_files/fanout_output3.kicad_pcb --nets "*lvds_rx*" --diff-pairs "*lvds_rx*" --primary-escape vertical --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --no-inner-top-layer', unbuffered)

        # Fan out DDR on FPGA
        run('python3 bga_fanout.py kicad_files/fanout_output3.kicad_pcb --component U3 --output kicad_files/fanout_output4.kicad_pcb --nets "Net-(U1*DQS*)" "Net-(U1*CK*)" --diff-pairs "Net-(U1*DQS*)" "Net-(U1*CK*)" --primary-escape horizontal', unbuffered)
        run('python3 bga_fanout.py kicad_files/fanout_output4.kicad_pcb --component U3 --output kicad_files/fanout_output5.kicad_pcb --nets "*U1A*" "*U1B*" --check-for-previous --primary-escape horizontal', unbuffered)

        # Fan out DDR on DDR chip
        run('python3 bga_fanout.py kicad_files/fanout_output5.kicad_pcb --component U1 --output kicad_files/fanout_output6.kicad_pcb --nets "Net-(U1*DQS*)" "Net-(U1*CK*)" --diff-pairs "Net-(U1*DQS*)" "Net-(U1*CK*)" --primary-escape horizontal --no-inner-top-layer', unbuffered)
        run('python3 bga_fanout.py kicad_files/fanout_output6.kicad_pcb --component U1 --output kicad_files/fanout_output7.kicad_pcb --nets "*U1A*" --check-for-previous --primary-escape horizontal --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --no-inner-top-layer', unbuffered)
        run('python3 bga_fanout.py kicad_files/fanout_output7.kicad_pcb --component U1 --output kicad_files/fanout_output.kicad_pcb --nets "*U1B*" --check-for-previous --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --no-inner-top-layer', unbuffered)

        # Route the FTDI tracks
        if quick: run('python3 route.py kicad_files/fanout_output.kicad_pcb kicad_files/routed_output.kicad_pcb --nets "Net-(U2A-DATA_11*)" --swappable-nets "Net-(U2A-DATA_*)"', unbuffered) # quick test
        else: run('python3 route.py kicad_files/fanout_output.kicad_pcb kicad_files/routed_output.kicad_pcb --nets "Net-(U2A-*)" --swappable-nets "Net-(U2A-DATA_*)" --mps-layer-swap', unbuffered)

        # Check for errors
        run('python3 check_drc.py kicad_files/routed_output.kicad_pcb --nets "Net-(U2A-*)"', unbuffered)

        # Check connections
        if not quick: run('python3 check_connected.py kicad_files/routed_output.kicad_pcb --nets "Net-(U2A-*)"', unbuffered)

        # Route LVDS diff pairs
        if quick:
            # Quick test: route just a few rx1_1* pairs
            run('python3 route_diff.py kicad_files/routed_output.kicad_pcb kicad_files/test_diffpair.kicad_pcb --nets "*lvds_rx1_1*" --swappable-nets "*lvds_rx1_1*" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu', unbuffered)
        else:
            # Full test: route all 56 LVDS pairs in two batches - those on bottom and then those on top
            run('python3 route_diff.py kicad_files/routed_output.kicad_pcb kicad_files/routed_output_diff12.kicad_pcb --nets "*lvds_rx1_*" "*lvds_rx2_*" "*lvds_rx*clkin1*" "*lvds_rx*clkin2*" --swappable-nets "*lvds_rx1_*" "*lvds_rx2_*" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu', unbuffered)
            run('python3 route_diff.py kicad_files/routed_output_diff12.kicad_pcb kicad_files/test_diffpair.kicad_pcb --nets "*lvds_rx3_*" "*lvds_rx4_*" "*lvds_rx*clkin3*" "*lvds_rx*clkin4*" --swappable-nets "*lvds_rx3_*" "*lvds_rx4_*" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu', unbuffered)

        # Check LVDS routing for errors and connectivity
        run('python3 check_drc.py kicad_files/test_diffpair.kicad_pcb --nets "*lvds*"', unbuffered)
        if not quick: run('python3 check_connected.py kicad_files/test_diffpair.kicad_pcb --nets "*lvds*"', unbuffered)

    # Route RAM
    run('python3 route_diff.py kicad_files/test_diffpair.kicad_pcb kicad_files/test_diffpair_ramdiff.kicad_pcb --nets "Net-(U1*DQS*)" "Net-(U1*CK_*)" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --bga-proximity-radius 1 --stub-proximity-radius 1 --length-match-group "Net-(U1*DQS*)" "Net-(U1*CK_*)" --mps-layer-swap --diff-pair-intra-match --heuristic-weight 1.5', unbuffered)
    if not quick: run('python3 route.py kicad_files/test_diffpair_ramdiff.kicad_pcb kicad_files/test_diffpair_ram.kicad_pcb --nets "Net-(U1*)" --swappable-nets "Net-(U1*DQ*)" --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu --bga-proximity-radius 1 --stub-proximity-radius 1 --length-match-group auto --max-iterations 1000000 --no-bga-zones U1', unbuffered)
    else: os.replace("kicad_files/test_diffpair_ramdiff.kicad_pcb", "kicad_files/test_diffpair_ram.kicad_pcb")

    # Check for errors
    run('python3 check_drc.py kicad_files/test_diffpair_ram.kicad_pcb --nets "Net-(U1*)"', unbuffered)

    # Check connections
    if not quick: run('python3 check_connected.py kicad_files/test_diffpair_ram.kicad_pcb --nets "Net-(U1*)"', unbuffered)

    print("\n=== All tests completed ===")


if __name__ == "__main__":
    main()
