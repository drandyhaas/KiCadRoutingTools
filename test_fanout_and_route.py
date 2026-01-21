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
    parser.add_argument('--quick', action='store_true', default=False,
                        help='Run quick test with reduced routing')
    parser.add_argument('--fanout', action='store_true', default=False,
                        help='Run fanout tests')
    parser.add_argument('--ftdi', action='store_true', default=False,
                        help='Run ftdi routing tests')
    parser.add_argument('--lvds', action='store_true', default=False,
                        help='Run lvds routing tests')
    parser.add_argument('--ram', action='store_true', default=False,
                        help='Run RAM routing tests')
    parser.add_argument('--planes', action='store_true', default=False,
                        help='Run planes routing tests')
    parser.add_argument('--checks', action='store_true', default=False,
                        help='Run checks of DRC and connectivity (unless --quick given)')
    parser.add_argument('--onlychecks', action='store_true', default=False,
                        help='Run only checks, no routing')
    parser.add_argument('--all', action='store_true', default=False,
                        help='Run all tests and checks')
    parser.add_argument('-u', '--unbuffered', action='store_true', default=False,
                        help='Run python commands with -u (unbuffered output)')
    args = parser.parse_args()

    quick = args.quick
    fanout = args.fanout
    ftdi = args.ftdi
    lvds = args.lvds
    ram = args.ram
    planes = args.planes
    checks = args.checks
    onlychecks = args.onlychecks
    if args.all:
        fanout=True
        ftdi=True
        lvds=True
        ram=True
        planes=True
        checks=True
    unbuffered = args.unbuffered

    # Common CLI parameter groups
    GEOMETRY = "--clearance 0.1 --via-size 0.3 --via-drill 0.2 --track-width 0.1"
    LAYERS_4 = "--layers F.Cu In1.Cu In2.Cu B.Cu"
    LAYERS_5 = "--layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu"

    if fanout and not onlychecks:
        # Fan out QFN
        run('python3 qfn_fanout.py kicad_files/haasoscope_pro_max_test.kicad_pcb --output kicad_files/qfn_fanned_out.kicad_pcb --component U2 --nets "Net-(U2*)"', unbuffered)

        # Fan out FTDI nets, first DATA nets, then others (to test "--check-for-previous")
        run('python3 bga_fanout.py kicad_files/qfn_fanned_out.kicad_pcb --component U3 --output kicad_files/fanout_starting_point.kicad_pcb --nets "*U2A*DATA*" --primary-escape horizontal --force-escape-direction', unbuffered)
        run('python3 bga_fanout.py kicad_files/fanout_starting_point.kicad_pcb --component U3 --output kicad_files/fanout_output1.kicad_pcb --nets "*U2A*" --primary-escape horizontal --check-for-previous --force-escape-direction', unbuffered)

        # Fan out LVDS from ADC
        run(f"python3 bga_fanout.py kicad_files/fanout_output1.kicad_pcb --component IC1 --output kicad_files/fanout_output2.kicad_pcb --nets '*lvds_rx*' --diff-pairs '*lvds_rx*' --primary-escape vertical {LAYERS_5} --no-inner-top-layer", unbuffered)

        # Fan out LVDS on FPGA
        run(f"python3 bga_fanout.py kicad_files/fanout_output2.kicad_pcb --component U3 --output kicad_files/fanout_output3.kicad_pcb --nets '*lvds_rx*' --diff-pairs '*lvds_rx*' --primary-escape vertical {LAYERS_5} --no-inner-top-layer", unbuffered)

        # Fan out DDR on FPGA
        run('python3 bga_fanout.py kicad_files/fanout_output3.kicad_pcb --component U3 --output kicad_files/fanout_output4.kicad_pcb --nets "Net-(U1*DQS*)" "Net-(U1*CK*)" --diff-pairs "Net-(U1*DQS*)" "Net-(U1*CK*)" --primary-escape horizontal', unbuffered)
        run('python3 bga_fanout.py kicad_files/fanout_output4.kicad_pcb --component U3 --output kicad_files/fanout_output5.kicad_pcb --nets "*U1A*" "*U1B*" --check-for-previous --primary-escape horizontal', unbuffered)

        # Fan out DDR on DDR chip
        run('python3 bga_fanout.py kicad_files/fanout_output5.kicad_pcb --component U1 --output kicad_files/fanout_output6.kicad_pcb --nets "Net-(U1*DQS*)" "Net-(U1*CK*)" --diff-pairs "Net-(U1*DQS*)" "Net-(U1*CK*)" --primary-escape horizontal --no-inner-top-layer', unbuffered)
        run(f"python3 bga_fanout.py kicad_files/fanout_output6.kicad_pcb --component U1 --output kicad_files/fanout_output7.kicad_pcb --nets '*U1A*' --check-for-previous --primary-escape horizontal {LAYERS_5} --no-inner-top-layer", unbuffered)
        run(f"python3 bga_fanout.py kicad_files/fanout_output7.kicad_pcb --component U1 --output kicad_files/fanout_output.kicad_pcb --nets '*U1B*' --check-for-previous {LAYERS_5} --no-inner-top-layer", unbuffered)

    if ftdi and not onlychecks:
        # Route the FTDI tracks
        ftdi_opts = f"--swappable-nets 'Net-(U2A-DATA_*)' --impedance 50 --vertical-attraction-radius 0 {GEOMETRY} {LAYERS_4}"
        if quick:
            run(f"python3 route.py kicad_files/fanout_output.kicad_pcb kicad_files/routed_output.kicad_pcb --nets 'Net-(U2A-DATA_11*)' {ftdi_opts}", unbuffered)
        else:
            run(f"python3 route.py kicad_files/fanout_output.kicad_pcb kicad_files/routed_output.kicad_pcb --nets 'Net-(U2A-*)' --mps-layer-swap {ftdi_opts}", unbuffered)

    if lvds and not onlychecks:
        # Route LVDS diff pairs
        lvds_opts = f"--impedance 100 {GEOMETRY} {LAYERS_5}"
        if quick:
            # Quick test: route just a few rx1_1* pairs
            run(f"python3 route_diff.py kicad_files/routed_output.kicad_pcb kicad_files/test_diffpair.kicad_pcb --nets '*lvds_rx1_1*' --swappable-nets '*lvds_rx1_1*' {lvds_opts}", unbuffered)
        else:
            # Full test: route all 56 LVDS pairs in two batches - those on bottom and then those on top
            run(f"python3 route_diff.py kicad_files/routed_output.kicad_pcb kicad_files/routed_output_diff12.kicad_pcb --nets '*lvds_rx1_*' '*lvds_rx2_*' '*lvds_rx*clkin1*' '*lvds_rx*clkin2*' --swappable-nets '*lvds_rx1_*' '*lvds_rx2_*' {lvds_opts}", unbuffered)
            run(f"python3 route_diff.py kicad_files/routed_output_diff12.kicad_pcb kicad_files/test_diffpair.kicad_pcb --nets '*lvds_rx3_*' '*lvds_rx4_*' '*lvds_rx*clkin3*' '*lvds_rx*clkin4*' --swappable-nets '*lvds_rx3_*' '*lvds_rx4_*' {lvds_opts}", unbuffered)

    if ram and not onlychecks:
        # Route RAM
        ram_opts = f"--bga-proximity-radius 1 --stub-proximity-radius 1 {GEOMETRY} {LAYERS_5}"
        # DDR diff pairs (DQS, CK)
        run(f"python3 route_diff.py kicad_files/test_diffpair.kicad_pcb kicad_files/test_diffpair_ramdiff.kicad_pcb --nets 'Net-(U1*DQS*)' 'Net-(U1*CK_*)' --length-match-group 'Net-(U1*DQS*)' 'Net-(U1*CK_*)' --mps-layer-swap --diff-pair-intra-match --heuristic-weight 1.5 {ram_opts}", unbuffered)
        # DDR single-ended (DQ, CA, etc.)
        ram_se_opts = f"--swappable-nets 'Net-(U1*DQ*)' --length-match-group auto --max-iterations 1000000 --no-bga-zones U1 {ram_opts}"
        if quick:
            run(f"python3 route.py kicad_files/test_diffpair_ramdiff.kicad_pcb kicad_files/test_diffpair_ram.kicad_pcb --nets 'Net-(U1B-CA*)' {ram_se_opts}", unbuffered)
        else:
            run(f"python3 route.py kicad_files/test_diffpair_ramdiff.kicad_pcb kicad_files/test_diffpair_ram.kicad_pcb --nets 'Net-(U1*)' {ram_se_opts}", unbuffered)

    if planes and not onlychecks:
        # Add and route GND plane
        run(f"python3 route_planes.py kicad_files/test_diffpair_ram.kicad_pcb kicad_files/test_diffpair_ram_planes.kicad_pcb --nets GND '/fpga_adc/VA19|/fpga_adc/VA11|/fpga_adc/VLVDS|/fpga_adc/VD11' --plane-layers In4.Cu In5.Cu --rip-blocker-nets --reroute-ripped-nets {GEOMETRY}", unbuffered)

    if checks or onlychecks:

        if ftdi:
            # Check for FTDI errors and connections
            run('python3 check_drc.py kicad_files/routed_output.kicad_pcb --clearance 0.1 --nets "Net-(U2A-*)"', unbuffered)
            if not quick: run('python3 check_connected.py kicad_files/routed_output.kicad_pcb --nets "Net-(U2A-*)"', unbuffered)

        if lvds:
            # Check LVDS routing for errors and connectivity
            run('python3 check_drc.py kicad_files/test_diffpair.kicad_pcb --clearance 0.1 --nets "*lvds*"', unbuffered)
            if not quick: run('python3 check_connected.py kicad_files/test_diffpair.kicad_pcb --nets "*lvds*"', unbuffered)

        if ram:
            # Check for RAM errors and connections
            run('python3 check_drc.py kicad_files/test_diffpair_ram.kicad_pcb --clearance 0.1 --nets "Net-(U1*)"', unbuffered)
            if not quick: run('python3 check_connected.py kicad_files/test_diffpair_ram.kicad_pcb --nets "Net-(U1*)"', unbuffered)

        if planes:
            # Check for plane errors and connections
            run('python3 check_drc.py kicad_files/test_diffpair_ram_planes.kicad_pcb --clearance 0.1 --clearance-margin 0.1', unbuffered)
            if not quick: run('python3 check_connected.py kicad_files/test_diffpair_ram_planes.kicad_pcb --nets "Net-(U1*)"', unbuffered)


    print("\n=== All tests completed ===")


if __name__ == "__main__":
    main()
