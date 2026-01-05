#!/bin/bash
# Test script for BGA fanout refactoring
set -e

rm -f fanout_output*.kicad_pcb 2>/dev/null || true

echo "Running fanout command 1..."
python3 bga_fanout.py fanout_starting_point.kicad_pcb --component U3 --output fanout_output1.kicad_pcb --nets "*U2A*" --primary-escape horizontal --check-for-previous --force-escape-direction > /dev/null 2>&1

echo "Running fanout command 2..."
python3 bga_fanout.py fanout_output1.kicad_pcb --component IC1 --output fanout_output2.kicad_pcb --nets "*lvds_rx*" --diff-pairs "*lvds_rx*" --primary-escape vertical --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu > /dev/null 2>&1

echo "Running fanout command 3..."
python3 bga_fanout.py fanout_output2.kicad_pcb --component U3 --output fanout_output3.kicad_pcb --nets "*lvds_rx*" --diff-pairs "*lvds_rx*" --primary-escape vertical --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu > /dev/null 2>&1

echo "Running fanout command 4..."
python3 bga_fanout.py fanout_output3.kicad_pcb --component U3 --output fanout_output4.kicad_pcb --nets "Net-(U1*DQS*)" "Net-(U1*CK*)" --diff-pairs "Net-(U1*DQS*)" "Net-(U1*CK*)" --primary-escape horizontal > /dev/null 2>&1

echo "Running fanout command 5..."
python3 bga_fanout.py fanout_output4.kicad_pcb --component U3 --output fanout_output5.kicad_pcb --nets "*U1A*" "*U1B*" --check-for-previous --primary-escape horizontal > /dev/null 2>&1

echo "Running fanout command 6..."
python3 bga_fanout.py fanout_output5.kicad_pcb --component U1 --output fanout_output6.kicad_pcb --nets "Net-(U1*DQS*)" "Net-(U1*CK*)" --diff-pairs "Net-(U1*DQS*)" "Net-(U1*CK*)" --primary-escape horizontal > /dev/null 2>&1

echo "Running fanout command 7..."
python3 bga_fanout.py fanout_output6.kicad_pcb --component U1 --output fanout_output7.kicad_pcb --nets "*U1A*" --check-for-previous --primary-escape horizontal --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu > /dev/null 2>&1

echo "Running fanout command 8..."
python3 bga_fanout.py fanout_output7.kicad_pcb --component U1 --output fanout_output.kicad_pcb --nets "*U1B*" --check-for-previous --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu > /dev/null 2>&1

# Use sorted segments hash for deterministic comparison
EXPECTED_SEGMENTS_HASH="b0fcea182ad7ffa8fcd953e04a4c28ff"
EXPECTED_SEGMENT_COUNT="1449"
EXPECTED_VIA_COUNT="918"
EXPECTED_LINE_COUNT="32060"

ACTUAL_SEGMENTS_HASH=$(grep "segment" fanout_output.kicad_pcb | sort | md5 -q)
ACTUAL_SEGMENT_COUNT=$(grep -c "segment" fanout_output.kicad_pcb)
ACTUAL_VIA_COUNT=$(grep -c "^[[:space:]]*(via" fanout_output.kicad_pcb)
ACTUAL_LINE_COUNT=$(wc -l < fanout_output.kicad_pcb | tr -d ' ')

FAILED=0

if [ "$ACTUAL_SEGMENT_COUNT" != "$EXPECTED_SEGMENT_COUNT" ]; then
    echo "FAILURE: Segment count differs! Expected: $EXPECTED_SEGMENT_COUNT, Actual: $ACTUAL_SEGMENT_COUNT"
    FAILED=1
fi

if [ "$ACTUAL_VIA_COUNT" != "$EXPECTED_VIA_COUNT" ]; then
    echo "FAILURE: Via count differs! Expected: $EXPECTED_VIA_COUNT, Actual: $ACTUAL_VIA_COUNT"
    FAILED=1
fi

if [ "$ACTUAL_SEGMENTS_HASH" != "$EXPECTED_SEGMENTS_HASH" ]; then
    echo "FAILURE: Segments hash differs! Expected: $EXPECTED_SEGMENTS_HASH, Actual: $ACTUAL_SEGMENTS_HASH"
    FAILED=1
fi

if [ "$ACTUAL_LINE_COUNT" != "$EXPECTED_LINE_COUNT" ]; then
    echo "FAILURE: Line count differs! Expected: $EXPECTED_LINE_COUNT, Actual: $ACTUAL_LINE_COUNT"
    FAILED=1
fi

if [ "$FAILED" = "0" ]; then
    echo "SUCCESS: Output matches expected"
    echo "  Segments: $ACTUAL_SEGMENT_COUNT"
    echo "  Vias: $ACTUAL_VIA_COUNT"
    echo "  Lines: $ACTUAL_LINE_COUNT"
    exit 0
else
    exit 1
fi
