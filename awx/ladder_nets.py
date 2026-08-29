#!/usr/bin/env python3
"""Print the first K nets of the ladder list (west by launch y, then
south), so every rung is a superset of the previous one."""
import sys

WEST = ('SDQM1,SDQ9,SDQ10,SDQ11,SDQS1N,SDQ8,SDQS1P,SDQ12,SDQ13,SDQM0,'
        'SDQ14,SA14,SDQ15,SDQ0,SA10,SA11,SDQ2,SDQ1,SA15,SA12,SDQS0P,'
        'SDQS0N,SDQ4,SA0,SDQ5,SBA1,SDQ6,SDQ3,SA3,SDQ7,SA1,SA4,SCS0,'
        'SCS1,SCKE1,SA6')
SOUTH = ('SA13,SA2,SA5,SA7,SA8,SA9,SBA0,SBA2,SCAS,SCKE0,SODT0,SODT1,'
         'SRAS,SRST,SWE')
ALL = (WEST + ',' + SOUTH).split(',')
K = int(sys.argv[1]) if len(sys.argv) > 1 else 51
print(','.join(ALL[:K]))
