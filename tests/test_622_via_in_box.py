"""A via stands in an array's grown box when its barrel reaches it (whole_frame.via_in_box): a dog-bone berth's via at
its stub's end stands ON the box's line, and the box is grown by the MEDIAN of the berths' standoffs, so a face whose
stubs end a hair short of it -- coordinates written to a tenth of a micron -- left those vias outside at a 1e-6
tolerance: the solve read them as no via ends, held their lanes to the stub's layer at the berth, and every one laid
joined nothing. One rule for the solve's via ends and the frame's (whole_solve.via_end, whole_frame.build)."""
import os
import sys
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'awx'))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

import whole_frame as wf  # noqa: E402

box = (127.815, 99.855, 133.625, 105.665)
fails = []


def check(what, got, want):
    if got != want:
        fails.append(f'{what}: got {got}, want {want}')


v = lambda x, y, size=0.3: SimpleNamespace(x=x, y=y, size=size)  # noqa: E731
check('a via inside', wf.via_in_box(v(130.0, 102.0), box), True)
check('a berth via a tenth of a micron outside the line', wf.via_in_box(v(127.8149, 101.36), box), True)
check('a berth via on the north line, a hair outside', wf.via_in_box(v(128.92, 99.854901), box), True)
check('a via whose barrel just reaches the box', wf.via_in_box(v(127.815 - 0.149, 101.0), box), True)
check('a via beyond its barrel', wf.via_in_box(v(127.815 - 0.151, 101.0), box), False)
check('a via far off', wf.via_in_box(v(121.8, 97.2), box), False)
check('a via of no size: on the line', wf.via_in_box(SimpleNamespace(x=127.815, y=101.0, size=None), box), True)
if fails:
    print('FAIL: ' + '; '.join(fails))
    sys.exit(1)
print('PASS: a via in the box by its barrel -- on the line, a hair outside it, and not past its own radius')
