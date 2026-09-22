#!/usr/bin/env python3
"""probe_memo.py -- never pay for the same probe twice (#622, 2026-09-18).

A probe's verdict is a pure function of what the probe sees: the copper
it routes against (the routed board with the coupled set's lanes taken
off), the ends it keeps (the fanout copper of the coupled set), the move
it lays, and the code and knobs that lay and judge it. Two probes with
that key equal give the same board, so the second is read back instead
of run. Measured before this existed: a population run at K51 spent 16
of 21 descents re-descending worlds already known to be at their local
optimum, each descent ~30 probes at ~7 s.

Three memos, one store (`tmp/memo/k<K>/`, JSON per key, shared by every
process that runs at that K):

  probe   replan.probe(): key -> the probe's result dict (+ the paths of
          the boards it wrote, for a result that could stand)
  screen  replan.engine_lays(): key -> (achieved end, exact, in_class)
  closed  evolve.descend(): a world whose descent under given arguments
          gained nothing is CLOSED under those arguments; never descended
          again

Every key carries the CODE HASH (every .py of awx/ and py_router/, the
router binary's version) and the KNOB HASH (the BRAID_/PROBE_/PLAN_/
TAUT_ environment), so an edit to anything that lays or judges copper
starts a cold memo rather than serving a stale verdict. PROBE_MEMO=0
turns the whole thing off; PROBE_MEMO_DIR moves the store.
"""
import glob
import hashlib
import json
import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ENABLED = os.environ.get('PROBE_MEMO', '1') != '0'
MEMO_DIR = os.environ.get('PROBE_MEMO_DIR') or os.path.join(HERE, 'tmp', 'memo')
KNOB_PREFIXES = ('BRAID_', 'PROBE_', 'PLAN_', 'TAUT_')
KNOB_NAMES = ('DST_CLIMB', 'SRC_CLIMB')

_code_hash = None
_stats = {'probe_hit': 0, 'probe_miss': 0, 'screen_hit': 0, 'screen_miss': 0,
          'closed_hit': 0, 'closed_miss': 0, 'stale_files': 0}


def code_hash():
    """One hash over the code that lays and judges a probe. Pinned by
    PROBE_MEMO_CODE for a deliberate carry-over across an edit that is
    known not to change copper (a log line, a comment)."""
    global _code_hash
    if _code_hash is not None:
        return _code_hash
    pin = os.environ.get('PROBE_MEMO_CODE')
    if pin:
        _code_hash = 'pin:' + pin
        return _code_hash
    h = hashlib.sha1()
    files = sorted(glob.glob(os.path.join(HERE, '*.py'))) + \
        sorted(glob.glob(os.path.join(HERE, '..', 'py_router', '*.py')))
    for f in files:
        if os.path.basename(f) in ('evolve_movie.py',):
            continue
        h.update(os.path.basename(f).encode())
        with open(f, 'rb') as fh:
            h.update(fh.read())
    try:
        sys.path.insert(0, os.path.join(HERE, '..', 'rust_router'))
        import grid_router
        h.update(str(getattr(grid_router, '__version__', '?')).encode())
        so = getattr(grid_router, '__file__', '')
        if so and os.path.exists(so):
            st = os.stat(so)
            h.update(f'{st.st_size}:{int(st.st_mtime)}'.encode())
    except Exception:                                  # noqa: BLE001
        h.update(b'no-grid-router')
    h.update(sys.version.encode())
    _code_hash = h.hexdigest()[:16]
    return _code_hash


def knob_hash(extra=None):
    items = sorted((k, v) for k, v in os.environ.items()
                   if k.startswith(KNOB_PREFIXES) or k in KNOB_NAMES)
    items = [(k, v) for k, v in items if k not in ('PROBE_MEMO', 'PROBE_MEMO_DIR', 'PROBE_MEMO_CODE')]
    if extra:
        items += sorted(extra.items())
    return hashlib.sha1(json.dumps(items).encode()).hexdigest()[:12]


# ---------------------------------------------------------------- copper
def _seg_key(s):
    return (round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4), round(s.end_y, 4),
            s.layer, s.net_id, round(s.width, 4))


def _via_key(v):
    return (round(v.x, 4), round(v.y, 4), round(v.size, 4), round(v.drill, 4), v.net_id)


def copper_hash(segments, vias, exclude=None):
    """The multiset of copper as a hash; `exclude` = (segments, vias)
    taken off it first (a lane multiset: each item removed once)."""
    segs = sorted(_seg_key(s) for s in segments)
    vs = sorted(_via_key(v) for v in vias)
    if exclude:
        ex_s, ex_v = exclude
        from collections import Counter
        cs = Counter(_seg_key(s) for s in ex_s)
        cv = Counter(_via_key(v) for v in ex_v)
        keep_s = []
        for k in segs:
            if cs.get(k):
                cs[k] -= 1
                continue
            keep_s.append(k)
        keep_v = []
        for k in vs:
            if cv.get(k):
                cv[k] -= 1
                continue
            keep_v.append(k)
        segs, vs = keep_s, keep_v
    return hashlib.sha1(json.dumps([segs, vs]).encode()).hexdigest()[:16]


def net_copper_hash(pcb, nids):
    """The copper of the given nets on a board, as a hash."""
    nids = set(nids)
    return copper_hash([s for s in pcb.segments if s.net_id in nids],
                       [v for v in pcb.vias if v.net_id in nids])


def key_of(parts):
    return hashlib.sha1(json.dumps(parts, sort_keys=True, default=str).encode()).hexdigest()


# ------------------------------------------------------------------ store
class Store:
    """JSON documents by key under one directory: an in-process dict in
    front of the files, written atomically (temp + rename), so parallel
    descents share one store without a lock."""

    def __init__(self, K, kind):
        self.dir = os.path.join(MEMO_DIR, f'k{K}', kind)
        self.kind = kind
        self.mem = {}
        if ENABLED:
            os.makedirs(self.dir, exist_ok=True)

    def path(self, key):
        return os.path.join(self.dir, key + '.json')

    def get(self, key):
        if not ENABLED:
            return None
        if key in self.mem:
            return self.mem[key]
        p = self.path(key)
        if not os.path.exists(p):
            return None
        try:
            with open(p, encoding='utf-8') as f:
                doc = json.load(f)
        except (OSError, ValueError):
            return None
        self.mem[key] = doc
        return doc

    def put(self, key, doc):
        if not ENABLED:
            return
        self.mem[key] = doc
        p = self.path(key)
        fd, tmp = tempfile.mkstemp(dir=self.dir, prefix='.w', suffix='.json')
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                json.dump(doc, f, indent=0, default=str)
            os.replace(tmp, p)
        except OSError:
            try:
                os.unlink(tmp)
            except OSError:
                pass

    def count(self):
        return len(glob.glob(os.path.join(self.dir, '*.json')))


def stats():
    return dict(_stats)


def bump(k):
    _stats[k] = _stats.get(k, 0) + 1


def summary():
    s = _stats
    return (f'memo: probes {s["probe_hit"]} hit / {s["probe_miss"]} miss'
            + (f' ({s["stale_files"]} hit(s) with boards gone, re-run)' if s['stale_files'] else '')
            + f'; screens {s["screen_hit"]} hit / {s["screen_miss"]} miss'
            + f' [{MEMO_DIR if ENABLED else "OFF"}, code {code_hash()}]')


def files_present(doc):
    return all(os.path.exists(p) for p in (doc.get('files') or {}).values())


# ------------------------------------------------------------ self-test
def _self_test():
    class S:
        def __init__(self, a, b, c, d, L, n, w=0.127):
            self.start_x, self.start_y, self.end_x, self.end_y, self.layer, self.net_id, self.width = a, b, c, d, L, n, w

    class V:
        def __init__(self, x, y, n):
            self.x, self.y, self.size, self.drill, self.net_id = x, y, 0.25, 0.15, n
    segs = [S(0, 0, 1, 0, 'F.Cu', 1), S(0, 0, 1, 0, 'F.Cu', 1), S(1, 0, 1, 1, 'B.Cu', 2)]
    vias = [V(1, 0, 1)]
    h_all = copper_hash(segs, vias)
    h_x1 = copper_hash(segs, vias, exclude=([segs[0]], []))       # one of the two equal segments off
    h_x2 = copper_hash(segs, vias, exclude=([segs[0], segs[1]], []))
    assert h_all != h_x1 != h_x2, 'a multiset exclusion must remove ONE copy per item'
    assert copper_hash(segs[1:], vias) == h_x1, 'excluding one equal item == the board without one'
    assert copper_hash(list(reversed(segs)), vias) == h_all, 'order is not copper'
    k1 = key_of({'a': 1, 'b': [1, 2]})
    k2 = key_of({'b': [1, 2], 'a': 1})
    assert k1 == k2
    d = tempfile.mkdtemp()
    os.environ['PROBE_MEMO_DIR'] = d
    global MEMO_DIR
    MEMO_DIR = d
    st = Store(99, 'probe')
    assert st.get('nokey') is None
    st.put('k', {'x': 1, 'files': {'b': os.path.join(d, 'nofile')}})
    assert Store(99, 'probe').get('k')['x'] == 1, 'a second process reads what the first wrote'
    assert not files_present(st.get('k'))
    assert len(code_hash()) == 16 and knob_hash()
    print(f'probe_memo: self-test ok (code {code_hash()}, knobs {knob_hash()})')


if __name__ == '__main__':
    if '--self-test' in sys.argv:
        _self_test()
    else:
        print(__doc__)
        print(f'code hash {code_hash()}  knob hash {knob_hash()}  dir {MEMO_DIR}')
        for K in sorted({os.path.basename(p) for p in glob.glob(os.path.join(MEMO_DIR, 'k*'))}):
            for kind in ('probe', 'screen', 'closed'):
                n = len(glob.glob(os.path.join(MEMO_DIR, K, kind, '*.json')))
                if n:
                    print(f'  {K} {kind}: {n} entries')
