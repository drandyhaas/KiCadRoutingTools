"""Exact zone-fill geometry from KiCad itself (pcbnew ZONE_FILLER).

Why this exists
---------------
ZoneFillModel and the oracle's island tracer are RASTER APPROXIMATIONS of
KiCad's fill. At marginal sites -- a pour pinched to ~the min-thickness
threshold, a corridor carved by clearance the model prices differently --
the approximation grades the fill CONNECTED where KiCad's exact polygon
math splits it. Every repair actor keyed off the model (region joiner,
island tracer) then sees one region where KiCad sees two, and the only
remaining actor is the link-router, which cannot handle zone|zone links
(zero-length: both endpoints are the pinch point) or walled endpoints.

This module asks KiCad for the truth: drive pcbnew's ZONE_FILLER headless
(refill + save, the fill-fidelity ground-truth recipe), then parse the
saved board's (filled_polygon ...) blocks. Each filled_polygon is one
connected island polygon -- island discovery comes free with the geometry,
so one pcbnew run yields both the diagnosis and the strap targets.

The pcbnew script is STRAIGHT-LINE module scope on purpose: pcbnew
scripting segfaults when board work happens inside functions/loops.

Availability: needs KiCad's bundled python (macOS/Windows) or a system
python with pcbnew (Linux distro installs). All entry points degrade to
None when unavailable; callers fall back to the raster model. That
degradation is SILENT by design, so a discovery bug reads as a modelling
quirk rather than a missing capability -- #647 shipped with the Windows
interpreter unfindable, which cost every Windows user the exact fill in
every consumer. Set `KICAD_PYTHON` to override the search, and print the
reason (not just the symptom) when falling back.
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Dict, List, NamedTuple, Optional, Tuple

import kicad_locate
from swig_compat import patch_swig_iterators as _patch_swig_iterators

# #795: repin_netcodes_from_file() walks board.GetTracks() in the pcbnew
# subprocess, which imports THIS module -- so the alias must be restored here
# too, not only via kicad_parser.
_patch_swig_iterators()

EXACT_FILL_TIMEOUT = 300

_REFILL_SCRIPT = """\
import sys
import pcbnew
src = sys.argv[1]
dst = sys.argv[2]
if len(sys.argv) > 3:
    sys.path.insert(0, sys.argv[3])   # py_router, for kicad_exact_fill
board = pcbnew.LoadBoard(src)
if board is None:
    # LoadBoard returns None (no exception) on a file it rejects;
    # ZONE_FILLER(None) then SEGFAULTS in C++ -- a macOS crash-report popup
    # per occurrence during test runs. Exit cleanly; the caller already
    # degrades to the raster model on any non-REFILL_OK outcome.
    print("REFILL_FAIL LoadBoard returned None")
    sys.exit(3)
# LoadBoard's net propagation REASSIGNS the netcode of a via/track that
# touches a STALE fill (no knockout around it yet) to the zone's net, so
# filling as-loaded keeps NO knockout around it -- phantom pour joins for
# exactly the copper the fill was meant to pull back from. Restore the
# FILE's stored netcodes before filling (mez_rx: 24 fanout vias per load).
try:
    from kicad_exact_fill import repin_netcodes_from_file
    n = repin_netcodes_from_file(board, src)
    if n:
        print("REPIN", n)
except Exception as e:
    print("(net repin skipped:", e, ")")
filler = pcbnew.ZONE_FILLER(board)
zones = board.Zones()
filler.Fill(zones)
# aSkipSettings: dst is parsed for island polygons only; the settings save
# aborts this subprocess (SIGABRT, degrading the fill to the raster model)
# when the staged project predates KiCad 10.
pcbnew.SaveBoard(dst, board, aSkipSettings=True)
print("REFILL_OK", len(list(zones)))
"""


def _file_nets_by_uuid(board_path: str) -> Dict[str, object]:
    """{uuid: stored net (str name or int code)} for every (segment ...)/
    (via ...) in the file.

    The FILE is the net-assignment truth: pcbnew.LoadBoard runs net
    propagation, which silently reassigns items touching stale zone fills
    (see repin_netcodes_from_file). This tool's writer stores item nets BY
    NAME ((net "B19"), no numeric net table at all); stock KiCad stores
    numeric codes. Light balanced-paren scan, no parser dependency."""
    with open(board_path, encoding='utf-8') as f:
        text = f.read()
    out: Dict[str, object] = {}
    for m in re.finditer(r'\((?:segment|via)\b', text):
        depth_, i = 0, m.start()
        while i < len(text):
            c = text[i]
            if c == '(':
                depth_ += 1
            elif c == ')':
                depth_ -= 1
                if depth_ == 0:
                    break
            i += 1
        block = text[m.start():i + 1]
        um = re.search(r'\(uuid\s+"?([0-9a-fA-F-]+)"?\)', block)
        if not um:
            continue
        nm = re.search(r'\(net\s+(\d+)\)', block)
        if nm:
            out[um.group(1).lower()] = int(nm.group(1))
            continue
        nn = re.search(r'\(net\s+"((?:[^"\\]|\\.)*)"\)', block)
        if nn:
            out[um.group(1).lower()] = nn.group(1).replace('\\"', '"')
    return out


def repin_netcodes_from_file(board, board_path: str) -> int:
    """Restore every track/via netcode on a LOADED pcbnew board to what the
    file actually stores. Returns the number of items re-pinned.

    pcbnew.LoadBoard (and any bare BuildConnectivity) runs net propagation:
    an item touching a STALE zone fill -- one poured before the item existed,
    so it has no knockout -- gets the ZONE's netcode instead of its own
    (measured on mez_rx: 24 of 125 fanout vias flipped to V3P3/V1P8/GND at
    load). Matching is by item UUID, so position rounding cannot mispair;
    net names resolve through the board's own net table."""
    want = _file_nets_by_uuid(board_path)
    if not want:
        return 0
    fixed = 0
    for t in board.GetTracks():
        u = getattr(t, 'm_Uuid', None)
        if u is None:
            continue
        spec = want.get(str(u.AsString()).lower())
        if spec is None:
            continue
        if isinstance(spec, int):
            nid = spec
        else:
            net = board.FindNet(spec)
            if net is None:
                continue
            nid = net.GetNetCode()
        if t.GetNetCode() != nid:
            t.SetNetCode(nid)
            fixed += 1
    return fixed


def _win_version_key(path: str):
    """Sort key for a versioned Windows KiCad dir, NEWEST first.

    `C:\\Program Files\\KiCad\\<ver>\\bin\\python.exe` -> the numeric parts of
    <ver>. Plain string sorting orders "9.0" above "10.0", which would hand a
    KiCad 10 user their old KiCad 9 interpreter.

    Thin wrapper over kicad_locate.path_version_key (#763 moved the platform
    knowledge to ONE module); kept because it is this module's published name
    for the key and is pinned by tests. `(0,)` rather than `()` for an
    unversioned path, preserving the original ordering contract.
    """
    return kicad_locate.path_version_key(path) or (0,)


def _this_interpreter() -> str:
    """Path of a REAL python interpreter for THIS process ('' if none found).

    Inside KiCad's embedded python -- i.e. the GUI plugin -- `sys.executable`
    is the HOST C++ BINARY (`.../MacOS/pcbnew`, `pcbnew.exe`), not python3.
    Handing that to subprocess.run([exe, script, board, ...]) does not run a
    script: pcbnew treats every argument as a FILE TO OPEN, so the exact-fill
    refill re-launched the APPLICATION -- once per call -- and KiCad answered
    "Pcbnew:OpenProjectFiles() takes a single filename" (the output path in
    that argv does not exist yet, which is the empty filename in the message).

    So reconstruct the interpreter from sys.prefix, the same way the plugin's
    deps_check._find_python_executable does for `-m pip`. Returning '' is
    correct when nothing is found: the caller then falls through to the
    platform paths below rather than spawning the host binary.
    """
    exe = sys.executable or ""
    if exe and os.path.basename(exe).lower().startswith('python'):
        return exe                      # already a python (CLI, KiCad python3)
    prefix = sys.prefix or getattr(sys, 'base_prefix', '') or ""
    if not prefix:
        return ""
    v = sys.version_info
    if sys.platform == 'win32':
        cands = [os.path.join(prefix, 'python.exe'),
                 os.path.join(prefix, 'Scripts', 'python.exe')]
    else:
        cands = [os.path.join(prefix, 'bin', n) for n in
                 (f'python{v.major}.{v.minor}', f'python{v.major}',
                  'python3', 'python')]
    for c in cands:
        if os.path.isfile(c):
            return c
    return ""


def kicad_python_candidates() -> List[str]:
    """Plausible pcbnew-capable interpreters, best first (unverified).

    Shared with the other re-exec front-ends (`headless_plan`,
    `py_tools/validate_pcb_data`) so a platform's install layout is described
    ONCE. Each caller still verifies -- they want different modules (pcbnew
    here, pcbnew + wx for the GUI launchers).

    The platform layouts themselves live in `kicad_locate` (#763): they were
    duplicated here and in install_plugin.py, and BOTH copies hardcoded drive
    C:, so a KiCad on D: was invisible to every consumer. `_this_interpreter()`
    stays here -- it is about THIS process (the GUI's pcbnew host binary), not
    about where KiCad is installed.
    """
    return kicad_locate.kicad_python_candidates(
        # THIS interpreter, when it is already KiCad's python (the GUI plugin)
        # or a Linux system python with pcbnew installed. _this_interpreter(),
        # NOT sys.executable: in the GUI the latter is the pcbnew HOST BINARY
        # and spawning it opens the app instead of running a script (see
        # _this_interpreter's docstring).
        this_interpreter=_this_interpreter())


_KICAD_PYTHON_MEMO: List[Optional[str]] = []


def find_kicad_python() -> Optional[str]:
    """Path of a python that can import pcbnew, or None.

    VERIFIES the candidate rather than trusting that the file exists: on a
    macOS box with no KiCad, `/usr/bin/python3` exists and cannot import
    pcbnew, so the unverified answer sent every caller down a subprocess that
    could only fail. Memoized -- the probe is one subprocess per process.

    Verification is deliberately OUT-OF-PROCESS even for sys.executable
    (unless pcbnew is already loaded, which settles it for free): a distro
    python that can import pcbnew is the common Linux case, and probing it
    in-process would pull pcbnew's ~100MB into every routing run to answer a
    question the subprocess answers anyway.
    """
    if _KICAD_PYTHON_MEMO:
        return _KICAD_PYTHON_MEMO[0]
    found = None
    for cand in kicad_python_candidates():
        # The free settle: this IS the running interpreter and pcbnew is
        # already imported, so no probe can tell us anything new. Requires
        # `cand is sys.executable` LITERALLY -- a path DERIVED from sys.prefix
        # (the GUI case, see _this_interpreter) is a reconstruction, so it
        # gets probed like any other candidate rather than trusted.
        if cand and cand == sys.executable and 'pcbnew' in sys.modules:
            found = cand           # the GUI plugin on a python-exe front
            break
        if not os.path.isfile(cand):
            continue
        try:
            if subprocess.run([cand, '-c', 'import pcbnew'],
                              capture_output=True, timeout=120).returncode == 0:
                found = cand
                break
        except Exception:
            continue
    _KICAD_PYTHON_MEMO.append(found)
    return found


def live_fill_islands(board):
    """{(net_name, layer_name): [island_polygon, ...]} from an IN-PROCESS
    ZONE_FILLER refill of a live pcbnew BOARD (#424 GUI path).

    Same result shape as refill_islands, with no file round-trip and no
    subprocess: the fill reflects exactly the copper the caller is routing
    against, unsaved edits included -- staleness is impossible by
    construction. Only callable under KiCad's python (pcbnew importable).
    Side effect: the live board's zone fills are recomputed, which is
    derived data KiCad refreshes on any fill action anyway.
    """
    import pcbnew
    zones = board.Zones()
    pcbnew.ZONE_FILLER(board).Fill(zones)
    out: Dict = {}
    for z in zones:
        if z.GetIsRuleArea():
            continue
        net = z.GetNetname()
        for lid in z.GetLayerSet().Seq():
            if not pcbnew.IsCopperLayer(lid):
                continue
            lname = board.GetLayerName(lid)
            sps = z.GetFilledPolysList(lid)
            for i in range(sps.OutlineCount()):
                ol = sps.Outline(i)
                poly = [(pcbnew.ToMM(ol.CPoint(j).x),
                         pcbnew.ToMM(ol.CPoint(j).y))
                        for j in range(ol.PointCount())]
                # Same degenerate-polygon guard as the file path
                # (_MIN_ISLAND_AREA_MM2): KiCad's fracturing emits these on
                # the live board too, and a guard on only one of the two
                # fronts is exactly the CLI/GUI divergence class.
                if len(poly) >= 3 \
                        and _polygon_area(poly) >= _MIN_ISLAND_AREA_MM2:
                    out.setdefault((net, lname), []).append(poly)
    return out


def _balanced_block(text: str, start: int) -> Tuple[str, int]:
    """The parenthesized block starting at text[start] == '(' and the index
    one past its closing paren. Quote-aware (net names may hold parens)."""
    depth = 0
    i = start
    n = len(text)
    in_str = False
    while i < n:
        c = text[i]
        if in_str:
            if c == '"' and text[i - 1] != '\\':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                return text[start:i + 1], i + 1
        i += 1
    return text[start:], n


_XY_RE = re.compile(r'\(xy\s+([-\d.]+)\s+([-\d.]+)\)')
_LAYER_RE = re.compile(r'\(layer\s+"([^"]+)"\)')
_NET_ID_RE = re.compile(r'\(net\s+(\d+)\)')
_NET_NAME_RE = re.compile(r'\(net_name\s+"((?:[^"\\]|\\.)*)"\)')
# KiCad 10 files reference zone nets BY NAME with no net table (#344 class):
# (net "Earth") -- there is no numeric token and no net_name attribute.
_NET_STR_RE = re.compile(r'\(net\s+"((?:[^"\\]|\\.)*)"\)')


# Smallest fill polygon that can be real copper, mm^2. KiCad's fracturing
# emits DEGENERATE polygons -- 3-5 collinear or ~coincident points enclosing
# no area -- alongside the real islands, and they are indistinguishable from
# real islands to anything that just counts (filled_polygon ...) blocks.
# Measured on storm_tracker's routed board: 741 of 788 polygons enclose less
# than 1e-3 mm^2 (verbatim examples: three points sharing a y to 6 decimals;
# three points 2 um apart), and the real copper starts at 1.04e-2 mm^2 -- a
# THREE-ORDER-OF-MAGNITUDE gap, so the exact cut is not load-bearing anywhere
# in [2e-4, 1e-2].
#
# Deliberately NOT a KiCad island_removal_mode / island_area_min decision:
# those choose which REAL islands the filler keeps (corpus values are ~5 mm^2,
# 5000x this), and honoring them is plane_region_connector's job
# (_island_kept_by_filler). This is strictly a "this polygon encloses no
# copper" test, so it is correct under every removal mode. 1e-3 mm^2 is a
# 32 um square -- an order of magnitude below the ~0.1 mm minimum feature any
# fab will image.
_MIN_ISLAND_AREA_MM2 = 1e-3


def _polygon_area(poly) -> float:
    """Absolute shoelace area of a closed polygon, mm^2."""
    a = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2.0


_REFILL_MEMO: Dict[tuple, dict] = {}
_REFILL_MEMO_CAP = 8
# Keys whose refill FAILED (audit finding): remember the "no" so N consumers
# do not each pay a pcbnew subprocess to rediscover it.
_REFILL_FAILED: set = set()


def _refill_memo_key(board_file: str, project_from: str = None):
    """Identity of everything a refill's result depends on: the board file's
    bytes (path + mtime_ns + size as proxy) and the EFFECTIVE .kicad_pro
    (the sibling, else project_from's -- netclass rules change the fill).
    None = unkeyable (missing file); caller skips the memo."""
    try:
        bp = os.path.abspath(board_file)
        st = os.stat(bp)
        key = [bp, st.st_mtime_ns, st.st_size]
        pro = os.path.splitext(bp)[0] + '.kicad_pro'
        if not os.path.isfile(pro) and project_from:
            pro = os.path.splitext(os.path.abspath(project_from))[0] + '.kicad_pro'
        if os.path.isfile(pro):
            pst = os.stat(pro)
            key += [os.path.abspath(pro), pst.st_mtime_ns, pst.st_size]
        else:
            key += [None]
        return tuple(key)
    except OSError:
        return None


#: Why an exact fill produced no islands. A CLOSED vocabulary (#713 item 4):
#: before this, FIVE distinct causes all returned a bare `None`, so no caller
#: could tell "this machine has no pcbnew" from "this board is too slow for a
#: 300 s subprocess" -- and exactly one of the five, `timeout`, is a fact about
#: the MACHINE rather than about the board. `plane_fragility` reported that one
#: to the user as "the KiCad refill failed", which is the wrong repair.
REFILL_REASONS = ('ok', 'no_kicad_python', 'memoised_failure',
                  'refill_failed', 'timeout', 'error')


class RefillStatus(NamedTuple):
    """Why `refill_islands_ex` returned the islands it did (or did not).

    `reason` is one of `REFILL_REASONS`; `ok` is the only one carrying
    islands. `detail` is a short free-text clause for the human line, never
    parsed. `elapsed_s` is filled on the timeout arm so a caller can say how
    long it waited rather than only that it gave up.

    The module docstring already asks callers to "print the reason (not just
    the symptom) when falling back" -- this is the channel that makes that
    possible. It is a return value rather than a module-global precisely
    because a module-global would reintroduce the run-history dependence
    #713 is about.
    """
    reason: str
    detail: str = ''
    elapsed_s: Optional[float] = None

    @property
    def ok(self) -> bool:
        return self.reason == 'ok'

    @property
    def is_timeout(self) -> bool:
        """The one arm whose answer depends on how fast THIS MACHINE is.

        It is the arm that makes an output depend on CPU speed, which is the
        whole of #713, and a caller deciding between candidates must not treat
        it as a fact about the board.

        This says nothing about whether the OTHER reasons are stable: an
        `error` (a full disk, a transient OSError) and a `memoised_failure`
        (scoped to this process, by its own wording) are not "the same answer
        next run" either. Only `no_kicad_python` is genuinely stable, and
        `PlaneScoreStatus.uniform` is where that finer question is answered --
        an earlier draft of this docstring claimed every non-timeout reason was
        stable, which contradicted `uniform` in the same change.
        """
        return self.reason == 'timeout'

    def why(self) -> str:
        """One clause naming the real cause, for a fallback notice."""
        base = {
            'ok': 'the KiCad refill succeeded',
            'no_kicad_python': ('no python with pcbnew found; set KICAD_PYTHON '
                                'to KiCad\'s bundled interpreter'),
            'memoised_failure': ('the KiCad refill already failed on these '
                                 'exact board+project bytes in this process'),
            'refill_failed': 'the KiCad refill failed',
            'timeout': 'the KiCad refill TIMED OUT',
            'error': 'the KiCad refill could not be attempted',
        }.get(self.reason)
        if base is None:
            # REFILL_REASONS is otherwise only a convention. Say so loudly
            # rather than echoing the token, which reads like a real clause.
            return f'unknown refill reason {self.reason!r}'
        if self.reason == 'timeout' and self.elapsed_s is not None:
            base += f' after {self.elapsed_s:.0f}s'
        return f'{base} ({self.detail})' if self.detail else base


def _ok_status(out, elapsed_s=None, memo=False) -> RefillStatus:
    """The success status, DERIVED from the islands rather than written at one
    return site.

    An EMPTY dict is a sixth outcome that several callers treat exactly like
    None (they test truthiness, not `is None`): the refill ran and KiCad poured
    nothing, which is a real answer about the board rather than an
    unavailability. Deriving the detail here is what makes that survive the
    MEMO -- an earlier draft wrote the clause only at the fresh-refill return,
    so the second consumer of the same unchanged file got a bare 'memo hit' and
    the distinction evaporated in exactly the many-consumers-one-file case the
    memo exists for.
    """
    detail = '' if out else 'the refill ran and produced no islands'
    if memo:
        detail = f'{detail}; memo hit' if detail else 'memo hit'
    return RefillStatus('ok', detail, elapsed_s)


def refill_islands(board_file: str, timeout: int = EXACT_FILL_TIMEOUT,
                   verbose: bool = False, project_from: str = None
                   ) -> Optional[Dict[Tuple[str, str],
                                      List[List[Tuple[float, float]]]]]:
    """The islands alone. See `refill_islands_ex` for why there are none.

    Kept at its original signature and return type so the existing call sites
    are untouched; a caller that makes a DECISION on the absence should use
    `refill_islands_ex` instead.
    """
    return refill_islands_ex(board_file, timeout=timeout, verbose=verbose,
                             project_from=project_from)[0]


def refill_islands_ex(board_file: str, timeout: int = EXACT_FILL_TIMEOUT,
                      verbose: bool = False, project_from: str = None
                      ) -> Tuple[Optional[Dict[Tuple[str, str],
                                               List[List[Tuple[float, float]]]]],
                                 RefillStatus]:
    """({(net_name, layer): [island_polygon, ...]}, RefillStatus) from a KiCad
    refill of `board_file`. The islands are None whenever `status.ok` is False,
    and the status says WHICH of the six causes it was.

    The board is staged into a temp dir WITH its sibling .kicad_pro so the
    refill runs at the project's real netclasses (a bare board refills at
    stock rules and shrinks tight pours -- the phantom-divergence trap).
    Each (filled_polygon ...) block of the saved board is one connected
    island polygon (KiCad's fracture output), so island discovery is free.

    MEMOIZED on (board bytes, effective .kicad_pro bytes) identity -- a
    refill is a pure function of those, and the plane-finalize pipeline
    (#562) refills the same unchanged file from several consumers
    (per-net fragility, the oracle's first round) at a pcbnew subprocess
    apiece. A file rewrite changes mtime_ns -> miss. Deep-copied on hit so
    a caller mutating its polygons cannot poison later consumers.
    """
    kpy = find_kicad_python()
    if kpy is None:
        return None, RefillStatus('no_kicad_python')
    _mk = _refill_memo_key(board_file, project_from)
    # FAILURES are memoized too (audit finding): the memo used to store only
    # successes, so a board whose refill fails re-spawned a pcbnew subprocess
    # for every consumer and every oracle round -- the slowest possible way to
    # learn the same "no" repeatedly. Keyed on content like the success path,
    # so the next edit to the board retries for real.
    if _mk is not None and _mk in _REFILL_FAILED:
        return None, RefillStatus('memoised_failure')
    if _mk is not None and _mk in _REFILL_MEMO:
        import copy as _copy
        return (_copy.deepcopy(_REFILL_MEMO[_mk]),
                _ok_status(_REFILL_MEMO[_mk], memo=True))
    tmpdir = tempfile.mkdtemp(prefix='exact_fill_')
    _t0 = time.monotonic()
    try:
        stem = os.path.splitext(os.path.basename(board_file))[0]
        staged = os.path.join(tmpdir, stem + '.kicad_pcb')
        shutil.copyfile(board_file, staged)
        sib_pro = os.path.splitext(board_file)[0] + '.kicad_pro'
        if not os.path.isfile(sib_pro) and project_from:
            # Mid-chain boards have no sibling project yet (the writeback
            # runs after the oracle, #338) -- stage the INPUT's project so
            # the refill runs at the real netclasses, not stock rules.
            sib_pro = os.path.splitext(project_from)[0] + '.kicad_pro'
        if os.path.isfile(sib_pro):
            shutil.copyfile(sib_pro, os.path.join(tmpdir,
                                                  stem + '.kicad_pro'))
        script = os.path.join(tmpdir, 'refill.py')
        with open(script, 'w') as f:
            f.write(_REFILL_SCRIPT)
        filled = os.path.join(tmpdir, stem + '_filled.kicad_pcb')
        # 3rd arg: py_router dir, so the script can import kicad_exact_fill
        # for the net re-pin (LoadBoard flips netcodes over stale fills).
        r = subprocess.run([kpy, script, staged, filled,
                            os.path.dirname(os.path.abspath(__file__))],
                           capture_output=True, text=True, timeout=timeout)
        if 'REFILL_OK' not in (r.stdout or '') or not os.path.isfile(filled):
            _why = (r.stderr or '').strip()[-200:]
            if verbose:
                print(f"  (exact-fill refill failed: rc={r.returncode} {_why})")
            if _mk is not None:
                if len(_REFILL_FAILED) >= _REFILL_MEMO_CAP:
                    _REFILL_FAILED.clear()
                _REFILL_FAILED.add(_mk)
            return None, RefillStatus('refill_failed',
                                      f'rc={r.returncode} {_why}'.strip(),
                                      time.monotonic() - _t0)
        with open(filled, 'r', encoding='utf-8') as f:
            text = f.read()
    except subprocess.TimeoutExpired:
        # A TIMEOUT is not remembered as a failure: it is a budget verdict,
        # not a property of the board, and a later step may legitimately have
        # more headroom. (The kicad-cli twin has _ORACLE_TIMED_OUT for that.)
        _dt = time.monotonic() - _t0
        if verbose:
            print(f"  (exact-fill refill timed out after {_dt:.0f}s)")
        return None, RefillStatus('timeout',
                                  f'limit {timeout}s', _dt)
    except Exception as e:
        if verbose:
            print(f"  (exact-fill unavailable: {e})")
        if _mk is not None:
            if len(_REFILL_FAILED) >= _REFILL_MEMO_CAP:
                _REFILL_FAILED.clear()
            _REFILL_FAILED.add(_mk)
        return None, RefillStatus('error', f'{type(e).__name__}: {e}',
                                  time.monotonic() - _t0)
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)
    _out = parse_filled_islands(text)
    if _mk is not None:
        if len(_REFILL_MEMO) >= _REFILL_MEMO_CAP:
            _REFILL_MEMO.pop(next(iter(_REFILL_MEMO)))
        import copy as _copy
        _REFILL_MEMO[_mk] = _copy.deepcopy(_out)
    return _out, _ok_status(_out, time.monotonic() - _t0)


def parse_filled_islands(text: str
                         ) -> Dict[Tuple[str, str],
                                   List[List[Tuple[float, float]]]]:
    """Parse (zone ...)/(filled_polygon ...) blocks of saved board text into
    {(net_name, layer): [island_polygon, ...]}. Net resolution: the zone's
    net_name token when present (KiCad <=9 keeps it), else the numeric
    (net N) mapped through the file's net table / item net names."""
    id_to_name: Dict[int, str] = {}
    for m in re.finditer(r'\(net\s+(\d+)\s+"((?:[^"\\]|\\.)*)"\)', text):
        id_to_name[int(m.group(1))] = m.group(2)
    out: Dict[Tuple[str, str], List[List[Tuple[float, float]]]] = {}
    pos = 0
    while True:
        z = text.find('(zone', pos)
        if z < 0:
            break
        block, pos = _balanced_block(text, z)
        head = block[:block.find('(filled_polygon')] \
            if '(filled_polygon' in block else block
        nm = _NET_NAME_RE.search(head) or _NET_STR_RE.search(head)
        net_name = nm.group(1) if nm else None
        if net_name is None:
            ni = _NET_ID_RE.search(head)
            if ni:
                net_name = id_to_name.get(int(ni.group(1)))
        if not net_name:
            continue
        fp_pos = 0
        while True:
            fp = block.find('(filled_polygon', fp_pos)
            if fp < 0:
                break
            fp_block, fp_pos = _balanced_block(block, fp)
            lm = _LAYER_RE.search(fp_block)
            if not lm:
                continue
            poly = [(float(a), float(b))
                    for a, b in _XY_RE.findall(fp_block)]
            if len(poly) >= 3 \
                    and _polygon_area(poly) >= _MIN_ISLAND_AREA_MM2:
                out.setdefault((net_name, lm.group(1)), []).append(poly)
    return out


def point_in_poly(x: float, y: float,
                  poly: List[Tuple[float, float]]) -> bool:
    """Even-odd ray cast."""
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xc:
                inside = not inside
    return inside


def exact_clusters(pcb_data, net_id: int, islands,
                   tolerance: float = 0.06):
    """Deterministic KiCad-truth connectivity clusters for one net.

    THE ORACLE REPLACEMENT (#490): kicad-cli DRC's connectivity is
    nondeterministic (threaded refill/ratsnest -- rp2040 gave three
    different unconnected-item reports on one unchanged board; orangecrab's
    repair graded 103/65/92 across identical-input runs because the welds
    chased a different report each round). pcbnew's ZONE_FILLER itself is
    measured-deterministic (3x byte-identical island signatures on both
    boards), so KiCad's fill truth + a deterministic union-find gives the
    same verdict every run -- and hands back real cluster geometry for
    strapping instead of ratsnest anchor names.

    `islands` = the net's exact filled_polygon list as [(layer, poly), ...]
    (from refill_islands). Copper components come from kicad_oracle's
    authoritative _net_track_components (no fill credit); an island joins a
    component when any of its segments (on the island's layer) or vias
    (any layer -- barrels pierce every fill) touches the island polygon;
    pads join via _cluster_points at their center, and directly join
    islands whose polygon contains them on a shared layer.

    Returns clusters largest-first, each
      {'islands': [idx...], 'pads': [Pad...], 'points': [(x, y, layer)...],
       'has_pads': bool}. len(clusters) > 1 => KiCad demands links.
    """
    import math as _m
    from kicad_oracle import _net_track_components, _cluster_points
    from kicad_parser import pad_is_plated_through

    comps = _net_track_components(pcb_data, net_id)
    comp_of_seg, comp_of_via, segs, vias, _ = comps
    # _net_track_components returns EMPTY comp lists when the net has no
    # segments (via-only nets) and None entries for vias the graph did not
    # place -- give every such item its own component identity.
    if len(comp_of_seg) != len(segs):
        comp_of_seg = ['s%d' % i for i in range(len(segs))]
    if len(comp_of_via) != len(vias):
        comp_of_via = ['v%d' % j for j in range(len(vias))]
    comp_of_via = [r if r is not None else 'v%d' % j
                   for j, r in enumerate(comp_of_via)]
    pads = list(pcb_data.pads_by_net.get(net_id, []))

    parent: dict = {}

    def find(a):
        parent.setdefault(a, a)
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    _samples_cache = {}

    def _island_samples(ii):
        if ii not in _samples_cache:
            _samples_cache[ii] = sample_poly_edges(islands[ii][1], cap=2000)
        return _samples_cache[ii]

    # Island <-> copper components.
    for ii, (layer, poly) in enumerate(islands):
        inode = ('i', ii)
        parent.setdefault(inode, inode)
        for i, s in enumerate(segs):
            if s.layer != layer:
                continue
            if point_in_poly(s.start_x, s.start_y, poly) \
                    or point_in_poly(s.end_x, s.end_y, poly):
                union(inode, ('c', comp_of_seg[i]))
        for j, v in enumerate(vias):
            if point_in_poly(v.x, v.y, poly):
                union(inode, ('c', comp_of_via[j]))

    # Pads <-> copper (position lookup through the authoritative comps) and
    # pads <-> islands (containment on a shared layer / plated barrel).
    for p in pads:
        pnode = ('p', id(p))
        parent.setdefault(pnode, pnode)
        cu = [l for l in p.layers if l.endswith('.Cu')]
        _th = pad_is_plated_through(p)
        lookup_layers = (None,) if _th or '*.Cu' in p.layers else tuple(cu)
        for lyr in lookup_layers:
            _, root = _cluster_points(pcb_data, net_id, p.global_x,
                                      p.global_y, lyr, comps, tol=tolerance
                                      + max(p.size_x, p.size_y) / 2.0)
            if root is not None:
                union(pnode, ('c', root))
                break
        # Pad <-> island: NEVER by center containment -- a thermal-relief
        # connection excludes the pad area from the filled polygon (the
        # spokes land on the pad ring), so the center tests OUTSIDE the
        # fill. Touch = any island boundary sample within the pad's copper
        # reach (+tol). zone_connection=none pads stay separate for free:
        # their fill keeps full clearance, beyond the reach test.
        _pr = max(p.size_x, p.size_y) / 2.0 + tolerance
        for ii, (layer, poly) in enumerate(islands):
            if not (_th or '*.Cu' in p.layers or layer in p.layers):
                continue
            if point_in_poly(p.global_x, p.global_y, poly):
                union(pnode, ('i', ii))
                continue
            for sx, sy in _island_samples(ii):
                if _m.hypot(sx - p.global_x, sy - p.global_y) <= _pr:
                    union(pnode, ('i', ii))
                    break

    # Pad <-> pad: copper-overlap union (the dual-pad / castellated class).
    # Two same-net pads whose copper physically overlaps are one cluster
    # even when neither touches a track or island on its OWN layers -- a
    # via-in-pad footprint (0.9mm SMD pad + 0.3mm TH sibling 0.25mm away)
    # otherwise yields one manufactured link per pad pair: dilemma graded
    # 141 phantom links and the oracle welded every one (286s, ~420 junk
    # segments) while kicad-cli's own connectivity reported the copper
    # connected. Shape-accurate test (#346: bounding circles
    # false-connect); the center-distance prefilter keeps it O(near
    # pairs); NPTH pads have no copper to touch (#328).
    from check_connected import _pads_copper_touch
    from net_queries import expand_pad_layers as _epl
    _cu_layers = pcb_data.board_info.copper_layers
    _cupads = [p for p in pads
               if getattr(p, 'pad_type', '') != 'np_thru_hole']

    def _shares_cu(pi, pj):
        if pad_is_plated_through(pi) or pad_is_plated_through(pj):
            return True
        return bool(set(_epl(pi.layers, _cu_layers))
                    & set(_epl(pj.layers, _cu_layers)))

    for i in range(len(_cupads)):
        pi = _cupads[i]
        _ri = max(pi.size_x, pi.size_y) / 2.0
        for j in range(i + 1, len(_cupads)):
            pj = _cupads[j]
            _reach = _ri + max(pj.size_x, pj.size_y) / 2.0 + tolerance
            if abs(pi.global_x - pj.global_x) > _reach \
                    or abs(pi.global_y - pj.global_y) > _reach:
                continue
            if _shares_cu(pi, pj) and _pads_copper_touch(pi, pj, tolerance):
                union(('p', id(pi)), ('p', id(pj)))

    # Every copper component gets a node even if it touched nothing.
    for r in set(comp_of_seg) | set(comp_of_via):
        parent.setdefault(('c', r), ('c', r))

    clusters: dict = {}

    def bucket(root):
        return clusters.setdefault(root, {'islands': [], 'pads': [],
                                          'points': [], 'has_pads': False})

    for ii, (layer, poly) in enumerate(islands):
        c = bucket(find(('i', ii)))
        c['islands'].append(ii)
        c['points'].extend((x, y, layer)
                           for x, y in sample_poly_edges(poly, cap=800))
    for p in pads:
        c = bucket(find(('p', id(p))))
        c['pads'].append(p)
        c['has_pads'] = True
        cu = [l for l in p.layers if l.endswith('.Cu')]
        c['points'].append((p.global_x, p.global_y, cu[0] if cu else None))
    for i, s in enumerate(segs):
        c = bucket(find(('c', comp_of_seg[i])))
        c['points'].append((s.start_x, s.start_y, s.layer))
        c['points'].append((s.end_x, s.end_y, s.layer))
    for j, v in enumerate(vias):
        c = bucket(find(('c', comp_of_via[j])))
        c['points'].append((v.x, v.y, None))
    return sorted(clusters.values(), key=lambda c: -len(c['points']))


def nearest_pair(pa_pts, pb_pts, same_layer_within: float = 3.0):
    """Nearest-approach pair between two point sets (items indexable as
    (x, y, layer[, ...])); returns (pa, pb) or (None, None).

    FULL point sets via KD-tree (#648): the old O(n*m) scan in
    exact_unconnected capped the primary cluster at its first 1500 points --
    island-ordered, so on the #589 champion the true 0.036mm gap to a
    mid-list island sat beyond the cap and the link anchored at a 1.34mm
    pair two ball-columns west; every downstream weld then fought the wrong
    corridor.

    LAYER-AWARE: the global nearest is often a CROSS-LAYER overlap
    (distance ~0 where two layers' fills interpenetrate) whose only bond is
    a via -- frequently infeasible in the very pocket that isolated the
    cluster -- while a nearby same-layer gap takes a plain track weld.
    Prefer a same-layer pair within `same_layer_within` mm; layer None
    (via points) matches any layer."""
    import numpy as _np
    from scipy.spatial import cKDTree as _KD
    if not pa_pts or not pb_pts:
        return None, None
    aa = _np.asarray([(p[0], p[1]) for p in pa_pts])
    bb = _np.asarray([(p[0], p[1]) for p in pb_pts])
    dq, jq = _KD(bb).query(aa, k=1)
    i = int(_np.argmin(dq))
    best = (pa_pts[i], pb_pts[int(jq[i])])
    lay_best = None
    for L in sorted({p[2] for p in pa_pts if len(p) > 2 and p[2]}):
        ai = [k for k, p in enumerate(pa_pts)
              if len(p) > 2 and (p[2] == L or p[2] is None)]
        bi = [k for k, p in enumerate(pb_pts)
              if len(p) > 2 and (p[2] == L or p[2] is None)]
        if not ai or not bi:
            continue
        dq2, jq2 = _KD(bb[bi]).query(aa[ai], k=1)
        k2 = int(_np.argmin(dq2))
        d2 = float(dq2[k2])
        if lay_best is None or d2 < lay_best[0]:
            lay_best = (d2, pa_pts[ai[k2]], pb_pts[bi[int(jq2[k2])]])
    if lay_best is not None and lay_best[0] <= same_layer_within:
        return lay_best[1], lay_best[2]
    return best


def exact_unconnected(board_file: str, net_names=None, pcb_data=None,
                      verbose: bool = False, project_from: str = None):
    """Drop-in DETERMINISTIC replacement for kicad_oracle.kicad_unconnected:
    [(net, (x, y, layer|None, kind), (x, y, layer|None, kind)), ...] --
    one link per secondary cluster, anchored at the true nearest-approach
    pair between that cluster and the net's primary cluster (real strap
    geometry, not ratsnest item names). None when pcbnew is unavailable.

    Scope: nets in `net_names` get full island+copper clustering; every
    OTHER net with copper gets a cheap pad-less-component sweep so the
    debris pass keeps seeing stranded-fragment links (XTAL_O class).
    """
    import math as _m
    # LIVE board first (#424 provider): the GUI applies each step's copper to
    # the in-memory board and only saves at the end, so `board_file` is the
    # ORIGINAL board mid-plan -- refilling it would price the oracle against a
    # board missing every step's copper. When pcb_data carries a provider it IS
    # the current board, so ask it. Same {(net, layer): [poly, ...]} shape.
    # (plane_fragility.py has consumed this provider since #424. The oracle
    # was MEANT to be the last consumer still insisting on a file, but it
    # never adopted this: all three of its call sites -- kicad_oracle's rounds
    # loop and its for-else refetch, and repair_planes -- still pass no
    # pcb_data, so from the oracle this branch is dead and every round pays a
    # file round-trip + pcbnew refill. Audit finding; left as-is rather than
    # wired blind, because the oracle mutates the board FILE between rounds
    # and a live provider would have to be re-derived each round to match.)
    provider = getattr(pcb_data, 'exact_fill_provider', None)
    if provider is not None:
        try:
            m = provider()
        except Exception as e:
            if verbose:
                print(f"  exact_unconnected: live fill provider failed ({e}); "
                      f"falling back to the file")
            m = refill_islands(board_file, verbose=verbose,
                               project_from=project_from)
    else:
        m = refill_islands(board_file, verbose=verbose,
                           project_from=project_from)
    if m is None:
        return None
    if pcb_data is None:
        from kicad_parser import parse_kicad_pcb
        pcb_data = parse_kicad_pcb(board_file)
    names = set(net_names or [])
    by_net: Dict[str, list] = {}
    for (net, layer), polys in m.items():
        by_net.setdefault(net, []).extend((layer, p) for p in polys)

    links = []

    _nearest = nearest_pair

    name_to_id = {net.name: nid for nid, net in pcb_data.nets.items()}
    for name in sorted(names):
        nid = name_to_id.get(name)
        if nid is None:
            continue
        cl = exact_clusters(pcb_data, nid, by_net.get(name, []))
        if len(cl) <= 1:
            continue
        primary = cl[0]
        ppts = primary['points']
        for c in cl[1:]:
            a, b = _nearest(c['points'], ppts)
            if a is None:
                continue
            kind_a = 'zone' if c['islands'] and not c['pads'] else \
                ('pad' if c['pads'] else 'track')
            links.append((name, (a[0], a[1], a[2], kind_a),
                          (b[0], b[1], b[2], 'zone'
                           if primary['islands'] else 'track')))

    # Debris sweep on every other net with copper: pad-less components.
    from kicad_oracle import _net_track_components
    for nid, net in pcb_data.nets.items():
        if not net.name or net.name in names:
            continue
        segs = [s for s in pcb_data.segments if s.net_id == nid]
        vias = [v for v in pcb_data.vias if v.net_id == nid]
        if not segs and not vias:
            continue
        pads = pcb_data.pads_by_net.get(nid, [])
        comps = _net_track_components(pcb_data, nid)
        comp_of_seg, comp_of_via, csegs, cvias, _cl2 = comps
        if not comp_of_seg and not comp_of_via:
            continue
        roots = set(comp_of_seg) | set(r for r in comp_of_via
                                       if r is not None)
        # A single-component net with NO pads demands nothing (one cluster,
        # no ratsnest). Debris semantics: a pad-less component matters when
        # the net ALSO has pads elsewhere (XTAL_O class) or splits into
        # multiple components.
        if len(roots) <= 1 and not pads:
            continue
        # A component with no pad within reach of any of its items is
        # debris-candidate; emit one self-link so the oracle's stranded-
        # fragment deletion can inspect it (it re-derives authoritatively).
        for root in sorted(roots, key=str):
            pts = []
            for i, s in enumerate(csegs):
                if comp_of_seg[i] == root:
                    # BOTH ends (audit, #659): a component whose only contact
                    # with a pad is at a segment's END was credited to nothing
                    # and, worse, a component whose START merely passed over a
                    # foreign-layer pad was credited to it.
                    pts.append((s.start_x, s.start_y, s.layer))
                    pts.append((s.end_x, s.end_y, s.layer))
            for j, v in enumerate(cvias):
                if j < len(comp_of_via) and comp_of_via[j] == root:
                    pts.append((v.x, v.y, None))
            if not pts:
                continue

            def _pad_reaches(p, x, y, lyr):
                """Could this pad actually TOUCH this point -- same layer,
                not just the same XY?

                The old test was pure XY, so an SMD pad on F.Cu credited a
                B.Cu component sitting under it. Measured on daisho:
                /ddr2/DM6's dead 3-segment B.Cu component starts at
                (124.50, 87.50), the centre of U1.AB9 -- an F.Cu-only SMD pad
                -- so has_pad was True, no link was emitted, and the
                authoritative deletion below never got to look at it. That is
                the fragment that then survived eleven chain steps.

                This filter only decides whether to ASK: the caller's
                _delete_stranded_link_fragment re-derives the verdict from the
                authoritative graph. So erring toward emitting costs one
                cheap check, while erring toward skipping loses the fragment
                for good -- the filter must be conservative in THAT
                direction."""
                if _m.hypot(p.global_x - x, p.global_y - y) > \
                        max(p.size_x, p.size_y) / 2.0 + 0.06:
                    return False
                if lyr is None:
                    return True      # via barrel: spans layers, any pad plausible
                from kicad_parser import pad_is_plated_through
                if pad_is_plated_through(p):
                    return True      # plated barrel ties every copper layer
                _pl = p.layers or ()
                return lyr in _pl or any(str(_l).startswith('*') for _l in _pl)

            has_pad = any(_pad_reaches(p, x, y, lyr)
                          for p in pads for x, y, lyr in pts)
            if not has_pad:
                x, y, lyr = pts[0]
                links.append((net.name, (x, y, lyr, 'track'),
                              (x, y, lyr, 'track')))
    return links


def rasterize_interior(poly: List[Tuple[float, float]],
                       window: Tuple[float, float, float, float],
                       step: float = 0.25, inset: float = 0.0,
                       edge_samples=None) -> List[Tuple[float, float]]:
    """Grid points inside `poly`, clipped to the `window` bbox, at least
    `inset` from the boundary (distance to `edge_samples`). Row-vectorized
    ray cast (numpy) -- pure-python point-in-poly over a window against a
    10k-vertex island polygon is prohibitive."""
    import numpy as np
    x0, y0, x1, y1 = window
    xs = np.arange(x0, x1 + step, step)
    ys = np.arange(y0, y1 + step, step)
    if not len(xs) or not len(ys):
        return []
    px = np.asarray([p[0] for p in poly])
    py = np.asarray([p[1] for p in poly])
    x2 = np.roll(px, -1)
    y2 = np.roll(py, -1)
    inside = np.zeros((len(xs), len(ys)), dtype=bool)
    for j, yv in enumerate(ys):
        mask = (py > yv) != (y2 > yv)
        if not mask.any():
            continue
        xc = px[mask] + (yv - py[mask]) * (x2[mask] - px[mask]) \
            / (y2[mask] - py[mask])
        cnt = (xs[:, None] < xc[None, :]).sum(axis=1)
        inside[:, j] = (cnt % 2) == 1
    ii, jj = np.nonzero(inside)
    pts = np.column_stack([xs[ii], ys[jj]])
    if inset > 0 and edge_samples is not None and len(pts):
        es = np.asarray([(p[0], p[1]) for p in edge_samples])
        keep = np.ones(len(pts), dtype=bool)
        chunk = 4000
        for i0 in range(0, len(pts), chunk):
            d2 = ((pts[i0:i0 + chunk, None, :] - es[None, :, :]) ** 2
                  ).sum(axis=2).min(axis=1)
            keep[i0:i0 + chunk] = d2 >= inset * inset
        pts = pts[keep]
    return [(float(x), float(y)) for x, y in pts]


def sample_poly_edges(poly: List[Tuple[float, float]], step: float = 0.25,
                      cap: int = 4000) -> List[Tuple[float, float]]:
    """Points along the polygon boundary every ~`step` mm (vertices always
    included), decimated to <= cap points."""
    import math
    pts: List[Tuple[float, float]] = []
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        pts.append((x1, y1))
        seg_len = math.hypot(x2 - x1, y2 - y1)
        for k in range(1, int(seg_len / step)):
            t = k * step / seg_len
            pts.append((x1 + (x2 - x1) * t, y1 + (y2 - y1) * t))
    if len(pts) > cap:
        stride = len(pts) // cap + 1
        pts = pts[::stride]
    return pts
