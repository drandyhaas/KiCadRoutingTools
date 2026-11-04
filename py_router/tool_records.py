"""Where the ``kicad_routing_tools`` records live, and which copy a reader gets.

THE RECORDS. Everything this repo carries down a chain beside the DRC floors
sits in one namespace: ``fab_floor_origin`` (ad7f24de), ``protected_nets`` and
``net_impedance`` (#521), ``pour_served_pads`` (#678),
``same_net_pad_clearance`` (#581), ``class_clearance_relaxed`` (#1160),
``pad_override_min_clearance`` (#1369) and ``saved_severities`` (#856). The CLI keeps it in the sibling ``.kicad_pro``
under ``"kicad_routing_tools"``, and every chain step carries the project.

WHY A SECOND HOME. KiCad's own project save rewrites the ``.kicad_pro`` from
the copy it loaded when the board was opened. Measured on KiCad 10.0.0: a
board loaded, its project then given a ``kicad_routing_tools`` record on disk,
then saved through pcbnew, came back WITHOUT the record. So whatever the GUI
writes into the project mid-session -- the origin, protected nets, impedance
specs, pour promises -- is gone after the user's next save, and a later CLI
step baselines on what is left: the ad7f24de silence, reached from the GUI.

The board itself is the one in-memory home pcbnew saves. ``BOARD::
SetProperties`` persists into the ``.kicad_pcb`` as a top-level
``(property "kicad_routing_tools" "<json>")`` (written after ``(setup ...)``),
reloads with the board, and survives the project rewrite -- measured the same
way. The project's text variables, the other candidate, live on a PROJECT
object SWIG does not wrap. So the GUI keeps the whole namespace on the board
as well as in the project, and every reader takes the newer of the two.

WHICH COPY WINS. Each copy carries ``records_rev``. A writer starts from the
winning copy and stores it with ``records_rev + 1``; the newer copy wins, a
tie goes to the project (the two then hold the same content), and a project
with no namespace at all loses to a board copy that has one. So: after a GUI
step and a KiCad save, the board's copy wins (the project was rewritten from
what KiCad loaded); after a CLI step the project wins (its writer bumped past
the board copy the output ``.kicad_pcb`` carried from its input); a project
nothing ever wrote a revision into reads as before.

Nothing here imports pcbnew. The board functions take any object with
``GetFileName`` / ``GetProperties`` / ``SetProperties``, and fall back to an
in-memory copy for one without the property calls.
"""
import json
import os
import re

NAMESPACE = "kicad_routing_tools"
REV_KEY = "records_rev"
BOARD_PROPERTY = "kicad_routing_tools"

# A depth-1 board property: KiCad 10 indents it one tab, KiCad 6-9 two spaces.
# Footprint properties sit deeper and never carry this name.
_PROP_RE = re.compile(
    r'^(?:\t| {2})\(property\s+"' + re.escape(BOARD_PROPERTY)
    + r'"\s+"((?:[^"\\]|\\.)*)"\s*\)', re.M)


def _stamp(path):
    try:
        st = os.stat(path)
    except (OSError, ValueError, TypeError):
        return None
    return (st.st_mtime_ns, st.st_size, st.st_ino)


def kicad_unescape(s: str) -> str:
    """A KiCad s-expression string's content: ``\\"`` and ``\\\\`` as KiCad
    writes them, ``\\n`` / ``\\r`` / ``\\t`` as the characters."""
    out, i = [], 0
    while i < len(s):
        c = s[i]
        if c == '\\' and i + 1 < len(s):
            n = s[i + 1]
            out.append({'n': '\n', 'r': '\r', 't': '\t'}.get(n, n))
            i += 2
        else:
            out.append(c)
            i += 1
    return ''.join(out)


def kicad_escape(s: str) -> str:
    """The inverse of :func:`kicad_unescape`, as KiCad quotes a string."""
    return (s.replace('\\', '\\\\').replace('"', '\\"')
            .replace('\n', '\\n').replace('\r', '\\r'))


def board_property_sexpr(ns: dict) -> str:
    """The ``(property ...)`` node KiCad writes for ``ns`` -- for fixtures
    that must not need pcbnew to put a board copy on a file."""
    return (f'(property "{BOARD_PROPERTY}" '
            f'"{kicad_escape(json.dumps(ns, sort_keys=True, separators=(",", ":")))}")')


def revision(ns) -> int:
    try:
        return int((ns or {}).get(REV_KEY) or 0)
    except (TypeError, ValueError):
        return 0


def _pick(ns_pro, ns_board):
    """The winning copy (see the module docstring)."""
    ns_pro = ns_pro if isinstance(ns_pro, dict) else {}
    if not isinstance(ns_board, dict) or not ns_board:
        return ns_pro
    rp, rb = revision(ns_pro), revision(ns_board)
    if rb > rp or (rb == rp and not ns_pro):
        return ns_board
    return ns_pro


_BOARD_CACHE: dict = {}
_PRO_CACHE: dict = {}


def board_namespace(pcb_path: str):
    """The namespace stored on the board FILE (its top-level property), or
    None. Cached by the file's stamp: the readers ask per obstacle-map build
    and per record, and the .kicad_pcb is the large file."""
    if not pcb_path:
        return None
    stamp = _stamp(pcb_path)
    hit = _BOARD_CACHE.get(pcb_path)
    if hit is not None and hit[0] == stamp:
        return hit[1]
    ns = None
    if stamp is not None:
        try:
            with open(pcb_path, 'r', encoding='utf-8', errors='replace') as f:
                text = f.read()
            m = _PROP_RE.search(text)
            if m:
                v = json.loads(kicad_unescape(m.group(1)))
                ns = v if isinstance(v, dict) else None
        except (OSError, ValueError):
            ns = None
    _BOARD_CACHE[pcb_path] = (stamp, ns)
    return ns


def project_namespace(pro_path: str) -> dict:
    """The namespace in a ``.kicad_pro`` (``{}`` when absent/unreadable),
    cached by the file's stamp."""
    if not pro_path:
        return {}
    stamp = _stamp(pro_path)
    hit = _PRO_CACHE.get(pro_path)
    if hit is not None and hit[0] == stamp:
        return hit[1]
    ns = {}
    if stamp is not None:
        try:
            with open(pro_path, 'r', encoding='utf-8') as f:
                v = (json.load(f) or {}).get(NAMESPACE)
            ns = v if isinstance(v, dict) else {}
        except (OSError, ValueError, AttributeError):
            ns = {}
    _PRO_CACHE[pro_path] = (stamp, ns)
    return ns


def _paths(path: str):
    base = os.path.splitext(path)[0]
    return base + '.kicad_pcb', base + '.kicad_pro'


def effective_namespace(path: str) -> dict:
    """THE reader: the records for the board at ``path`` (a ``.kicad_pcb`` or
    ``.kicad_pro`` path) -- the newer of the project's copy and the board
    file's. A copy: mutating it changes nothing on disk."""
    if not path:
        return {}
    pcb, pro = _paths(path)
    return json.loads(json.dumps(_pick(project_namespace(pro),
                                       board_namespace(pcb))))


def load_project(pro_path: str) -> dict:
    """``json.load`` of a ``.kicad_pro`` whose namespace is the EFFECTIVE one
    (the board file's copy when that is newer), for a writer to modify and
    store with :func:`stamp_revision`. Raises like ``json.load``."""
    with open(pro_path, 'r', encoding='utf-8') as f:
        proj = json.load(f)
    pcb, _pro = _paths(pro_path)
    mine = proj.get(NAMESPACE) if isinstance(proj.get(NAMESPACE), dict) else {}
    win = _pick(mine, board_namespace(pcb))
    if win is not mine:
        proj[NAMESPACE] = json.loads(json.dumps(win))
    return proj


def stamp_revision(proj: dict) -> None:
    """Before a writer stores ``proj``: the namespace's revision becomes one
    past the copy it started from, so it outranks the board copy the file
    beside it may still carry. A project with no namespace is left alone."""
    ns = proj.get(NAMESPACE)
    if isinstance(ns, dict) and ns:
        ns[REV_KEY] = revision(ns) + 1


# --- the LIVE board (the GUI) -------------------------------------------------

#: Board copies for an object with no property calls (a test stand-in, or a
#: pcbnew without SetProperties), by its file or identity.
_FALLBACK: dict = {}


def _board_key(board) -> str:
    try:
        path = board.GetFileName() or ""
    except Exception:                                           # noqa: BLE001
        path = ""
    return os.path.normcase(os.path.abspath(path)) if path else f"id:{id(board)}"


def live_board_namespace(board):
    """The namespace on the LIVE board (its in-memory property), or None."""
    try:
        props = board.GetProperties()
    except Exception:                                           # noqa: BLE001
        return _FALLBACK.get(_board_key(board))
    try:
        for k, v in props.items():
            if str(k) == BOARD_PROPERTY:
                ns = json.loads(str(v))
                return ns if isinstance(ns, dict) else None
    except Exception:                                           # noqa: BLE001
        return None
    return None


def set_live_board_namespace(board, ns: dict) -> None:
    """Store ``ns`` on the live board, where pcbnew's next save persists it."""
    text = json.dumps(ns, sort_keys=True, separators=(',', ':'))
    try:
        props = board.GetProperties()
        props[BOARD_PROPERTY] = text
        board.SetProperties(props)
    except Exception:                                           # noqa: BLE001
        _FALLBACK[_board_key(board)] = json.loads(text)


def _board_pro(board) -> str:
    try:
        path = board.GetFileName() or ""
    except Exception:                                           # noqa: BLE001
        path = ""
    return os.path.splitext(path)[0] + '.kicad_pro' if path else ""


def live_namespace(board) -> dict:
    """The records for a LIVE board: the newer of its project's copy on disk
    and its in-memory copy (which is newer than the board file's: it is what
    the next save writes). A copy."""
    return json.loads(json.dumps(_pick(project_namespace(_board_pro(board)),
                                       live_board_namespace(board))))


def write_live_namespace(board, ns: dict) -> None:
    """Store ``ns`` (one revision past what it was read from) on the live
    board, and into its project file when it has one -- where an in-session
    CLI step or a later read sees it before any save."""
    ns = json.loads(json.dumps(ns))
    ns[REV_KEY] = revision(ns) + 1
    pro = _board_pro(board)
    if pro and os.path.isfile(pro):
        try:
            with open(pro, 'r', encoding='utf-8') as f:
                proj = json.load(f)
            proj[NAMESPACE] = ns
            tmp = pro + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(proj, f, indent=2)
                f.write('\n')
            os.replace(tmp, pro)
        except Exception:                                       # noqa: BLE001
            pass
    set_live_board_namespace(board, ns)


def update_live_namespace(board, update) -> bool:
    """Apply ``update(ns) -> changed`` to the live board's records and store
    them when it changed anything. The GUI's one write path."""
    ns = live_namespace(board)
    if not update(ns):
        return False
    write_live_namespace(board, ns)
    return True


def mirror_project_to_live_board(board) -> bool:
    """After a FILE writer (fix_project_for_output) wrote the live board's
    project in-process: put what it wrote on the board too, or the user's
    next save drops it. True when the board copy changed."""
    ns = live_namespace(board)
    if not ns or ns == (live_board_namespace(board) or {}):
        return False
    set_live_board_namespace(board, ns)
    return True
