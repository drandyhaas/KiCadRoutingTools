"""Does KiCad load this `.kicad_dru`? KiCad's own answer (#1852).

KiCad drops a whole rules file it cannot load: it then enforces none of the
file's rules and says nothing (exit 0, the JSON report and the console the
same as a run with no rules file). `design_rules.dru_refusals` is a
hand-written model of what makes it do that (#1690), and KiCad's grammar is
irregular enough that the model misses forms (#1840 measured 31). So when
kicad-cli is installed, KiCad is asked:

- a STUB board is written: one footprint and the real board's layer table
  (every layer it declares, with its user name, so a rule naming a layer by
  either name resolves as it does on the board), in a temporary directory;
- beside it, the rules file's text plus one probe rule (`probe_rule`: an
  `assertion` at warning severity that the stub's one footprint fails);
- one `kicad-cli pcb drc` of the stub. "Loaded" means a row names the probe.

That costs one kicad-cli start (about 1.5 s) whatever the real board's size,
and needs nothing of the board but its layer table: whether KiCad loads a
rules file is decided by the file's text and the layer names it resolves
against. The answer is cached by the rules text, the layer table and the
kicad-cli version (`cache_dir`), on disk and per process, and only a
definite answer is cached.

**Disclosure only.** What this says changes what a step SAYS about the file
("KiCad does not load this file: none of its rules is enforced"), never what
it reads or routes: copper may not depend on whether kicad-cli is installed.
The rules are read from the file as before, whatever KiCad's answer.
"""
import hashlib
import json
import os
import re
import shutil
import subprocess
import tempfile
from typing import Dict, List, Optional, Sequence, Tuple

#: Bumped whenever the stub board or the probe changes, so an answer cached
#: for the old stub is not read back for the new one.
STUB_FORMAT = 1

#: The probe's rule name: the row KiCad writes for it names it
#: (`(rule 'NAME' ...`). check_complete's staged-copy probe (#1810) is the
#: same rule under the same name.
LOAD_PROBE = 'check_complete load probe'

#: The reference of the stub's one footprint, which the probe names.
STUB_REF = 'KRTLOADPROBE1'

#: How long one stub DRC may take, in seconds.
TIMEOUT = 120

_MEMO: Dict[str, dict] = {}


def probe_rule(ref: str, taken: Sequence[str] = ()) -> Tuple[str, str]:
    """``(name, rule text)``: a `.kicad_dru` rule KiCad reports once on a
    board whose footprint ``ref`` is the only one with that reference,
    whenever it loads the rules file the rule stands in, and never when it
    drops the file -- an `assertion`, at warning severity, that the footprint
    does not carry that reference. ``taken`` is the rule names the file
    already declares; the probe takes a name none of them has.

    A warning row is never counted, and KiCad reports every assertion rule
    that binds an item, so the probe takes nothing from an assertion of the
    file's own on the same footprint (measured on kicad-cli 10.0.6 and
    10.0.7: the report with the probe is the report without it plus the
    probe's one row, also on a project whose `rule_severities` ignores
    `assertion_failure`)."""
    name, n = LOAD_PROBE, 1
    while name in taken:
        n += 1
        name = f'{LOAD_PROBE} {n}'
    return name, (f'(rule "{name}" (severity warning)\n'
                  f'  (constraint assertion "A.Reference != \'{ref}\'")\n'
                  f'  (condition "A.Type == \'Footprint\'"))')


def rule_names(text: str) -> List[str]:
    """Every rule name ``text`` declares, read off the text (a file KiCad
    refuses may not parse)."""
    return re.findall(r'\(\s*rule\s+"((?:[^"\\]|\\.)*)"', text or '') + \
        re.findall(r'\(\s*rule\s+([^\s"()]+)', text or '')


# ---------------------------------------------------------------- the stub

#: KiCad 9/10's layer ordinals by canonical name (the `(layers ...)` table's
#: numbers; KiCad 9 renumbered them, and the stub is written in its format).
_FIXED_ORDINALS = {
    'F.Cu': 0, 'F.Mask': 1, 'B.Cu': 2, 'B.Mask': 3, 'F.SilkS': 5, 'B.SilkS': 7,
    'F.Adhes': 9, 'B.Adhes': 11, 'F.Paste': 13, 'B.Paste': 15,
    'Dwgs.User': 17, 'Cmts.User': 19, 'Eco1.User': 21, 'Eco2.User': 23,
    'Edge.Cuts': 25, 'Margin': 27, 'B.CrtYd': 29, 'F.CrtYd': 31,
    'B.Fab': 33, 'F.Fab': 35,
}
#: The stub's format: KiCad 9.0's, which kicad-cli 9 and 10 both read.
_STUB_VERSION = 20241229
_ENTRY = re.compile(r'\(\s*(\d+)\s+"((?:[^"\\]|\\.)*)"\s+([\w.]+)'
                    r'(?:\s+"((?:[^"\\]|\\.)*)")?\s*\)')
_ENTRY_BARE = re.compile(r'\(\s*(\d+)\s+([^\s"()]+)\s+([\w.]+)\s*\)')


def _ordinal(name: str) -> Optional[int]:
    if name in _FIXED_ORDINALS:
        return _FIXED_ORDINALS[name]
    m = re.fullmatch(r'In(\d+)\.Cu', name)
    if m and 1 <= int(m.group(1)) <= 30:
        return 2 * int(m.group(1)) + 2
    m = re.fullmatch(r'User\.(\d+)', name)
    if m and int(m.group(1)) >= 1:
        return 37 + 2 * int(m.group(1))
    return None


def board_layer_table(board_path: Optional[str]) -> List[Tuple[str, str, Optional[str]]]:
    """The board's `(layers ...)` table as ``[(canonical name, type, user
    name or None)]``, in file order, read off the head of the file; [] when
    there is no board or no table. A layer KiCad 9/10 has no ordinal for
    (a KiCad 5 `Rescue`) is left out."""
    if not board_path or not os.path.isfile(board_path):
        return []
    try:
        with open(board_path, encoding='utf-8', errors='replace') as fh:
            head = fh.read(1 << 20)
    except OSError:
        return []
    m = re.search(r'\(layers\s*\n', head) or re.search(r'\(layers\b', head)
    if not m:
        return []
    from kicad_parser import find_matching_paren
    block = head[m.end():find_matching_paren(head, m.start())]
    out = []
    for rx in (_ENTRY, _ENTRY_BARE):
        for e in rx.finditer(block):
            name = e.group(2)
            if _ordinal(name) is not None and all(name != x[0] for x in out):
                user = e.group(4) if rx is _ENTRY else None
                out.append((name, e.group(3), user if user else None))
    return out


def stub_layers(board_path: Optional[str] = None,
                copper_layers: Optional[Sequence[str]] = None
                ) -> List[Tuple[int, str, str, Optional[str]]]:
    """``[(ordinal, canonical name, type, user name)]`` for the stub: the
    board's own table when it has one, else ``copper_layers`` (F.Cu and B.Cu
    when that is empty too) and the layers the stub's footprint uses."""
    table = board_layer_table(board_path)
    if not table:
        table = [(n, 'signal', None) for n in (copper_layers or ('F.Cu', 'B.Cu'))
                 if _ordinal(n) is not None]
    have = {t[0] for t in table}
    for name, kind in (('F.Cu', 'signal'), ('B.Cu', 'signal'), ('F.SilkS', 'user'),
                       ('F.Fab', 'user'), ('Edge.Cuts', 'user')):
        if name not in have:
            table.append((name, kind, None))
    return [(_ordinal(n), n, t, u) for n, t, u in table]


def _quote(s: str) -> str:
    return '"' + s.replace('\\', '\\\\').replace('"', '\\"') + '"'


def stub_board(layers: Sequence[Tuple[int, str, str, Optional[str]]]) -> str:
    """The stub board's text: ``layers`` and one footprint, `STUB_REF`."""
    table = '\n'.join(f'\t\t({o} {_quote(n)} {t}' + (f' {_quote(u)}' if u else '') + ')'
                      for o, n, t, u in layers)
    return (f'(kicad_pcb\n\t(version {_STUB_VERSION})\n\t(generator "pcbnew")\n'
            f'\t(generator_version "9.0")\n\t(general\n\t\t(thickness 1.6)\n\t)\n'
            f'\t(paper "A4")\n\t(layers\n{table}\n\t)\n'
            f'\t(setup\n\t\t(pad_to_mask_clearance 0)\n\t)\n\t(net 0 "")\n'
            f'\t(footprint ""\n\t\t(layer "F.Cu")\n'
            f'\t\t(uuid "4b52545f-0000-4000-8000-000000000001")\n\t\t(at 0 0)\n'
            f'\t\t(property "Reference" "{STUB_REF}"\n\t\t\t(at 0 0 0)\n'
            f'\t\t\t(layer "F.SilkS")\n'
            f'\t\t\t(uuid "4b52545f-0000-4000-8000-000000000002")\n'
            f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n'
            f'\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n\t)\n)\n')


# ---------------------------------------------------------------- kicad-cli

def _find_cli() -> Optional[str]:
    """kicad-cli, found the way every other kicad-cli step finds it."""
    try:
        from kicad_oracle import find_kicad_cli
        return find_kicad_cli(warn=False)
    except Exception:                                           # noqa: BLE001
        return None


def cache_dir() -> str:
    """Where answers are kept: ``$XDG_CACHE_HOME/krt_dru_load``, else
    ``~/.cache/krt_dru_load`` -- outside any checkout, shared by every
    process of the user. Each answer is its own file, written to a
    temporary name and renamed into place, so two processes asking at once
    each read a whole answer (and write the same one)."""
    return os.path.join(os.environ.get('XDG_CACHE_HOME') or os.path.expanduser('~/.cache'),
                        'krt_dru_load')


def _read_json(path: str) -> Optional[dict]:
    try:
        with open(path, encoding='utf-8') as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else None
    except (OSError, ValueError):
        return None


def _write_json(path: str, data: dict) -> None:
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix='.tmp-', suffix='.json')
        with os.fdopen(fd, 'w', encoding='utf-8') as fh:
            json.dump(data, fh)
        os.replace(tmp, path)
    except OSError:
        pass                     # an answer not cached is asked again next time


def cli_version(cli: str) -> Optional[str]:
    """`kicad-cli version`, remembered on disk by the binary's path, size and
    modification time, so a cached answer costs no kicad-cli start."""
    try:
        st = os.stat(cli)
        ident = f'{os.path.realpath(cli)}|{st.st_size}|{st.st_mtime_ns}'
    except OSError:
        return None
    path = os.path.join(cache_dir(), 'cli-' + hashlib.sha256(ident.encode()).hexdigest()[:32]
                        + '.json')
    got = _read_json(path)
    if got and got.get('ident') == ident and got.get('version'):
        return got['version']
    try:
        from kicad_oracle import kicad_cli_version
        version = kicad_cli_version(cli)
    except Exception:                                           # noqa: BLE001
        version = None
    if version:
        _write_json(path, {'ident': ident, 'version': version})
    return version


def kicad_loads_dru(text: str, board_path: Optional[str] = None,
                    copper_layers: Optional[Sequence[str]] = None,
                    timeout: float = TIMEOUT) -> dict:
    """KiCad's answer to "does it load this rules file", on the stub board
    with ``board_path``'s layer table (or ``copper_layers``).

    -> ``{'loaded': True | False, 'kicad_version': str, 'cached': bool}``,
    or ``{'loaded': None, 'why': str}`` when KiCad cannot be asked (no
    kicad-cli, a timeout, no report). Never raises."""
    cli = _find_cli()
    if not cli:
        return {'loaded': None, 'why': 'kicad-cli not found (set KICAD_CLI)'}
    version = cli_version(cli)
    if not version:
        return {'loaded': None, 'why': f'{cli} gave no version'}
    layers = stub_layers(board_path, copper_layers)
    key = hashlib.sha256(json.dumps([STUB_FORMAT, version, text, layers]).encode()
                         ).hexdigest()[:40]
    if key in _MEMO:
        return dict(_MEMO[key], cached=True)
    path = os.path.join(cache_dir(), key + '.json')
    got = _read_json(path)
    if got and isinstance(got.get('loaded'), bool) and got.get('kicad_version') == version:
        _MEMO[key] = got
        return dict(got, cached=True)
    name, rule = probe_rule(STUB_REF, rule_names(text))
    tmp = tempfile.mkdtemp(prefix='krt_dru_load.')
    try:
        board = os.path.join(tmp, 'stub.kicad_pcb')
        with open(board, 'w', encoding='utf-8') as fh:
            fh.write(stub_board(layers))
        with open(os.path.join(tmp, 'stub.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(text.rstrip('\n') + '\n' + rule + '\n')
        out = os.path.join(tmp, 'drc.json')
        try:
            r = subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--units', 'mm',
                                '--severity-all', '--output', out, board],
                               capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            return {'loaded': None, 'why': f'kicad-cli timed out after {timeout:g} s'}
        report = _read_json(out)
        if report is None:
            return {'loaded': None,
                    'why': f'kicad-cli wrote no report of the stub board (rc={r.returncode}: '
                           f'{(r.stderr or "").strip()[:200]})'}
    except Exception as exc:                                    # noqa: BLE001
        return {'loaded': None, 'why': f'{type(exc).__name__}: {exc}'}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    tag = f"(rule '{name}'"
    loaded = any(tag in (v.get('description') or '') for v in report.get('violations') or ())
    answer = {'loaded': loaded, 'kicad_version': version}
    _MEMO[key] = answer
    _write_json(path, answer)
    return dict(answer, cached=False)


def refusals(text: Optional[str], model: Sequence[dict], board_path: Optional[str] = None,
             copper_layers: Optional[Sequence[str]] = None) -> List[dict]:
    """Why KiCad enforces no rule of the rules file ``text``, [] when it
    loads it: KiCad's answer (`kicad_loads_dru`) when it can be asked, and
    ``model`` -- `design_rules.dru_refusals` of the same text -- otherwise.

    The model stays the source of WHY: when KiCad drops the file and the
    model names a cause, its rows are kept (each tagged ``kicad_version``);
    when it names none, one row says KiCad drops the file for a reason the
    model does not know. When KiCad loads the file, a refusal the model
    named is not one KiCad makes, and nothing is returned."""
    if text is None:
        return list(model)
    ans = kicad_loads_dru(text, board_path, copper_layers)
    if ans.get('loaded') is None:
        return list(model)
    if ans['loaded']:
        return []
    version = ans['kicad_version']
    if model:
        return [dict(r, kicad_version=version) for r in model]
    from design_rules import _KICAD_REFUSES
    return [{'rule': '', 'kicad_version': version,
             'reason': (f"{_KICAD_REFUSES} this file: kicad-cli {version} enforced no rule of "
                        f"it on a stub board with this board's layers (a probe rule appended "
                        f"to it did not fire), for a reason this tool's model of what KiCad "
                        f"refuses does not name")}]
