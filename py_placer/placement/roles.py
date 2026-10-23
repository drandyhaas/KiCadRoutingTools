"""What a part IS, read off the board -- role knowledge both sides share.

`py_tools/board_context.py` prints a role per part, and the decap owner
predicate (`groups.supply_owner_evidence`) asks the same question of a part
with fewer than four copper pads: is this a regulator? The table lives here,
in py_placer (which never imports py_tools), so the sheet and the predicate
read ONE table.

Deliberately NOT part of `part_class.classify_part`: `grade_body_overlap.
_waiver_for` branches on that classifier's `.name` against its marker and
edge tables, so a role vocabulary there would silently change waivers on the
body-overlap channel. These read it, never extend it.

The board carries no datasheet, description or 3D-model field (`PCBData` has
none), so a role here is an INFERENCE from what the board does carry --
footprint name, reference prefix, `pintype` / `pinfunction`, net names -- and
each one names its source. `unknown` is a first-class answer: "I looked and
could not tell" is printed apart from a guess.
"""
from __future__ import annotations

import re
from typing import Dict, List, Optional

#: (role, keys) by footprint-name substring, matched on `_norm`ed text (lower
#: case, separators dropped) -- so `TO-92_HandSolder`, `TO92` and `to_92`
#: are one name, and KiCad's own spellings (`TO-92`, `SOT-223-3_TabPin2`)
#: match the keys `to92` and `sot223`.
_ROLE_FP = (
    ('crystal', ('xtal', 'crystal', 'resonator', 'oscillator')),
    ('usb receptacle', ('usbmicro', 'usbc', 'usbb', 'usba', 'uicro')),
    ('pin header', ('pinheader', 'hn1x', 'socketstrip', 'header')),
    ('npn/pnp transistor', ('sot23', 'sot323', 'to92')),
    ('regulator', ('sot89', 'sot223', 'to252', 'dpak')),
)

#: Connector rows above. A hit for one of these found only in a footprint's
#: LIBRARY nickname does not outrank an IC's reference prefix: a board's own
#: library groups parts by where they are used, and fusb302_pd_dev files its
#: buck converter U2 as `usb-c:TI-RNX0012A-VQFN-HR`. Over the 478 corpus
#: boards that rule changes that one part; reading the library nickname not
#: at all would change 19 others for the worse (a crystal's `Crystal-FP:`
#: library, `Mlab_Pin_Headers:Straight_2x03`).
_CONNECTOR_FP_ROLES = ('usb receptacle', 'pin header')

#: The two package rows above hold parts of BOTH kinds -- a 78L05 and a
#: BC237 are both TO-92, a BCX56 and an ME6210 are both SOT-89 -- so a hit
#: there is read through the part's own pins first (`_package_role`).
_SHARED_PACKAGE_ROLES = ('npn/pnp transistor', 'regulator')

#: Pin functions that name a transistor terminal (bipolar and FET). A part
#: whose pins are named these is a transistor whatever its package says.
_TRANSISTOR_PINS = frozenset({'B', 'C', 'E', 'G', 'D', 'S'})

#: role by reference prefix, used only when nothing stronger fired.
_ROLE_PREFIX = {
    'C': 'capacitor', 'R': 'resistor', 'L': 'inductor', 'D': 'diode',
    'Q': 'transistor', 'U': 'integrated circuit', 'Y': 'crystal',
    'X': 'crystal', 'J': 'connector', 'SW': 'switch', 'FB': 'ferrite bead',
    'TP': 'test point', 'FID': 'fiducial', 'LED': 'led',
}


def _norm(text: str) -> str:
    """Lower case with the separators KiCad spells inconsistently dropped."""
    return re.sub(r'[\s_\-.]', '', (text or '').lower())


def _prefix(ref: str) -> str:
    out = ''
    for ch in ref:
        if ch.isalpha() or ch in '#*':
            out += ch
        else:
            break
    return out.upper()


def pin_function_name(pad) -> str:
    """A pad's pin FUNCTION with KiCad's `_<pad number>` suffix removed,
    upper-cased: `PGND_1` on pad 1 -> `PGND`, `D+_A6` on pad A6 -> `D+`,
    `I/O1_6` on pad 6 -> `I/O1`. KiCad writes the suffix when it
    disambiguates pin names, so `VIN_2` and `VIN` are the same function.
    '' when the pad carries none."""
    fn = str(getattr(pad, 'pinfunction', '') or '').strip()
    num = str(getattr(pad, 'pad_number', '') or '')
    if num and fn.endswith('_' + num) and len(fn) > len(num) + 1:
        fn = fn[:-(len(num) + 1)]
    return fn.upper()


def _copper_pads(fp) -> List:
    """The pads that carry copper, aperture-only windows excluded."""
    from kicad_parser import non_aperture_pads
    return [p for p in non_aperture_pads(fp)
            if any(str(l).endswith('.Cu') for l in (p.layers or ()))]


def power_out_pads(fp) -> List:
    """The copper pads typed `power_out` (a `+no_connect` pad excluded): the
    pins the symbol says SOURCE a rail."""
    out = []
    for p in _copper_pads(fp):
        toks = str(getattr(p, 'pintype', '') or '').split('+')
        if 'power_out' in toks and 'no_connect' not in toks:
            out.append(p)
    return out


#: The reference-prefix roles each shared-package reading is compatible
#: with: a regulator is an IC, a transistor is a transistor.
_PACKAGE_PREFIX_OK = {'npn/pnp transistor': ('transistor',),
                      'regulator': ('integrated circuit',)}


def _package_role(fp, role: str, hit: str,
                  pref: str) -> Optional[Dict[str, object]]:
    """A shared-package hit read through the pins (#1249), or None to fall
    through to the reference prefix.

    * a `power_out` pad -> `regulator`: a transistor never sources a rail
      (complex_hierarchy U101, a 78L05 in `TO-92_HandSolder`);
    * pin functions naming only transistor terminals -> `npn/pnp
      transistor`;
    * a reference prefix naming a DIFFERENT kind of part -> fall through:
      a package both kinds come in does not outrank it (glasgow D18, a
      BAT54CW dual diode in SOT-323; a `U` in SOT-23-6 is an IC, not a
      transistor);
    * pin functions that name NO transistor terminal on a transistor-first
      package -> fall through: the pins say it is something else and the
      package alone cannot say what (kit-dev LV101, a DS1818 reset
      supervisor in TO-92 with pins `Rst` / `In` / `Gbd`);
    * otherwise the package's usual reading, labelled as such.
    """
    ev = [f'footprint name contains {hit!r}']
    po = power_out_pads(fp)
    if po:
        return {'role': 'regulator', 'source': 'footprint',
                'evidence': ev + [f'pad {po[0].pad_number} typed power_out']}
    names = {pin_function_name(p) for p in _copper_pads(fp)} - {''}
    if names and names <= _TRANSISTOR_PINS:
        return {'role': 'npn/pnp transistor', 'source': 'footprint',
                'evidence': ev + ['pin functions '
                                  + '/'.join(sorted(names))]}
    by_prefix = _ROLE_PREFIX.get(pref)
    if by_prefix is not None and by_prefix not in _PACKAGE_PREFIX_OK[role]:
        return None
    if names and not names & _TRANSISTOR_PINS and role != 'regulator':
        return None
    return {'role': role, 'source': 'footprint', 'evidence': ev}


def infer_role(fp, ref: str, cls_name: Optional[str]) -> Dict[str, object]:
    """`{'role', 'source', 'evidence'}` -- or role `'unknown'`.

    Ordered strongest-first, and each rung names itself. `unknown` is returned
    rather than a prefix guess when the reference carries no convention this
    table knows: "I looked and could not tell" is a different statement from a
    low-confidence guess, and a reader who cannot tell them apart will trust
    the guess.
    """
    name = (getattr(fp, 'footprint_name', '') or '').lower()
    value = (getattr(fp, 'value', '') or '').strip()
    pref = _prefix(ref)

    if cls_name in ('mount_hole', 'fiducial', 'testpoint', 'edge_receptacle',
                    'edge_actuator'):
        return {'role': cls_name.replace('_', ' '), 'source': 'part_class',
                'evidence': [f'placement.part_class says {cls_name}']}
    flat = _norm(name)
    own = _norm(name.split(':', 1)[1]) if ':' in name else flat
    for role, keys in _ROLE_FP:
        hit = next((k for k in keys if k in flat), None)
        if not hit:
            continue
        if (role in _CONNECTOR_FP_ROLES and hit not in own
                and _ROLE_PREFIX.get(pref) == 'integrated circuit'):
            continue
        if role in _SHARED_PACKAGE_ROLES:
            got = _package_role(fp, role, hit, pref)
            if got is None:
                break
            return got
        return {'role': role, 'source': 'footprint',
                'evidence': [f'footprint name contains {hit!r}']}
    # #1176: by the ITEM name, as `part_class` reads it -- a horizontal
    # D-sub carries `MountingHolesOffset` in its name.
    from .part_class import is_mount_hole_name
    if is_mount_hole_name(name):
        return {'role': 'mounting hole', 'source': 'footprint',
                'evidence': ["footprint item name says 'MountingHole'"]}
    if pref in _ROLE_PREFIX:
        ev = [f'reference prefix {pref!r} (convention only)']
        if value:
            ev.append(f'value {value!r}')
        return {'role': _ROLE_PREFIX[pref], 'source': 'ref_prefix',
                'evidence': ev}
    return {'role': 'unknown', 'source': 'looked',
            'evidence': [f'footprint {name!r}, value {value!r}, prefix '
                         f'{pref!r} match no rule here']}
