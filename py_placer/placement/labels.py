"""
beautify_labels engine (#481): tidy reference-designator silkscreen.

Places every in-scope Reference label at a uniform font size and orientation,
near its own part, without overlapping pads, drill holes, other labels, or the
board edge. Copper is never touched; a label that cannot be placed legally is
reported `unchanged` and its bytes are left exactly as they were.

Obstacle model (static world, v1): pad rects + NPTH hole circles from
`legality.build_part_pads` (through-hole pads block BOTH silk sides), the real
Edge.Cuts outline via `legality.BoardOutlineGate`, and other visible labels --
greedy-sequential in sorted-ref order, so each placed label becomes an
obstacle for the next. Neighbor courtyards are a SOFT penalty only. v1 does
NOT parse silkscreen graphics (no reader exists in either parse path;
hard-blocking on all drawn silk degrades to mostly-unplaceable -- the
courtyard penalty plus hard pad avoidance covers drawn bodies in practice).

Candidate search: canonical anchors (above -> below -> left -> right of the
part's own courtyard, offset by `anchor_gap`), sliding along each side in
0.25 mm steps, growing the gap ring-by-ring up to `max_distance`. Degradation
ladder: target size at the preferred orientation -> rotated 90 deg -> shrink
by `size_step` down to `min_size` (thickness ratio-scaled, floor 0.1),
preferred-then-rotated at each size -> give up (`unchanged`, with a reason).

Both front-ends use the same `label_box` width ESTIMATOR, never pcbnew's
exact bbox, so CLI and GUI make identical decisions.
"""
from __future__ import annotations

import fnmatch
import math
from dataclasses import asdict, dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from kicad_parser import _global_to_local, local_to_global
from placement.legality import (BoardOutlineGate, build_part_pads,
                                footprint_side, rect_gap, rect_overlap_area,
                                rotate_local_bounds)
from placement.parser import courtyard_for_side, extract_courtyard_sides

EPS = 1e-6

# --- label geometry ---------------------------------------------------------

# Per-character advance factor for the width estimate, CALIBRATED AS AN
# OVERESTIMATE against pcbnew's real text bounding boxes (KiCad 10 stroke
# font, splitflap + ulx3s): worst measured (bbox_w - thickness)/(len * size)
# is 1.1603 (a ref containing '&'); typical uppercase/digit refs measure
# 1.05. The plan's initial 0.95 was measured as an UNDER-estimate and would
# have let placed labels overlap. Worst case of overestimating is a spurious
# `unchanged`, never an overlap.
CHAR_ADVANCE = 1.2
# Ink height factor: cap height IS the font size for the uppercase+digit
# glyphs a reference uses; the pcbnew BBOX height factor measures ~1.55-1.8x
# but that is the font LINE box (overbar + descender allowance), not drawn
# ink. size * 1.1 + thickness covers the stroke with margin.
HEIGHT_FACTOR = 1.1
THICKNESS_FLOOR = 0.1   # mm; ratio-scaling a shrunk label never goes below
SLIDE_STEP = 0.25       # mm along the courtyard side
RING_STEP = 0.25        # mm of anchor-gap growth per ring
ANCHORS = ('above', 'below', 'left', 'right')
# Soft weight (mm of score per mm^2) for candidate boxes over a NEIGHBOR
# courtyard -- drawn bodies usually live inside it, so prefer clear space
# without hard-failing boards packed too tight for any clear anchor.
COURTYARD_PENALTY = 2.0


def fold_upright(angle: float) -> float:
    """Keep-upright fold into (-90, 90], matching PCB_FIELD.GetDrawRotation().

    Probed (KiCad 10, splitflap + ulx3s, all 28 distinct rotation combos):
    GetDrawRotation() == this fold of the STORED angle, independent of the
    footprint's rotation. 180 -> 0, 270 -> 90.
    """
    a = angle % 360.0
    while a > 90.0:
        a -= 180.0
    while a <= -90.0:
        a += 180.0
    return a


def label_world_angle(fp, label) -> float:
    """The angle the label is actually DRAWN at on the board.

    PROBE VERDICT (KiCad 10 pcbnew vs file bytes, splitflap_driver + ulx3s,
    every distinct (fp_rotation, side, stored_angle) combo -- 28 rows): the
    Reference node's stored `(at ... ANGLE)` is the label's ABSOLUTE board
    angle, NOT a footprint-relative one. `GetTextAngle()` returns the file
    value unchanged (U8: fp_rot 90 + stored 180 -> GetDrawRotation() 0 ==
    fold(180), not fold(270)), and `GetDrawRotation()` is the keep-upright
    fold of that stored value with the footprint rotation playing no part.
    So: world angle = fold_upright(stored angle); `fp` is accepted only so a
    flipped verdict stays a two-line fix in this helper pair.
    """
    return fold_upright(label.rotation)


def label_file_angle(world_angle: float, fp=None) -> float:
    """Inverse of `label_world_angle` for writing: absolute storage means the
    world angle IS the file angle (probe verdict above). A relative-storage
    verdict would make this `(world_angle - fp.rotation) % 360`."""
    return world_angle % 360.0


def label_box(text: str, size: float, thickness: float) -> Tuple[float, float]:
    """(width_along_text, height) of the drawn label, mm. The shared
    estimator both front-ends decide with (see CHAR_ADVANCE above)."""
    return (len(text) * CHAR_ADVANCE * size + thickness,
            HEIGHT_FACTOR * size + thickness)


def label_world_anchor(fp, label) -> Tuple[float, float]:
    """World center of the label text. `local_to_global` handles B-side
    parts (KiCad stores flipped children pre-mirrored, like pad locals) --
    probed bit-equal to `Reference().GetPosition()` on both boards."""
    return local_to_global(fp.x, fp.y, fp.rotation, label.at_x, label.at_y)


def label_side(label) -> str:
    """'F' or 'B' from the label's own layer (a B-side part CAN carry an
    F-side label; the layer, not the footprint, decides what it collides
    with)."""
    return 'B' if str(label.layer).startswith('B') else 'F'


def label_world_rect(fp, label, text: Optional[str] = None,
                     size: Optional[float] = None,
                     thickness: Optional[float] = None):
    """Axis-aligned world rect of a label at its CURRENT stored pose."""
    cx, cy = label_world_anchor(fp, label)
    w, h = label_box(text if text is not None else fp.reference,
                     size if size is not None else label.size_h,
                     thickness if thickness is not None else label.thickness)
    if abs(fold_upright(label.rotation)) > 45.0:  # vertical
        w, h = h, w
    return (cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)


# --- config / result --------------------------------------------------------

@dataclass
class LabelConfig:
    """Knobs for `beautify_labels`. The GUI builds one from its options
    panel dict; the CLI from argparse. All lengths in mm."""
    size: float = 1.0                 # target font size (height == width)
    thickness: float = 0.15           # stroke width at target size
    min_size: float = 0.7             # shrink ladder floor
    size_step: float = 0.05           # shrink ladder step
    orientation: str = 'horizontal'   # preferred: 'horizontal' | 'vertical'
    allow_rotate_fallback: bool = True
    max_distance: float = 2.0         # max anchor gap growth from courtyard
    anchor_gap: float = 0.2           # base gap between courtyard and label
    silk_pad_clearance: float = 0.15  # label ink to pad copper / hole
    label_clearance: float = 0.15     # label ink to other label ink
    board_edge_clearance: Optional[float] = None  # None -> board's own floor
    include_hidden: bool = False      # place hidden labels too
    skip_locked: bool = False         # leave locked footprints' labels alone
    refs: Optional[List[str]] = None  # glob patterns; None = every label


@dataclass
class LabelResult:
    """One label's outcome. File-frame fields feed `write_label_output`;
    world-frame fields feed the GUI's direct pcbnew apply."""
    reference: str
    action: str            # 'placed' | 'rotated' | 'shrunk' | 'shrunk_rotated'
    #                        | 'unchanged' | 'skipped_hidden' | 'skipped_locked'
    #                        | 'skipped_multiline'
    reason: str = ""
    at_x: float = 0.0      # writer input: local-frame (at ...) values
    at_y: float = 0.0
    file_rotation: float = 0.0  # writer input: stored angle (ABSOLUTE, probed)
    world_x: float = 0.0   # GUI apply input: world center / drawn angle
    world_y: float = 0.0
    world_angle: float = 0.0    # folded to {0, 90} by construction
    size: float = 1.0
    thickness: float = 0.15
    old: Dict = field(default_factory=dict)

    @property
    def changed(self) -> bool:
        return self.action in ('placed', 'rotated', 'shrunk', 'shrunk_rotated')


# --- internal helpers -------------------------------------------------------

def _circle_rect_gap(cx: float, cy: float, radius: float, rect) -> float:
    """Distance from a circle's EDGE to a rect (negative = penetrating)."""
    dx = max(rect[0] - cx, 0.0, cx - rect[2])
    dy = max(rect[1] - cy, 0.0, cy - rect[3])
    return math.hypot(dx, dy) - radius


def _slide_offsets(half_span: float):
    """0, +s, -s, +2s, -2s, ... out to half_span (deterministic order)."""
    yield 0.0
    k = 1
    while k * SLIDE_STEP <= half_span + EPS:
        yield k * SLIDE_STEP
        yield -k * SLIDE_STEP
        k += 1


def _world_courtyard(fp, courtyard_sides, part_pads):
    """World-frame courtyard rect for a part, with the same fallbacks the
    quench uses: own-side courtyard, any-side courtyard, else the pad/hole
    extent (no courtyard margin -- warned about elsewhere), else a 1 mm box."""
    local = courtyard_for_side(courtyard_sides.get(fp.reference),
                               footprint_side(fp))
    if local is not None:
        x0, y0, x1, y1 = rotate_local_bounds(*local, fp.rotation)
        return (fp.x + x0, fp.y + y0, fp.x + x1, fp.y + y1)
    pp = part_pads.get(fp.reference)
    if pp is not None:
        ext = pp.extent(fp.x, fp.y, fp.rotation)
        if ext is not None:
            return ext
    return (fp.x - 0.5, fp.y - 0.5, fp.x + 0.5, fp.y + 0.5)


def _candidates(courtyard, gap, w, h):
    """Yield (anchor_rank, slide_abs, cx, cy) for one ring at `gap`."""
    x0, y0, x1, y1 = courtyard
    mid_x, mid_y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    half_w, half_h = (x1 - x0) / 2.0, (y1 - y0) / 2.0
    for rank, anchor in enumerate(ANCHORS):
        if anchor in ('above', 'below'):
            cy = (y0 - gap - h / 2.0) if anchor == 'above' \
                else (y1 + gap + h / 2.0)
            for dx in _slide_offsets(half_w + w / 2.0):
                yield rank, abs(dx), mid_x + dx, cy
        else:
            cx = (x0 - gap - w / 2.0) if anchor == 'left' \
                else (x1 + gap + w / 2.0)
            for dy in _slide_offsets(half_h + h / 2.0):
                yield rank, abs(dy), cx, mid_y + dy


# --- the engine -------------------------------------------------------------

def beautify_labels(pcb_data, pcb_file: Optional[str],
                    config: LabelConfig,
                    cancel_check: Optional[Callable[[], bool]] = None,
                    progress_callback: Optional[Callable] = None) -> Dict:
    """Compute new poses for every in-scope Reference label.

    Pure computation: nothing is written (that is `write_label_output`'s
    job), so the GUI can apply the same results directly to a live board.
    `cancel_check`/`progress_callback` follow the quench protocol (zero-arg
    cancel probe; `(current, total, label)` progress).

    Returns ``{'results': [LabelResult...], 'summary': {...}}``.
    """
    footprints = pcb_data.footprints
    results: List[LabelResult] = []

    # -- scope ---------------------------------------------------------------
    targets: List[str] = []
    for ref in sorted(footprints):
        fp = footprints[ref]
        label = fp.ref_label
        if label is None or ref.startswith('#'):
            continue  # reference-less drill dot / unparseable node: no label
        if config.refs and not any(fnmatch.fnmatchcase(ref, p)
                                   for p in config.refs):
            continue  # out of scope entirely (still an obstacle below)
        if label.hidden and not config.include_hidden:
            results.append(LabelResult(ref, 'skipped_hidden',
                                       reason='label is hidden',
                                       old=asdict(label)))
            continue
        if config.skip_locked and fp.locked:
            results.append(LabelResult(ref, 'skipped_locked',
                                       reason='footprint is locked',
                                       old=asdict(label)))
            continue
        if '\n' in ref:
            # The width estimator is single-line; a multi-line ref would be
            # modelled wrong, so report it instead of mis-placing it.
            results.append(LabelResult(ref, 'skipped_multiline',
                                       reason='multi-line reference text',
                                       old=asdict(label)))
            continue
        targets.append(ref)
    target_set = set(targets)

    # -- static obstacles ----------------------------------------------------
    edge_margin = config.board_edge_clearance
    if edge_margin is None and pcb_file:
        from list_nets import board_floor_knobs
        _, edge_margin, _ = board_floor_knobs(pcb_file, None, None)
    if edge_margin is None:
        edge_margin = 0.55

    # copper_holes=False (#730): these circles keep label INK off a drill,
    # and an NPTH pad's `(clearance ...)` is a COPPER rule -- KiCad has no
    # silk-to-hole rule at all. Honouring it here would push a reference
    # designator further from a mounting hole for a reason that does not
    # apply to silkscreen (measured on ulx3s: 0.20mm, at the shipped
    # silk_pad_clearance of 0.15). The copper consumers in legality.py take
    # the default.
    part_pads = build_part_pads(footprints, config.silk_pad_clearance,
                                copper_holes=False)
    pad_rects = {'F': [], 'B': []}   # (x0, y0, x1, y1)
    hole_circles = []                # (cx, cy, r) -- drills go through: both sides
    for ref, pp in part_pads.items():
        fp = footprints[ref]
        for (x0, y0, x1, y1, _net, pside) in pp.pad_rects(fp.x, fp.y,
                                                          fp.rotation):
            for s in (('F', 'B') if pside is None else (pside,)):
                pad_rects[s].append((x0, y0, x1, y1))
        # `hole_circles`, deliberately, NOT `hole_keepouts` (#761): silk adds
        # its own term below (`< config.silk_pad_clearance`), so this consumer
        # was already complete -- and the keep-out accessor carries a COPPER
        # requirement, which is the same reason build_part_pads above asks for
        # copper_holes=False.
        hole_circles.extend(pp.hole_circles(fp.x, fp.y, fp.rotation))

    gate = BoardOutlineGate(pcb_data.board_info, edge_margin)

    courtyard_sides = extract_courtyard_sides(pcb_file) if pcb_file else {}
    courtyards = {ref: _world_courtyard(footprints[ref], courtyard_sides,
                                        part_pads)
                  for ref in footprints}

    # Visible labels OUTSIDE the target set stay where they are: obstacles.
    # Target labels are NOT pre-seeded (they will move); each placed result
    # is appended so later refs see it (greedy sequential, sorted order).
    label_rects = {'F': [], 'B': []}
    for ref in sorted(footprints):
        fp = footprints[ref]
        label = fp.ref_label
        if label is None or label.hidden or ref in target_set:
            continue
        label_rects[label_side(label)].append(label_world_rect(fp, label))

    # -- the degradation ladder ---------------------------------------------
    pref_vertical = (config.orientation == 'vertical')
    rungs: List[Tuple[str, float, bool]] = [('placed', config.size,
                                             pref_vertical)]
    if config.allow_rotate_fallback:
        rungs.append(('rotated', config.size, not pref_vertical))
    s = config.size - config.size_step
    while s >= config.min_size - EPS:
        rungs.append(('shrunk', s, pref_vertical))
        if config.allow_rotate_fallback:
            rungs.append(('shrunk_rotated', s, not pref_vertical))
        s -= config.size_step

    def _thickness_for(size: float) -> float:
        return max(THICKNESS_FLOOR,
                   round(config.thickness * size / config.size, 4))

    canceled = False
    for i, ref in enumerate(targets):
        if cancel_check is not None and cancel_check():
            canceled = True
            break
        if progress_callback is not None:
            progress_callback(i, len(targets), f'label {ref}')
        fp = footprints[ref]
        label = fp.ref_label
        side = label_side(label)
        courtyard = courtyards[ref]

        # Prefilter obstacles to the label's reachable disk: everything a
        # candidate box could ever touch lies within max_distance + the
        # largest box extent + clearance of the courtyard.
        max_w, _ = label_box(ref, config.size, config.thickness)
        reach = (config.anchor_gap + config.max_distance + max_w
                 + max(config.silk_pad_clearance, config.label_clearance))
        near = (courtyard[0] - reach, courtyard[1] - reach,
                courtyard[2] + reach, courtyard[3] + reach)
        near_pads = [r for r in pad_rects[side]
                     if rect_gap(near, r) <= 0.0]
        near_holes = [c for c in hole_circles
                      if _circle_rect_gap(c[0], c[1], c[2], near) <= 0.0]
        near_courtyards = [courtyards[o] for o in footprints
                           if o != ref
                           and footprint_side(footprints[o]) == side
                           and rect_gap(near, courtyards[o]) <= 0.0]
        gate_edges = gate.edges_near(('label', ref), near, 0.0) \
            if gate.active else None

        def _legal(rect):
            if gate.active:
                if gate.rect_blocked(rect, gate_edges):
                    return False
            elif gate.usable is not None:
                if (rect[0] < gate.usable[0] or rect[1] < gate.usable[1]
                        or rect[2] > gate.usable[2]
                        or rect[3] > gate.usable[3]):
                    return False
            for r in near_pads:
                if rect_gap(rect, r) < config.silk_pad_clearance - EPS:
                    return False
            for (hx, hy, hr) in near_holes:
                if _circle_rect_gap(hx, hy, hr, rect) \
                        < config.silk_pad_clearance - EPS:
                    return False
            for r in label_rects[side]:
                if rect_gap(rect, r) < config.label_clearance - EPS:
                    return False
            return True

        placed = None  # (action, size, vertical, rect, cx, cy)
        for action, size, vertical in rungs:
            th = _thickness_for(size)
            w, h = label_box(ref, size, th)
            if vertical:
                w, h = h, w
            best = None  # (score, rect, cx, cy)
            ring = 0.0
            while config.anchor_gap + ring <= config.anchor_gap \
                    + config.max_distance + EPS:
                if best is not None and best[0] <= ring:
                    break  # no later ring can beat the best (score >= ring)
                gap = config.anchor_gap + ring
                for rank, slide, cx, cy in _candidates(courtyard, gap, w, h):
                    rect = (cx - w / 2.0, cy - h / 2.0,
                            cx + w / 2.0, cy + h / 2.0)
                    if not _legal(rect):
                        continue
                    overlap = sum(rect_overlap_area(rect, c)
                                  for c in near_courtyards)
                    score = rank + ring + slide + COURTYARD_PENALTY * overlap
                    if best is None or score < best[0] - EPS:
                        best = (score, rect, cx, cy)
                ring += RING_STEP
            if best is not None:
                placed = (action, size, vertical, best[1], best[2], best[3])
                break

        if placed is None:
            n_rungs = len(rungs)
            results.append(LabelResult(
                ref, 'unchanged',
                reason=(f'no legal position within {config.max_distance}mm '
                        f'of the courtyard at any of {n_rungs} '
                        f'size/orientation rung(s) (>= {config.min_size}mm)'),
                old=asdict(label)))
            continue

        action, size, vertical, rect, cx, cy = placed
        world_angle = 90.0 if vertical else 0.0
        lx, ly = _global_to_local(fp.x, fp.y, fp.rotation, cx, cy)
        results.append(LabelResult(
            ref, action,
            at_x=round(lx, 6), at_y=round(ly, 6),
            file_rotation=label_file_angle(world_angle, fp),
            world_x=cx, world_y=cy, world_angle=world_angle,
            size=size, thickness=_thickness_for(size),
            old=asdict(label)))
        label_rects[side].append(rect)  # obstacle for every later label

    if progress_callback is not None and not canceled:
        progress_callback(len(targets), len(targets), 'labels done')

    tally = {a: 0 for a in ('placed', 'rotated', 'shrunk', 'shrunk_rotated',
                            'unchanged', 'skipped_hidden', 'skipped_locked',
                            'skipped_multiline')}
    for res in results:
        tally[res.action] = tally.get(res.action, 0) + 1
    summary = {
        'labels_total': len(results),
        'unchanged_refs': sorted(r.reference for r in results
                                 if r.action == 'unchanged'),
        'size': config.size,
        'thickness': config.thickness,
        'orientation': config.orientation,
        'canceled': canceled,
    }
    summary.update(tally)
    return {'results': results, 'summary': summary}
