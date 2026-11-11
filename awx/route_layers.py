"""route_layers.py -- the copper layers the whole route lays its lanes on: ROUTE_LAYERS, F.Cu,B.Cu by default -- F.Cu and
B.Cu first, then the inner layers it may use, in the order given (F.Cu,B.Cu,In2.Cu). A lane's run between two of its
changes is on one of them; every via is a through via. Index 0 is F.Cu and 1 B.Cu on every count, as the two-layer
code reads a layer. A routing layer that carries a plane is priced (plane_kinds): where an inner layer carries none
(inner_signal), a run on a ground plane's layer by the run (gnd_run_vias), on a power plane's by its length
(pwr_vias_per_mm); where every inner layer is a plane's, a run on any of them a via (no inner layer to take instead).

Read when asked, never at import: a stage run in another stage's process reads its own setting (awx_settings)."""
import awx_settings

DEFAULT = ('F.Cu', 'B.Cu')


def layers():
    """the routing layers, F.Cu and B.Cu first"""
    v = awx_settings.get('ROUTE_LAYERS') or ''
    got = tuple(x.strip() for x in v.split(',') if x.strip()) or DEFAULT
    if got[:2] != DEFAULT or len(set(got)) != len(got):
        raise SystemExit(f'ROUTE_LAYERS={v!r}: F.Cu,B.Cu first, then the inner layers the whole route may use, each once')
    return got


def stacked(copper):
    """the routing layers in the board's stack order, `copper` its copper layers top to bottom: F.Cu, the inner ones,
    B.Cu -- the order the fanout engine reads, its via from its first layer to its last a through via"""
    rl = layers()
    return [L for L in (copper or DEFAULT) if L in rl]


def escape_layers(copper=None):
    """the layers a bus's ESCAPES are planned and laid on: F.Cu and B.Cu (stack order) -- a surface escape on F.Cu, a
    via's run on B.Cu. A via stands through every layer, so a run on an inner routing layer is the same via; with more
    routing layers than two the lane's layer at a via end is the solve's (whole_solve's via ends), and the relayer
    moves the run onto it. Offered on every routing layer, the four-layer zynq LVDS bus's escape plans past 1.5 GB
    (U1's comb 23,639 moves, against 14,577); on two layers these are the routing layers themselves"""
    return [L for L in stacked(copper) if L in DEFAULT]


def index(name):
    """a routing layer's index (F.Cu 0, B.Cu 1, the inner ones after)"""
    return layers().index(name)


def plane_kinds(pcb, names=None):
    """{layer: 'gnd' | 'pwr' | '-'} for the layers `names` (the routing layers by default): an INNER layer is a PLANE's
    when its pours together cover a quarter of what the most-poured inner layer's do -- a GROUND plane's when the ground
    family's pours (net_queries) cover the most of it, else a POWER plane's, one supply's or the several the planes step
    split it among -- and every other layer, F.Cu and B.Cu always, a SIGNAL layer. Every step of the bus prices a run on
    a plane's layer by its kind (gnd_run_vias, pwr_vias_per_mm): the solve, the ends model, the joint fanout"""
    from net_queries import is_ground_net_name
    names = list(layers() if names is None else names)

    def _area(p_):
        return abs(sum(p_[i][0] * p_[(i + 1) % len(p_)][1] - p_[(i + 1) % len(p_)][0] * p_[i][1]
                       for i in range(len(p_)))) / 2
    tot, gnd = {}, {}
    for z_ in getattr(pcb, 'zones', None) or []:
        if z_.layer in DEFAULT or not z_.net_name or len(z_.polygon or ()) < 3:
            continue
        a_ = _area(z_.polygon)
        tot[z_.layer] = tot.get(z_.layer, 0.0) + a_
        if is_ground_net_name(z_.net_name):
            gnd[z_.layer] = gnd.get(z_.layer, 0.0) + a_
    big = max(tot.values(), default=0.0)
    out = {}
    for L_ in names:
        if L_ in DEFAULT or big <= 0 or tot.get(L_, 0.0) < 0.25 * big:
            out[L_] = '-'
        else:
            out[L_] = 'gnd' if 2 * gnd.get(L_, 0.0) >= tot[L_] else 'pwr'
    return out


def inner_signal(kinds):
    """`kinds` (plane_kinds) has an INNER routing layer that carries no plane: a lane kept off a plane has an inner
    layer to run on. Where it has none -- every inner layer a plane's -- the planes are the board's only inner routing,
    and a run on one is priced a via, flat (gnd_run_vias, run_price): priced as planes there, a DDR bus's ends left one
    lane's crossings no room in every round's solve, relaxed or not, and a board's two ground planes left its solve no
    plan it could prove"""
    return any(k_ == '-' for L_, k_ in kinds.items() if L_ not in DEFAULT)


def gnd_run_vias(inner=True):
    """PLANE_GND_RUN: a lane's run on a GROUND plane's layer, in vias over the same run on a signal layer (default 10),
    whatever its length: the run cuts the reference plane under its neighbours, so it is taken only where it saves
    that many vias. With no inner signal layer (`inner` False: inner_signal), a via"""
    return float(awx_settings.get('PLANE_GND_RUN') or '10') if inner else 1.0


def pwr_run_vias(inner=True):
    """PLANE_PWR_RUN: a lane's run on a POWER plane's layer, in vias over the same run on a signal layer (default 10,
    a ground plane's), whatever its length -- with its length's price beside it (pwr_vias_per_mm): a supply's pour is
    cut round the run and its pieces must be joined again, so the layer is a FALLBACK, taken only where it saves that
    many vias. At its length's price alone a bus laid half its exits on a supply's layer, 738 mm of it, where a hand
    had 13 mm. With no inner signal layer (`inner` False: inner_signal), a via"""
    return float(awx_settings.get('PLANE_PWR_RUN') or '10') if inner else 1.0


def pwr_vias_per_mm():
    """PLANE_PWR_X: a POWER plane's layer prices a run's length this many times a signal layer's (default 2) -- the
    excess, in vias per mm at the one exchange rate (select_moves.VIA_MM): a supply's pour is cut round the run, and its
    pieces must be joined again. The solve prices each run by its length; the ends model, which ranks ends and lays
    no run, a run flat at VIA_MM's length (run_price)"""
    import select_moves
    return max(0.0, float(awx_settings.get('PLANE_PWR_X') or '2') - 1.0) / select_moves.VIA_MM


def run_price(kind, inner=True):
    """the ENDS model's price of a run on a layer of kind `kind` (plane_kinds), in vias over the same run on a signal
    layer (the exact route's and the ends kernel's, which know no run's length): a ground plane's gnd_run_vias, a power
    plane's pwr_run_vias and VIA_MM of run at pwr_vias_per_mm -- with no inner signal layer (`inner` False), a via
    either"""
    if kind == 'gnd':
        return gnd_run_vias(inner)
    if kind == 'pwr':
        if not inner:
            return 1.0
        import select_moves
        return pwr_run_vias(inner) + pwr_vias_per_mm() * select_moves.VIA_MM
    return 0.0


def escape_vias(end):
    """ESCAPE_VIAS: the arrays whose escapes of the run's nets all go through a via -- a dog-bone or a via in its pad, no
    surface escape -- so a lane may leave that array on any routing layer from its via (whole_solve's via ends): 'dest'
    (the destination's), 'both', or '0' (the default: each escape the fanout's own choice, a via where it lays one).
    `end` 'src' or 'dest'. With more routing layers than two a via end whose lane leaves on its pad's layer drops its
    via (the relayer), so forced, the generated via-end cases route at their optimum (synth_layers' vias_*); on two
    layers a via end is held on the layer its run was laid, and its lane pays the via and a change. A lane may change
    layer right at a tooth laid on the surface (whole_solve's tooth vias)"""
    v = awx_settings.get('ESCAPE_VIAS') or '0'
    if v not in ('0', 'dest', 'both'):
        raise SystemExit(f'ESCAPE_VIAS={v!r}: expected 0, dest or both')
    return v == 'both' or (v == 'dest' and end == 'dest')
