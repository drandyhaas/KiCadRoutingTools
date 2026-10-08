"""route_layers.py -- the copper layers the whole route lays its lanes on: ROUTE_LAYERS, F.Cu,B.Cu by default -- F.Cu and
B.Cu first, then the inner layers it may use, in the order given (F.Cu,B.Cu,In2.Cu). A lane's run between two of its
changes is on one of them; every via is a through via. Index 0 is F.Cu and 1 B.Cu on every count, as the two-layer
code reads a layer.

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
