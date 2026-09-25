"""whole_audit.py -- plan_audit's checks on a whole-route plan (a geometry, a polish or a snap), installed in the
planned corridor as the router gets it (whole_ctx.install).

usage as a driver: whole_audit.py GEO.json [checks]   -- plans the bench (whole_ctx: BENCH / NETS / DEST) as
plan_audit does, installs, runs plan_audit's checks (default: pitch dives static shape bands swim). whole_gate.py
reads the output."""
import sys, json


if __name__ == '__main__':
    import whole_ctx
    from whole_ctx import install
    import plan_audit as pa
    geo = json.load(open(sys.argv[1]))
    checks = sys.argv[2].split(',') if len(sys.argv) > 2 else ['pitch', 'dives', 'static', 'shape', 'bands', 'swim']
    ctx, cs = whole_ctx.plan()
    install(ctx, cs[0], geo)
    cs = cs[:1]
    if 'pitch' in checks:
        pa.check_pitch(ctx, cs)
    if 'dives' in checks:
        pa.check_dives(ctx, cs)
    if 'static' in checks:
        pa.check_static(ctx, cs)
    if 'shape' in checks:
        pa.check_shape(ctx, cs)
    if 'bands' in checks:
        pa.check_bands(ctx, cs)
    if 'swim' in checks:
        pa.check_swim(ctx, cs)
