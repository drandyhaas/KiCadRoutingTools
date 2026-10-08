"""whole_audit.py -- plan_audit's checks on a whole-route plan (a geometry, a polish or a snap), installed in the
whole route's lanes as the router gets them (whole_ctx.lanes).

usage as a driver: whole_audit.py GEO.json [checks]   -- reads the bench (whole_ctx: BENCH / NETS / DEST), installs,
runs plan_audit's checks (default: pitch dives static shape bands swim). whole_gate.py reads the output."""
import sys, json
import awx_settings


if __name__ == '__main__':
    import whole_ctx
    import plan_audit as pa
    geo = json.load(open(sys.argv[1]))
    checks = sys.argv[2].split(',') if len(sys.argv) > 2 else ['pitch', 'dives', 'static', 'shape', 'bands', 'swim']
    ctx, _groups = whole_ctx.plan()
    cs = [whole_ctx.lanes(ctx, awx_settings.req('DEST'), geo)]
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
