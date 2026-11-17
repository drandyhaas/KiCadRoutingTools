"""Constrained Delaunay triangles of a polygon, from shapely 2.0 on.

`constrained_delaunay_triangles(geom)` is shapely's own function of that name
where shapely has it (2.1 and newer), else the same GEOS routine,
GEOSConstrainedDelaunayTriangulation (GEOS 3.10 and newer), called directly on
the GEOS library shapely carries, the geometry passed both ways as WKB. Both
return a GeometryCollection of triangles that keep to the polygon's edges and
its holes' (shapely 2.1's function is a wrapper over exactly that GEOS call).

Why: the bus step's 2-D embedding (awx/whole_embed.py) cuts each layer's free
space into such triangles, and KiCad 10's bundled python on macOS is 3.9, which
shapely 2.1 does not support -- its shapely 2.0.7 carries GEOS 3.11.4, which has
the routine. `unavailable()` says why neither path is open (the bus step's
dependency gate reads it).
"""
import ctypes
import ctypes.util
import glob
import os

import shapely

_GEOS = []          # [the loaded library, or None when none has the routine]


def _candidates():
    """The GEOS C libraries to try: those shapely's wheel carries (macOS
    .dylibs, Linux and Windows <dist>.libs, a DLLs folder), then the system's."""
    here = os.path.dirname(os.path.abspath(shapely.__file__))
    up = os.path.dirname(here)
    pats = [os.path.join(here, '.dylibs', 'libgeos_c*'),
            os.path.join(up, 'shapely.libs', '*geos_c*'),
            os.path.join(up, 'Shapely.libs', '*geos_c*'),
            os.path.join(here, 'DLLs', '*geos_c*'),
            os.path.join(here, '*geos_c*')]
    out = [p for pat in pats for p in sorted(glob.glob(pat))]
    sys_lib = ctypes.util.find_library('geos_c')
    if sys_lib:
        out.append(sys_lib)
    return out


def _geos():
    if not _GEOS:
        lib = None
        for cand in _candidates():
            try:
                c = ctypes.CDLL(cand)
            except OSError:
                continue
            if hasattr(c, 'GEOSConstrainedDelaunayTriangulation_r'):
                lib = c
                break
        if lib is not None:
            vp, sz = ctypes.c_void_p, ctypes.c_size_t
            for name, res, args in (
                    ('GEOS_init_r', vp, []),
                    ('GEOS_finish_r', None, [vp]),
                    ('GEOSWKBReader_create_r', vp, [vp]),
                    ('GEOSWKBReader_destroy_r', None, [vp, vp]),
                    ('GEOSWKBReader_read_r', vp, [vp, vp, ctypes.c_char_p, sz]),
                    ('GEOSWKBWriter_create_r', vp, [vp]),
                    ('GEOSWKBWriter_destroy_r', None, [vp, vp]),
                    ('GEOSWKBWriter_write_r', ctypes.POINTER(ctypes.c_ubyte), [vp, vp, vp, ctypes.POINTER(sz)]),
                    ('GEOSConstrainedDelaunayTriangulation_r', vp, [vp, vp]),
                    ('GEOSGeom_destroy_r', None, [vp, vp]),
                    ('GEOSFree_r', None, [vp, vp])):
                f = getattr(lib, name)
                f.restype, f.argtypes = res, args
        _GEOS.append(lib)
    return _GEOS[0]


def unavailable():
    """None where constrained triangles can be made here, else why not."""
    if hasattr(shapely, 'constrained_delaunay_triangles'):
        return None
    if _geos() is None:
        return (f"shapely {shapely.__version__} has no constrained_delaunay_triangles (shapely 2.1+), and no "
                f"GEOS library with GEOSConstrainedDelaunayTriangulation (GEOS 3.10+) was found beside it "
                f"(its GEOS: {getattr(shapely, 'geos_version_string', '?')})")
    return None


def constrained_delaunay_triangles(geom):
    """The constrained Delaunay triangulation of GEOM (a polygon or a
    multipolygon) as a GeometryCollection of triangles (see the module)."""
    if hasattr(shapely, 'constrained_delaunay_triangles'):
        return shapely.constrained_delaunay_triangles(geom)
    return _geos_triangles(geom)


def _geos_triangles(geom):
    lib = _geos()
    if lib is None:
        raise RuntimeError(unavailable())
    wkb = shapely.to_wkb(geom)
    h = lib.GEOS_init_r()
    rd = wr = g = t = None
    try:
        rd = lib.GEOSWKBReader_create_r(h)
        g = lib.GEOSWKBReader_read_r(h, rd, wkb, len(wkb))
        if not g:
            raise RuntimeError('GEOS could not read the geometry')
        t = lib.GEOSConstrainedDelaunayTriangulation_r(h, g)
        if not t:
            raise RuntimeError('GEOS could not triangulate the geometry')
        wr = lib.GEOSWKBWriter_create_r(h)
        n = ctypes.c_size_t(0)
        buf = lib.GEOSWKBWriter_write_r(h, wr, t, ctypes.byref(n))
        if not buf:
            raise RuntimeError('GEOS could not write the triangles')
        try:
            out = ctypes.string_at(buf, n.value)
        finally:
            lib.GEOSFree_r(h, ctypes.cast(buf, ctypes.c_void_p))
        return shapely.from_wkb(out)
    finally:
        for obj, kill in ((t, lib.GEOSGeom_destroy_r), (g, lib.GEOSGeom_destroy_r),
                          (rd, lib.GEOSWKBReader_destroy_r), (wr, lib.GEOSWKBWriter_destroy_r)):
            if obj:
                kill(h, obj)
        lib.GEOS_finish_r(h)
