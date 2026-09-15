"""Independent #961 probes; native KiCad constructs and reads written fixtures."""
import contextlib
import io
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
CODE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT
sys.path[:0] = [str(CODE / p) for p in ('py_router', 'py_placer', 'py_tools')]
import pcbnew
from kicad_parser import parse_kicad_pcb
from placement import floorplan
from placement.body import board_bodies

OUT = ROOT / 'reviewer-evidence'
OUT.mkdir(exist_ok=True)

def xy(x, y):
    return pcbnew.VECTOR2I(round(x * 1e6), round(y * 1e6))

def segments(owner, points, layer, closed=True):
    for a, b in zip(points, points[1:] + (points[:1] if closed else [])):
        s = pcbnew.PCB_SHAPE(owner)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(xy(*a)); s.SetEnd(xy(*b)); s.SetLayer(layer)
        s.SetWidth(100000)
        owner.Add(s)

def fixture(name, points, *, layer=pcbnew.F_Fab, rotation=0, back=False,
            position=(2, 10), outline=None, cutout=None, closed=True, pad_x=0,
            primitive=None, extra_body=None, open_cut=None, netted=False, courtyard=False,
            pad_size=(.5,.5),pad_angle=0,pad_y=0,pad_shape='rect'):
    board = pcbnew.BOARD()
    segments(board, outline or [(0, 0), (20, 0), (20, 20), (0, 20)], pcbnew.Edge_Cuts)
    if cutout:
        segments(board, cutout, pcbnew.Edge_Cuts)
    if open_cut:
        segments(board, open_cut, pcbnew.Edge_Cuts, False)
    fp = pcbnew.FOOTPRINT(board)
    fp.SetReference('J1'); fp.SetValue('ReviewConnector'); board.Add(fp)
    if points:
        segments(fp, points, layer, closed)
    if extra_body:
        segments(fp, extra_body, layer)
    if courtyard:
        segments(fp,[(-2.5,-2.5),(2.5,-2.5),(2.5,2.5),(-2.5,2.5)],pcbnew.F_CrtYd)
    if primitive:
        s = pcbnew.PCB_SHAPE(fp)
        if primitive == 'rect':
            s.SetShape(pcbnew.SHAPE_T_RECT)
            s.SetStart(xy(-2,-2));s.SetEnd(xy(2,2))
        elif primitive == 'poly':
            s.SetShape(pcbnew.SHAPE_T_POLY)
            s.SetPolyPoints([xy(-2,2),xy(2,-2),xy(2,2)])
        else:
            s.SetShape(pcbnew.SHAPE_T_CIRCLE)
            s.SetCenter(xy(0,0)); s.SetEnd(xy(2,0))
        s.SetLayer(layer); s.SetWidth(100000); fp.Add(s)
    pad = pcbnew.PAD(fp)
    pad.SetNumber('1'); pad.SetShape({'rect':pcbnew.PAD_SHAPE_RECT,'oval':pcbnew.PAD_SHAPE_OVAL,'roundrect':pcbnew.PAD_SHAPE_ROUNDRECT}[pad_shape])
    pad.SetAttribute(pcbnew.PAD_ATTRIB_SMD); pad.SetSize(xy(*pad_size))
    if pad_shape == 'roundrect': pad.SetRoundRectRadiusRatio(.25)
    pad.SetPosition(xy(pad_x, pad_y)); pad.SetOrientationDegrees(pad_angle); pad.SetLayerSet(pcbnew.LSET.AllCuMask())
    if netted:
        net=pcbnew.NETINFO_ITEM(board,'/A'); board.Add(net); pad.SetNet(net)
    fp.Add(pad)
    fp.SetPosition(xy(*position))
    fp.SetOrientationDegrees(rotation)
    if back:
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    path = OUT / (name + '.kicad_pcb')
    pcbnew.SaveBoard(str(path), board)
    # Independent oracle: read ACTUAL saved geometry, centerline endpoints.
    reread = pcbnew.LoadBoard(str(path))
    native_fp = next(iter(reread.GetFootprints()))
    pts = []
    for s in native_fp.GraphicalItems():
        if s.GetLayer() in (pcbnew.F_Fab, pcbnew.B_Fab, pcbnew.F_SilkS, pcbnew.B_SilkS):
            native_points = s.GetPolyPoints() if s.GetShape()==pcbnew.SHAPE_T_POLY else s.GetRectCorners() if s.GetShape()==pcbnew.SHAPE_T_RECT else [s.GetStart(),s.GetEnd()]
            pts.extend((p.x/1e6,p.y/1e6) for p in native_points)
    return path, pts, native_fp.GetOrientationDegrees(), native_fp.GetLayerName()

def run():
    cases = [
        ('fab_flush', dict(points=[(-2,-2),(2,-2),(2,2),(-2,2)])),
        ('silk_pads_extend', dict(points=[(-2,-2),(2,-2),(2,2),(-2,2)], layer=pcbnew.F_SilkS, pad_x=-3)),
        ('missing', dict(points=[])),
        ('open_fab_line', dict(points=[(-2,-2),(2,2)], closed=False)),
        ('triangle_45', dict(points=[(-2,2),(2,-2),(2,2)], rotation=45, position=(2.5,10))),
        ('fab_back_37', dict(points=[(-2,-2),(2,-2),(2,2),(-2,2)], rotation=37, back=True, position=(2.5,10))),
        ('fab_front_37', dict(points=[(-2,-2),(2,-2),(2,2),(-2,2)], rotation=37, position=(2.5,10))),
        ('concave_notch', dict(points=[(-2,-2),(2,-2),(2,2),(-2,2)], position=(8,10), outline=[(0,0),(20,0),(20,20),(0,20),(0,12),(7,12),(7,8),(0,8)])),
        ('cutout', dict(points=[(-2,-2),(2,-2),(2,2),(-2,2)], position=(7,10), cutout=[(5,8),(8,8),(8,12),(5,12)])),
        ('opposite_face_only', dict(points=[(-2,-2),(2,-2),(2,2),(-2,2)], layer=pcbnew.B_Fab)),
        ('concave_body', dict(points=[(-2,-2),(2,-2),(2,2),(0,2),(0,0),(-2,0)])),
        ('circle_body', dict(points=[], primitive='circle')),
        ('disconnected_bodies', dict(points=[(-2,-2),(0,-2),(0,0),(-2,0)], extra_body=[(1,1),(2,1),(2,2),(1,2)])),
        ('slanted_boundary', dict(points=[(-2,-2),(2,-2),(2,2),(-2,2)], outline=[(0,0),(20,0),(18,20),(0,20)])),
        ('open_cut', dict(points=[(-2,-2),(2,-2),(2,2),(-2,2)], open_cut=[(10,8),(10,12)])),
        ('corner_two_edges', dict(points=[(-2,-2),(2,-2),(2,2),(-2,2)], position=(1.9,1.9))),
        ('fab_rect_37',dict(points=[],primitive='rect',rotation=37,position=(2.5,10))),
        ('fab_poly_45',dict(points=[],primitive='poly',rotation=45,position=(2.5,10))),
    ]
    rows = []
    for name, kw in cases:
        path, native_pts, angle, side = fixture(name, **kw)
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(str(path))
            body_rect, basis = floorplan.drawn_body_rect(board_bodies(pcb, str(path)).get('J1'), pcb.footprints['J1'])
        row = dict(name=name, fixture_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), native_points=native_pts, native_angle=angle,
                   native_face=side, old_drawn_rect=body_rect, old_basis=basis,
                   native_west_overhang=(max(0, -min(x for x,y in native_pts)) if native_pts else None), grades=[])
        try:
            from placement.connector_geometry import ConnectorGeometry
            cg = ConnectorGeometry(pcb, str(path))
            row['connector_geometry'] = cg.measure('J1', 'west')
            row['inferred_edge'] = cg.inferred_edge('J1')
        except ImportError:
            pass
        for margin, minimum, maximum, setback in ((.25,.05,.2,None),(.55,.05,.2,None),(.25,0,.65,None),(.55,0,.65,None),(.25,0,.65,.1)):
            entry = dict(ref='J1', edge='west', overhang_mm=dict(min=minimum,max=maximum))
            if setback is not None:
                entry['max_setback_mm'] = setback
            intent = floorplan.intent_from_dict(dict(schema=floorplan.SCHEMA_VERSION, kind=floorplan.KIND, units='mm', edge_connectors=[entry]))
            with contextlib.redirect_stdout(io.StringIO()):
                try:
                    r = floorplan.grade(intent, pcb, str(path), clearance=margin, board_edge_clearance=0)
                    result = dict(edge_seating=r.edge_seating,
                                  violations=[dict(rule=v.rule,message=v.message,measured=v.measured) for v in r.violations],
                                  abstained=getattr(r,'abstained',None))
                except Exception as e:
                    result = dict(error=type(e).__name__, reason=str(e))
            row['grades'].append(dict(requested_clearance=margin, requested_edge_clearance=0, declared_entry=entry, result=result))
        rows.append(row)
    report = dict(revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=CODE,text=True).strip(), native_kicad=pcbnew.GetBuildVersion(), rows=rows)
    (OUT/'geometry.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    for row in rows:
        print(row['name'], 'native:', row['native_west_overhang'], 'old body:',row['old_drawn_rect'],row['old_basis'])
    if any('connector_geometry' in r for r in rows):
        checked = 0
        supported = {'fab_flush', 'silk_pads_extend', 'triangle_45', 'fab_back_37', 'fab_front_37', 'corner_two_edges','fab_rect_37','fab_poly_45'}
        for row in rows:
            cg = row['connector_geometry']
            assert cg['body_measured'] == (row['name'] in supported), row['name']
            checked += 1
            for grade in row['grades']:
                result = grade['result']
                assert 'error' not in result, (row['name'],result)
                seat = result['edge_seating'][0]
                assert seat['body_overhang_mm'] == cg['body_overhang_mm'], row['name']
                assert seat['overhang_mm'] == cg['body_overhang_mm'], row['name']
                for field in ('body_overhang','body_setback','pad_copper_edge_gap','copper_edge_shortfall'):
                    m = seat['measurements'][field]
                    assert {'units','geometry_basis','value','declared_limit','requirement_source','disposition'} <= m.keys(), (row['name'],m)
                if row['name'] in supported:
                    assert abs(cg['body_overhang_mm'] - row['native_west_overhang']) < 1e-6, row['name']
                    oh = cg['body_overhang_mm']
                    band = grade['declared_entry']['overhang_mm']
                    expected = 'pass' if band['min'] - 1e-9 <= oh <= band['max'] + 1e-9 else 'fail'
                    assert seat['measurements']['body_overhang']['disposition'] == expected, row['name']
                else:
                    assert seat['measurements']['body_overhang']['disposition'] == 'unmeasured', row['name']
                    assert any('unmeasured' in v['message'] for v in result['violations']), row['name']
                checked += 8
        assert next(r for r in rows if r['name']=='corner_two_edges')['inferred_edge'] is None
        print('Independent native/grade assertions passed:',checked)

if __name__ == '__main__':
    run()



