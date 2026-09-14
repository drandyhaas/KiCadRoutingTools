"""Independent native written-board copper support calculations vs candidate API."""
import contextlib
import io
import math
import hashlib
import json
import reviewer_geometry as g
import pcbnew
from kicad_parser import parse_kicad_pcb
from placement.connector_geometry import candidate_copper
from pose_score import make_state

rows=[]
for shape in ('rect','oval','roundrect'):
    for back in (False,True):
        for start in (0,89.5,90.5,37):
            name='copper_'+shape+'_'+str(back)+'_'+str(start)
            src,*_=g.fixture(name,[(-2,-2),(2,-2),(2,2),(-2,2)],position=(6,10),rotation=start,back=back,
                netted=False,pad_size=(1.2,.6),pad_x=-1.1,pad_y=.3,pad_angle=0,pad_shape=shape)
            with contextlib.redirect_stdout(io.StringIO()):
                pcb=parse_kicad_pcb(str(src));state=make_state(pcb,str(src),clearance=.25,board_edge_clearance=.55)
            for target in (0,.5,1.0001,37,89.5,90,90.5,179.5,359.5):
                state.parts['J1'].rot=target
                got=candidate_copper(state,'J1',2,10)
                board=pcbnew.LoadBoard(str(src));fp=next(iter(board.GetFootprints()))
                fp.SetOrientationDegrees(target);fp.SetPosition(g.xy(2,10))
                dst=g.OUT/(name+'_to_'+str(target)+'.kicad_pcb')
                pcbnew.SaveBoard(str(dst),board)
                native=pcbnew.LoadBoard(str(dst));pad=next(iter(next(iter(native.GetFootprints())).Pads()))
                # Native pose/shape and analytic support, independent of parser
                # normalization and candidate transform being reviewed.
                sx,sy=pad.GetSize().x/1e6,pad.GetSize().y/1e6
                angle=math.radians(pad.GetOrientationDegrees())
                rad=min(sx,sy)/2 if shape=='oval' else min(sx,sy)*.25 if shape=='roundrect' else 0
                ex=(sx/2-rad)*abs(math.cos(angle))+(sy/2-rad)*abs(math.sin(angle))+rad
                ey=(sx/2-rad)*abs(math.sin(angle))+(sy/2-rad)*abs(math.cos(angle))+rad
                x,y=pad.GetPosition().x/1e6,pad.GetPosition().y/1e6
                expected=min(x-ex,20-x-ex,y-ey,20-y-ey)
                discrepancy=got['minimum_gap_mm']-expected
                row=dict(shape=shape,back=back,start=start,target=target,native_pad_angle=pad.GetOrientationDegrees(),native_pad_center=[x,y],
                         expected_gap_mm=expected,candidate_gap_mm=got['minimum_gap_mm'],delta_mm=discrepancy,complete=got['complete'],
                         expected_shortfall_mm=max(0,.55-expected),findings=got['findings'],fixture_sha256=hashlib.sha256(dst.read_bytes()).hexdigest())
                rows.append(row)
                assert got['complete'] and abs(discrepancy)<2e-6,row
                assert bool(got['findings'])==(expected < .55-2e-6),row
(g.OUT/'candidate_copper.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
print('Native-written candidate copper controls passed:',len(rows),'max gap discrepancy mm',max(abs(r['delta_mm']) for r in rows))
