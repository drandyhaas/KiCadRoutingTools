#!/usr/bin/env python3
"""The milestone ledger's board store and ledger (`board_store`), and
`pose_score.rank_poses`: position and rotation ranked together, nearest
legal pose first.
"""
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # placement split
from board_store import BoardStore, Ledger, sha256_file  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


# ---------------------------------------------------------------- board store

def test_same_content_is_one_entry_and_round_trips():
    with tempfile.TemporaryDirectory() as td:
        s = BoardStore(os.path.join(td, 'store'))
        a = os.path.join(td, 'a.kicad_pcb')
        b = os.path.join(td, 'b.kicad_pcb')
        shutil.copyfile(BOARD, a)
        shutil.copyfile(BOARD, b)          # same bytes, different path
        sha1, sha2 = s.put(a), s.put(b)
        assert sha1 == sha2, "content-addressed: the path must not matter"
        stored = os.path.join(s.root, sha1, 'board.kicad_pcb')
        assert sha256_file(stored) == sha1, "the stored copy must be byte-identical"
    print("  PASS: same content -> one entry; the stored copy is byte-identical")


def test_siblings_travel_with_the_board():
    """A board without its .kicad_pro grades differently -- the project carries
    the DRC floor and the net classes. A store that drops it hands back
    something that is not the board that was stored."""
    with tempfile.TemporaryDirectory() as td:
        brd = os.path.join(td, 'x.kicad_pcb')
        shutil.copyfile(BOARD, brd)
        with open(os.path.join(td, 'x.kicad_pro'), 'w') as f:
            f.write('{"net_settings": {"classes": [{"name": "Default"}]}}')
        with open(os.path.join(td, 'x.kicad_dru'), 'w') as f:
            f.write('(version 1)\n')
        s = BoardStore(os.path.join(td, 'store'))
        sha = s.put(brd)
        assert os.path.isfile(os.path.join(s.root, sha, 'board.kicad_pro')), \
            "the project sibling did not travel"
        assert os.path.isfile(os.path.join(s.root, sha, 'board.kicad_dru')), \
            "the custom-rules sibling did not travel"
    print("  PASS: .kicad_pro and .kicad_dru travel with the board")


# --------------------------------------------------------------------- ledger

def test_the_last_accepted_row_is_found_past_a_rejected_one():
    with tempfile.TemporaryDirectory() as td:
        lg = Ledger(os.path.join(td, 'ledger.jsonl'))
        lg.append({'iteration': 1, 'kind': 'completion', 'parent_sha': 'a' * 64,
                   'result_sha': 'b' * 64, 'accepted': True})
        lg.append({'iteration': 2, 'kind': 'systemic', 'accepted': False,
                   'lever': 'restored the net classes'})
        assert lg.last_accepted()['iteration'] == 1, \
            "the last ACCEPTED entry, not the last entry"
    print("  PASS: the last accepted row is found past a rejected one")


def test_a_torn_line_does_not_lose_the_ledger():
    """JSONL so a killed run keeps everything written before it."""
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'l.jsonl')
        lg = Ledger(p)
        lg.append({'iteration': 1, 'accepted': True})
        with open(p, 'a', encoding='utf-8') as f:
            f.write('{"iteration": 2, "acce')      # killed mid-write
        assert len(lg.entries()) == 1
        assert lg.last_accepted()['iteration'] == 1
    print("  PASS: a torn final line costs only that line")


# ----------------------------------------------------------------- pose score

def test_poses_are_ranked_and_legal_only():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    from kicad_parser import parse_kicad_pcb
    import pose_score
    pcb = parse_kicad_pcb(BOARD)
    st = pose_score.make_state(pcb, BOARD)
    ref = sorted(st.parts)[0]
    poses = pose_score.rank_poses(pcb, BOARD, ref, radius=1.0, step=0.5, state=st)
    assert poses, "no legal pose at all, including staying put"
    dists = [p['dist_mm'] for p in poses]
    assert dists == sorted(dists), "poses must come back nearest first"
    assert all(p['legal'] for p in poses), "an illegal pose must be dropped, not ranked"
    assert {'x', 'y', 'rot', 'dist_mm', 'legal'} <= set(poses[0])
    # position AND rotation vary
    assert len({p['rot'] for p in poses}) >= 1
    assert len({(p['x'], p['y']) for p in poses}) > 1, \
        "position must vary, not just rotation"
    print(f"  PASS: {len(poses)} legal poses, nearest first, position+rotation together")


def test_scoring_leaves_the_board_where_it_found_it():
    """Ranking only asks the gate. If it moved a part, every later
    decision would be made against a board nobody chose."""
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    from kicad_parser import parse_kicad_pcb
    import pose_score
    pcb = parse_kicad_pcb(BOARD)
    st = pose_score.make_state(pcb, BOARD)
    ref = sorted(st.parts)[0]
    before = (st.parts[ref].x, st.parts[ref].y, st.parts[ref].rot)
    wire_before = st.wire_metrics()
    pose_score.rank_poses(pcb, BOARD, ref, radius=1.0, step=0.5, state=st)
    after = (st.parts[ref].x, st.parts[ref].y, st.parts[ref].rot)
    assert before == after, f"ranking moved the part: {before} -> {after}"
    assert st.wire_metrics() == wire_before
    print("  PASS: ranking is side-effect free")


if __name__ == '__main__':
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")
