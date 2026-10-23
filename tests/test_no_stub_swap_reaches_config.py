#!/usr/bin/env python3
"""
Regression test: --no-stub-layer-swap must reach GridRouteConfig.stub_layer_swap.

The flag became batch_route(enable_layer_switch=False), which gated only the
up-front stub pass. The in-loop and reconciliation STUB-SWAP RESCUE read
config.stub_layer_swap (single_ended_loop.py, stub_layer_switching.py), which
was never set and defaulted to True, so a run asked to keep input stubs still
moved them: on cm5-io stage 3 a 0.8 mm CHG_CC_5V stub swapped onto F.Cu shorted
a fine-pitch U1002 pad. route_diff.py had the same gap, and its multipoint
terminal stub switch read neither setting.

Checks:
  1. batch_route and batch_route_diff_pairs build their config with
     stub_layer_swap == enable_layer_switch (stopped at GridRouteConfig).
  2. The diff-pair multipoint terminal stub switch moves nothing when
     stub_layer_swap is off (control: it switches the same stubs when on).
  3. route.py on the #1261 STUB-SWAP RESCUE board (its stub left unlocked):
     the rescue switches NET_A's stub by default and not under
     --no-stub-layer-swap, which ships the input stub where it was.

Run:
    python3 tests/test_no_stub_swap_reaches_config.py
"""

import contextlib
import io
import os
import re
import shutil
import sys
import tempfile
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "py_router"))

import route  # noqa: E402
import route_diff  # noqa: E402
from run_utils import check as run_check, evidence, tool  # noqa: E402

BOARD = os.path.join(ROOT, "kicad_files", "cap_chain.kicad_pcb")
RESCUE = "STUB-SWAP RESCUE: NET_A"
failures = []


def check(label, ok, detail=""):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + str(detail)) if detail else ''}")
    if not ok:
        failures.append(label)


class _Stop(Exception):
    pass


def captured_stub_swap(module, engine, enable):
    real = module.GridRouteConfig
    seen = {}

    def capture(**kwargs):
        seen.update(kwargs)
        raise _Stop

    module.GridRouteConfig = capture
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            getattr(module, engine)(BOARD, os.devnull, [], enable_layer_switch=enable)
    except _Stop:
        pass
    finally:
        module.GridRouteConfig = real
    return seen.get("stub_layer_swap", "<not set>")


def config_reach():
    print("1. the flag reaches config.stub_layer_swap")
    for module, engine in ((route, "batch_route"),
                           (route_diff, "batch_route_diff_pairs")):
        got = (captured_stub_swap(module, engine, False),
               captured_stub_swap(module, engine, True))
        check(f"{engine}: enable_layer_switch False/True -> stub_layer_swap",
              got == (False, True), got)


class _WalledMap:
    """F.Cu (index 0) fully walled, B.Cu open."""
    def is_blocked(self, gx, gy, layer):
        return layer == 0


def multipoint_switch():
    print("2. diff-pair multipoint terminal stub switch")
    from synth import make_net, make_pad, make_pcb, make_seg
    from kicad_parser import BoardInfo
    from routing_config import GridRouteConfig
    import diff_pair_multipoint as dpm

    def run(swap_on):
        pp = make_pad(1, 10.0, 10.0, ref="J1", num="1", size_x=0.6, size_y=0.6)
        nn = make_pad(2, 11.0, 10.0, ref="J1", num="2", size_x=0.6, size_y=0.6)
        pcb = make_pcb(nets={1: make_net(1, "/D_P"), 2: make_net(2, "/D_N")},
                       segments=[make_seg(10.0, 10.0, 10.0, 11.0, net_id=1),
                                 make_seg(11.0, 10.0, 11.0, 11.0, net_id=2)],
                       pads_by_net={1: [pp], 2: [nn]},
                       board_info=BoardInfo(layers={0: "F.Cu", 31: "B.Cu"},
                                            copper_layers=["F.Cu", "B.Cu"],
                                            board_bounds=(0.0, 0.0, 30.0, 30.0)))
        cfg = GridRouteConfig(layers=["F.Cu", "B.Cu"], grid_step=0.1,
                              track_width=0.2, clearance=0.15, via_size=0.45,
                              via_drill=0.2, stub_layer_swap=swap_on)
        state = SimpleNamespace(config=cfg, pcb_data=pcb,
                                diff_pair_base_obstacles=_WalledMap())
        with contextlib.redirect_stdout(io.StringIO()):
            switched = dpm._switch_blocked_terminal_stubs(state, None, [(pp, nn)])
        return switched, sorted(s.layer for s in pcb.segments), len(pcb.vias)

    switched, layers, vias = run(True)
    if not switched:
        check("BROKEN FIXTURE: the control switches the walled stubs", False, layers)
        return
    check("control: the walled stubs are switched", layers == ["B.Cu", "B.Cu"], layers)
    switched, layers, vias = run(False)
    check("stub_layer_swap off: nothing switched, no pad via",
          not switched and layers == ["F.Cu", "F.Cu"] and vias == 0,
          f"{len(switched)} switch(es), stubs on {layers}, {vias} via(s)")


def rescue_route():
    print("3. route.py STUB-SWAP RESCUE")
    from test_1261_locked_stub_never_switches import rescue_board
    text, args = rescue_board(lock=False)
    tmp = tempfile.mkdtemp(prefix="t1270_")
    try:
        inp = os.path.join(tmp, "in.kicad_pcb")
        with open(inp, "w", encoding="utf-8") as fh:
            fh.write(text)
        evidence(inp, "synthetic input board")
        env = dict(os.environ, KICAD_PLANE_FINALIZE="0")
        outs = {}
        for tag, extra in (("default", []), ("no_swap", ["--no-stub-layer-swap"])):
            out = os.path.join(tmp, f"{tag}.kicad_pcb")   # fresh path per run
            r = run_check([sys.executable, "-X", "utf8", tool("route.py"), inp, out]
                          + args + ["--keep-input-copper", "--escalation", "off",
                                    "--no-fix-drc-settings"] + extra,
                          accept=True, env=env)
            evidence(out, "routed output")
            with open(out, encoding="utf-8") as fh:
                outs[tag] = (r.stdout + r.stderr, fh.read())
        con, _ = outs["default"]
        if RESCUE not in con:
            check("BROKEN FIXTURE: the default run takes the stub-swap rescue", False)
            return
        check("control: the default run switches NET_A's stub", True)
        con, board = outs["no_swap"]
        hit = [l.strip() for l in con.splitlines() if RESCUE in l]
        check("--no-stub-layer-swap: no rescue switch", not hit, "; ".join(hit))
        stub = re.findall(r'\(segment \(start 10\.05 15\.05\) \(end 14\.05 15\.05\)'
                          r'[^\n]*\(layer "([^"]+)"\)', board)
        check("and the input stub ships on F.Cu", stub == ["F.Cu"], stub)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main() -> int:
    config_reach()
    multipoint_switch()
    rescue_route()
    print("FAILED: " + ", ".join(failures) if failures else
          "PASS: --no-stub-layer-swap reaches config.stub_layer_swap")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
