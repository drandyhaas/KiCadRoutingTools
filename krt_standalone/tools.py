"""The standalone commands and their original KiCadRoutingTools scripts."""

from dataclasses import dataclass
from pathlib import Path
import json
import sys


@dataclass(frozen=True)
class ToolSpec:
    command: str
    script: str
    description: str
    aliases: tuple[str, ...] = ()


TOOLS = (
    ToolSpec('self-test', 'krt_standalone/self_test.py', 'Verify bundled native calculations offline'),
    ToolSpec('route', 'py_router/route.py', 'Route single-ended signals'),
    ToolSpec('route-diff', 'py_router/route_diff.py', 'Route differential pairs'),
    ToolSpec('route-planes', 'py_router/route_planes.py', 'Route power and ground planes'),
    ToolSpec('repair-planes', 'py_router/repair_planes.py', 'Repair plane connectivity'),
    ToolSpec('autoplan', 'py_router/autoplan.py', 'Plan and execute a routing chain'),
    ToolSpec('place', 'py_placer/place_simple.py', 'Place components by functional group'),
    ToolSpec('judge-seeds', 'py_placer/judge_seeds.py', 'Route and compare placement seeds', ('judge',)),
    ToolSpec('route-bus', 'awx/route_bus.py', 'Route a bus using CP-SAT'),
    ToolSpec('render-placement', 'py_tools/render_placement.py', 'Render placement to PNG'),
    ToolSpec('render', 'py_router/route_render.py', 'Render a routed board to PNG'),
    ToolSpec('check-drc', 'py_router/check_drc.py', 'Check routing geometry'),
    ToolSpec('check-connected', 'py_router/check_connected.py', 'Check net connectivity'),
    ToolSpec('check-floorplan', 'py_tools/check_floorplan.py', 'Check floorplan intent'),
    ToolSpec('check-assembly', 'py_tools/check_assembly.py', 'Check placement assembly'),
    ToolSpec('check-groups', 'py_tools/check_groups.py', 'Check placement groups'),
    ToolSpec('quick-score', 'py_tools/quick_score.py', 'Score a routed placement'),
    ToolSpec('board-context', 'py_tools/board_context.py', 'Read component and board context'),
    ToolSpec('board-brief', 'py_tools/board_brief.py', 'Summarize a board for placement'),
    ToolSpec('place-pose', 'py_placer/place_pose.py', 'Apply explicit component poses'),
    ToolSpec('beautify-labels', 'py_placer/beautify_labels.py', 'Tidy reference labels'),
    ToolSpec('bga-fanout', 'py_router/bga_fanout.py', 'Fan out BGA pads'),
    ToolSpec('qfn-fanout', 'py_router/qfn_fanout.py', 'Fan out QFN and QFP pads'),
    ToolSpec('list-nets', 'py_router/list_nets.py', 'List board nets and design rules'),
    ToolSpec('copy-board', 'py_router/copy_board.py', 'Copy a board and project siblings'),
    ToolSpec('capabilities', 'krt_capabilities.py', 'Describe bundled engine capabilities'),
)


def runtime_root():
    if getattr(sys, 'frozen', False):
        return Path(sys._MEIPASS) / 'krt'
    return Path(__file__).resolve().parents[1]


def bundled_scripts(root=None):
    root = root or runtime_root()
    manifest = root / 'scripts.json'
    if manifest.is_file():
        data = json.loads(manifest.read_text(encoding='utf-8'))
        if data.get('schema') != 1 or not isinstance(data.get('scripts'), list):
            raise ValueError('KRT_SCRIPT_MANIFEST_INVALID')
        scripts = frozenset(data['scripts'])
    elif getattr(sys, 'frozen', False):
        raise ValueError('KRT_SCRIPT_MANIFEST_MISSING')
    else:
        directories = ('py_router', 'py_placer', 'py_tools', 'awx', 'krt_standalone')
        scripts = frozenset(
            path.relative_to(root).as_posix()
            for directory in directories
            for path in (root / directory).rglob('*.py')
        ) | {'krt_capabilities.py'}
    return scripts


def resolve_script(value, root=None):
    root = (root or runtime_root()).resolve()
    path = Path(value)
    path = path.resolve() if path.is_absolute() else (root / path).resolve()
    try:
        relative = path.relative_to(root).as_posix()
    except ValueError:
        raise ValueError('KRT_TOOL_PATH_NOT_ALLOWED') from None
    if relative not in bundled_scripts(root) or not path.is_file():
        raise ValueError('KRT_TOOL_NOT_BUNDLED: ' + relative)
    return path


def find_tool(name):
    return next((tool for tool in TOOLS if name == tool.command or name in tool.aliases), None)
