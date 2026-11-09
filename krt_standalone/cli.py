"""Execute bundled KRT tools without changing their CLI contracts."""

import json
from pathlib import Path
import runpy
import sys

from .tools import TOOLS, find_tool, resolve_script, runtime_root
from .process import close_children


def run_script(script, args):
    path = resolve_script(script)
    previous_argv, previous_path = sys.argv, sys.path[:]
    sys.argv = [str(path), *args]
    sys.path.insert(0, str(path.parent))
    sys.path.insert(1, str(runtime_root()))
    try:
        runpy.run_path(str(path), run_name='__main__')
    except SystemExit as error:
        if error.code is None:
            return 0
        if isinstance(error.code, int):
            return error.code
        print(error.code, file=sys.stderr)
        return 1
    finally:
        try:
            close_children()
        finally:
            sys.argv, sys.path[:] = previous_argv, previous_path
    return 0


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, 'reconfigure'):
            stream.reconfigure(encoding='utf-8', errors='replace', line_buffering=True)
    if not args or args[0] in ('--help', '-h'):
        print('KiCadRoutingTools standalone CLI\n\nUsage: krt <command> [arguments]\n')
        for tool in TOOLS:
            print(f'  {tool.command:20} {tool.description}')
        print('\nUse krt <command> --help for the original tool options.')
        return 0
    if args == ['--version']:
        print('KiCadRoutingTools ' + (runtime_root() / 'VERSION').read_text().strip())
        return 0
    if args == ['--build-info']:
        manifest = runtime_root() / 'build-info.json'
        print(manifest.read_text(encoding='utf-8') if manifest.is_file() else json.dumps({
            'version': (runtime_root() / 'VERSION').read_text().strip(), 'mode': 'source',
        }))
        return 0
    try:
        if args[0] == '_tool':
            if len(args) < 2:
                raise ValueError('KRT_TOOL_ARGUMENT_REQUIRED')
            return run_script(args[1], args[2:])
        tool = find_tool(args[0])
        if tool is None:
            raise ValueError('KRT_UNKNOWN_COMMAND: ' + args[0])
        return run_script(tool.script, args[1:])
    except ValueError as error:
        print(str(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
