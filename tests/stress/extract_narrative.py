#!/usr/bin/env python3
"""Turn a stress-test agent transcript into a compact routing decision trail.

Reads a JSONL agent transcript (either a `claude -p --output-format stream-json`
capture or a session/sub-agent transcript) and writes a Markdown file pairing the
agent's narration with the **routing-pipeline commands** it issued
(route / route_diff / route_planes / route_disconnected_planes / bga_fanout /
qfn_fanout). Analysis, file reads, and verification (check_*/compare/list_nets)
steps are dropped — their output already lives in the run dir's `*.log` files.

The model's internal *thinking* blocks are NOT in these transcripts; this captures
what the agent said and the routing commands it ran.

Usage:
    extract_narrative.py <transcript.jsonl> [-o OUTPUT.md] [--board NAME]

Designed to be non-fatal: malformed/empty input prints a warning and exits 0 so a
stress run is never failed by narrative generation.
"""
import argparse
import datetime
import json
import os
import re
import sys

# The board-mutating routing pipeline. A tool only counts when it is actually
# executed (python3 ... <tool>.py ...), so prose that merely mentions a tool name
# is left as narration.
_TOOL = r'(route_disconnected_planes|route_planes|route_diff|route|bga_fanout|qfn_fanout)\.py'
_EXEC = re.compile(r'python3[^\n|]*?' + _TOOL)


def _ts(rec):
    try:
        return datetime.datetime.fromisoformat(
            rec['timestamp'].replace('Z', '+00:00')).strftime('%H:%M:%S')
    except Exception:
        return ""


def _routing_cmd(block):
    """If a tool_use Bash block executes a routing tool, return a clean one-liner."""
    if block.get('name') != 'Bash':
        return None
    cmd = (block.get('input') or {}).get('command', '')
    m = _EXEC.search(cmd)
    if not m:
        return None
    tool_at = re.search(_TOOL, cmd[m.start():]).start() + m.start()
    s = cmd[tool_at:]
    tee = re.search(r'tee\s+(\S+)', s)
    s = re.split(r'\s*2>&1|\s*\|\s*', s)[0]
    s = s.replace('\\\n', ' ').replace(' \\ ', ' ').replace('\\', ' ')  # drop continuations
    s = re.sub(r'/\S+/([^/\s]+\.kicad_pcb)', r'\1', s)                   # abs paths -> basenames
    s = re.sub(r'\s+', ' ', s).strip()
    return f"`{s}`" + (f"  →  {tee.group(1)}" if tee else "")


def build(records, board=None):
    recs = [r for r in records if isinstance(r, dict)]
    times = [t for t in (_ts(r) for r in recs) if t]
    title = f"{board} — routing decision trail" if board else "Routing decision trail"

    out = [
        f"# {title}\n",
        "> Agent narration paired with the **routing-pipeline commands** it issued "
        "(diff / signal / planes / repair / fanout). Analysis, reads, and verification "
        "steps are omitted; tool outputs are in the sibling `*.log` files. The model's "
        "internal *thinking* is not captured in the transcript.\n",
    ]
    if times:
        out.append(f"_{times[0]}–{times[-1]} UTC._\n")
    out.append("---\n")
    turns = cmds_n = 0
    for rec in recs:
        if rec.get('type') != 'assistant':
            continue
        content = (rec.get('message') or {}).get('content')
        if not isinstance(content, list):
            continue
        txt = " ".join(
            b.get('text', '').strip() for b in content
            if isinstance(b, dict) and b.get('type') == 'text' and b.get('text', '').strip())
        cmds = [c for b in content if isinstance(b, dict)
                for c in [_routing_cmd(b)] if c]
        if not txt and not cmds:
            continue
        turns += 1
        if txt:
            out.append(txt)
        for c in cmds:
            out.append(c)
            cmds_n += 1
        out.append("")
    return "\n".join(out).rstrip() + "\n", turns, cmds_n


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("transcript", help="agent transcript JSONL")
    ap.add_argument("-o", "--output", help="output .md (default: <transcript dir>/agent_narrative.md)")
    ap.add_argument("--board", help="board name for the title (default: derived from output dir)")
    args = ap.parse_args()

    if not os.path.exists(args.transcript):
        print(f"extract_narrative: no transcript at {args.transcript} (skipping)", file=sys.stderr)
        return 0

    records = []
    for line in open(args.transcript, encoding="utf-8", errors="replace"):
        line = line.strip()
        if not line:
            continue
        try:
            records.append(json.loads(line))
        except Exception:
            continue
    if not records:
        print(f"extract_narrative: {args.transcript} had no parseable JSON (skipping)", file=sys.stderr)
        return 0

    out_path = args.output or os.path.join(os.path.dirname(os.path.abspath(args.transcript)),
                                           "agent_narrative.md")
    board = args.board or os.path.basename(os.path.dirname(os.path.abspath(out_path))) or None
    md, turns, cmds_n = build(records, board=board)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(md)
    print(f"extract_narrative: wrote {out_path} ({turns} turns, {cmds_n} routing commands)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
