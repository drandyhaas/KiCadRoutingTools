#!/usr/bin/env python3
"""Fail fast if VERSION, metadata.json (and optionally the release tag) disagree.

Run this BEFORE tagging a release — and it also runs as the first CI gate in
`.github/workflows/release.yml`, ahead of the build matrix:

    python3 check_release_version.py                  # VERSION <-> metadata.json
    python3 check_release_version.py --tag v0.17.0     # also assert tag == vVERSION

It catches the exact half-bumped state that bit v0.17.0: `VERSION` and
`rust_router/Cargo.toml` were bumped but `metadata.json` was left behind, so the
release built all four binaries and then failed in `package_pcm.write_top_level`
("metadata.json has no version entry for 0.17.0"). That check is correct but
runs ~10 minutes too late; this one runs in a second.

`rust_router/Cargo.toml` is intentionally NOT checked here — the Rust crate
version is bumped only when the crate changes and legitimately differs from
`VERSION` in its patch component (see docs/release-pipeline.md).
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def fail(msg: str) -> None:
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


# Engine + GUI surfaces whose changes require a CLI/GUI parity re-audit. A
# commit since the last audit that touches ANY of these (by path prefix) trips
# the release gate; a commit touching only docs / tests / .gui-parity-checked
# itself does not. Keep in sync with CLAUDE.md's parity rule-of-thumb.
PARITY_SURFACES = (
    "route.py", "route_diff.py", "route_planes.py",
    "route_disconnected_planes.py", "bga_fanout.py", "place_fanout_clearance.py",
    "placement/", "single_ended_routing.py", "single_ended_loop.py",
    "diff_pair_routing.py", "diff_pair_loop.py", "layer_swap_optimization.py",
    "layer_swap_fallback.py", "mps_layer_swap.py", "stub_layer_switching.py",
    "phase3_routing.py", "reroute_loop.py", "obstacle_map.py", "obstacle_cache.py",
    "obstacle_costs.py", "routing_context.py", "pcb_modification.py",
    "kicad_parser.py", "output_writer.py", "kicad_writer.py", "plane_io.py",
    "kicad_routing_plugin/", "settings_persistence.py",
)


def check_gui_parity(skip: bool) -> None:
    """Release gate: no engine/GUI code has changed since the last parity audit.

    The CLI and GUI plugin call the same routing engine, but new engine
    parameters / results-data keys / writer args have to be threaded through
    BOTH the argparse layer AND every kicad_routing_plugin/ call site (see the
    parity rules in CLAUDE.md). That thread-through can't be verified purely
    mechanically, so a maintainer audits `<recorded>..HEAD` ranges by hand and
    records the last-confirmed SHA in `.gui-parity-checked`.

    This gate fails a tagged release when a commit touching a PARITY_SURFACE has
    landed since that SHA -- i.e. engine/GUI code changed and parity has not been
    re-confirmed. Commits touching only docs / tests / the marker file itself do
    NOT trip it (so bumping `.gui-parity-checked` afterward doesn't self-trip).
    Run the audit (CLAUDE.md 'Tracking the last-audited commit'), fix any gaps,
    then update `.gui-parity-checked` to HEAD. --skip-parity-check force-bypasses."""
    marker = HERE / ".gui-parity-checked"
    if not marker.exists():
        fail(".gui-parity-checked is missing; run the CLI/GUI parity audit "
             "(see CLAUDE.md) and record the audited HEAD there before releasing.")
    tokens = marker.read_text().split()
    recorded = tokens[0].strip() if tokens else ""
    if not recorded:
        fail(".gui-parity-checked has no SHA on its first line; record the "
             "audited HEAD (SHA date outcome) before releasing.")
    try:
        head = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=HERE, text=True).strip()
        if head.startswith(recorded):
            return  # audit is at HEAD
        changed = subprocess.check_output(
            ["git", "diff", "--name-only", f"{recorded}..HEAD"],
            cwd=HERE, text=True).splitlines()
    except Exception as e:
        print(f"WARNING: could not diff against {recorded[:12]} to check GUI "
              f"parity ({e}); skipping parity gate.", file=sys.stderr)
        return
    touched = sorted({f for f in changed
                      if any(f.startswith(pfx) for pfx in PARITY_SURFACES)})
    if touched:
        if skip:
            print(f"WARNING: {len(touched)} engine/GUI file(s) changed since the "
                  f"last parity audit ({recorded[:12]}), but --skip-parity-check "
                  f"was passed.", file=sys.stderr)
            return
        preview = ", ".join(touched[:6]) + ("..." if len(touched) > 6 else "")
        fail(f"{len(touched)} engine/GUI file(s) changed since the last CLI/GUI "
             f"parity audit ({recorded[:12]}): {preview}. Run the audit (CLAUDE.md), "
             f"fix any gaps, update .gui-parity-checked to HEAD, then re-run. "
             f"(if you have confirmed parity, just bump .gui-parity-checked; "
             f"docs/test-only release: --skip-parity-check.)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tag", help="release tag to check against VERSION, e.g. v0.17.0")
    ap.add_argument("--skip-parity-check", action="store_true",
                    help="bypass the .gui-parity-checked==HEAD gate (docs-only release)")
    args = ap.parse_args()

    version = (HERE / "VERSION").read_text().strip()

    meta = json.loads((HERE / "metadata.json").read_text())
    versions = [v.get("version") for v in meta.get("versions", [])]
    if version not in versions:
        fail(f"metadata.json has no version entry for {version}; available: {versions}. "
             f"Bump metadata.json's versions[].version (and its download_url) to {version}.")

    # The PCM zip the package job builds is named for VERSION; the version
    # entry's download_url must point at the matching release asset.
    entry = next(v for v in meta["versions"] if v.get("version") == version)
    expected = f"/releases/download/v{version}/KiCadRoutingTools-{version}.zip"
    url = entry.get("download_url", "")
    if not url.endswith(expected):
        fail(f"metadata.json download_url for {version} should end with {expected!r}, "
             f"got {url!r}. Update the download_url path to v{version}.")

    if args.tag is not None and args.tag != f"v{version}":
        fail(f"tag {args.tag} does not match VERSION {version} (expected v{version}).")

    # Release-mode gate: confirm CLI/GUI parity was re-audited at HEAD.
    if args.tag is not None:
        check_gui_parity(args.skip_parity_check)

    print(f"OK: VERSION={version} matches metadata.json"
          + (f" and tag {args.tag}" if args.tag else "") + ".")


if __name__ == "__main__":
    main()
