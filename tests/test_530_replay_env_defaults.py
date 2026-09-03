#!/usr/bin/env python3
"""#530 replay knobs: KICAD_FAB_TIER_DEFAULT / KICAD_ESCALATION_DEFAULT set the
DEFAULT of --fab-tier / --escalation for a command that omits them, so a
cloud_replay_sets --env arm can replay pre-#857 manifests under the old ladder
('auto' + 'fab'). Parse-level test:

  env unset          -> standard / board (the shipped defaults)
  env auto / fab     -> auto / fab
  env bogus values   -> ignored (shipped defaults)
  explicit flags     -> win over the env
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import fab_tiers  # noqa: E402


def _parse(env, argv):
    saved = {k: os.environ.get(k) for k in ('KICAD_FAB_TIER_DEFAULT', 'KICAD_ESCALATION_DEFAULT')}
    for k in saved:
        os.environ.pop(k, None)
    os.environ.update(env)
    try:
        p = argparse.ArgumentParser()
        fab_tiers.add_fab_tier_args(p)
        a = p.parse_args(argv)
        return a.fab_tier, a.escalation
    finally:
        for k, v in saved.items():
            os.environ.pop(k, None)
            if v is not None:
                os.environ[k] = v


def main():
    fails = []
    cases = [
        ({}, [], (fab_tiers.DEFAULT_TIER, fab_tiers.DEFAULT_ESCALATION), "unset env"),
        ({'KICAD_FAB_TIER_DEFAULT': 'auto', 'KICAD_ESCALATION_DEFAULT': 'fab'}, [],
         ('auto', 'fab'), "env auto/fab"),
        ({'KICAD_FAB_TIER_DEFAULT': 'bogus', 'KICAD_ESCALATION_DEFAULT': 'nope'}, [],
         (fab_tiers.DEFAULT_TIER, fab_tiers.DEFAULT_ESCALATION), "bogus env ignored"),
        ({'KICAD_FAB_TIER_DEFAULT': 'auto', 'KICAD_ESCALATION_DEFAULT': 'fab'},
         ['--fab-tier', 'advanced', '--escalation', 'off'], ('advanced', 'off'), "explicit wins"),
    ]
    for env, argv, want, name in cases:
        got = _parse(env, argv)
        if got != want:
            fails.append(f"{name}: got {got}, want {want}")
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print("PASS: KICAD_FAB_TIER_DEFAULT / KICAD_ESCALATION_DEFAULT default the omitted flags, "
          "bogus values are ignored, explicit flags win")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
