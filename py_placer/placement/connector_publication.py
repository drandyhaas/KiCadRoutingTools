"""Check declared connector requirements on the final candidate before publication."""
from __future__ import annotations

import contextlib
import copy
import io
import json
import os
import sys
import tempfile


class _CandidateOutput(io.TextIOBase):
    """Stream progress, retain candidate summaries until their disposition is known."""
    def __init__(self, target):
        self.target = target
        self.pending = ''
        self.summary = {}

    def write(self, text):
        self.pending += text
        while '\n' in self.pending:
            line, self.pending = self.pending.split('\n', 1)
            if line.startswith('JSON_SUMMARY: '):
                self.summary = json.loads(line[len('JSON_SUMMARY: '):])
            else:
                self.target.write(line + '\n')
        return len(text)

    def flush(self):
        self.target.flush()


def run_checked(args, execute):
    """Stage intent-driven seed/reconstruction work outside any armed destination.

    Existing no-intent and nonbinding-class exploration retains its policy.
    For declared connector requirements, only the finished candidate can be
    published. The underlying command's other improving-pile/exit policies
    remain intact; a published exploratory result is explicitly unclean.
    """
    from . import floorplan
    if not args.intent:
        return execute(args)
    try:
        intent = floorplan.load_intent(args.intent)
    except (OSError, ValueError):
        return execute(args)  # preserve the CLI's own schema refusal
    claimed = any(c.get('edge') or c.get('max_setback_mm') is not None
                  or c.get('center_on_edge') or c.get('along_edge_band')
                  or (c.get('overhang_mm') or {}).get('max') is not None
                  or (c.get('overhang_mm') or {}).get('min', 0) > 0
                  for c in intent.edge_connectors)
    if not claimed:
        return execute(args)
    from kicad_parser import parse_kicad_pcb
    from .publication import input_identity, publish_board, PublicationError
    from .provenance import UnaidedViolation
    identity = input_identity(args.input_file)
    with tempfile.TemporaryDirectory(prefix='krt_connector_candidate_') as stage:
        trial = copy.copy(args)
        trial.output_file = os.path.join(stage, os.path.basename(args.output_file))
        capture = _CandidateOutput(sys.stdout)
        with contextlib.redirect_stdout(capture):
            rc = execute(trial)
        if capture.pending:
            print(capture.pending, end='')
        summary = capture.summary
        if args.dry_run or not os.path.isfile(trial.output_file):
            if summary:
                summary.update(output=None, published=False)
                print('JSON_SUMMARY: ' + json.dumps(summary, sort_keys=True, default=str))
            return rc
        pcb = parse_kicad_pcb(trial.output_file)
        graded = floorplan.grade(intent, pcb, trial.output_file,
            clearance=args.clearance, board_edge_clearance=args.board_edge_clearance)
        errors = [v for v in graded.errors if v.rule == 'edge_connector']
        unmeasured = {k: v for k, v in graded.budget_abstained.items()
                      if k.startswith('edge_connectors[')}
        accepted = not errors and not unmeasured
        summary['connector_requirements'] = {
            'accepted': accepted, 'complete': not unmeasured,
            'edge_seating': graded.edge_seating,
            'errors': [v.message for v in errors], 'unmeasured': unmeasured}
        summary.update(output=None, published=False)
        if not accepted:
            summary.update(status='refused', complete=False,
                           refusal='declared connector requirements failed or unmeasured',
                           output_state='unchanged')
            print('REFUSED before publication: declared connector requirements failed or unmeasured')
            rc = 4
        else:
            try:
                publish_board(trial.output_file, args.output_file,
                              input_file=args.input_file, expected_input=identity)
                summary.update(output=args.output_file, published=True,
                               status='ok' if rc == 0 else 'exploratory',
                               engineering_clean=False)
            except (PublicationError, UnaidedViolation) as exc:
                details = getattr(exc, 'details', {'output_state': 'unchanged'})
                summary.update(details)
                summary.update(status='error', complete=False, error=str(exc),
                               published=details.get('output_state') == 'committed')
                if details.get('output_state') in ('committed', 'partial'):
                    summary['output'] = args.output_file
                rc = 4
            # This gate certifies connector requirements only; broader physical
            # and routing certification remains the independent checker's job.
        print('JSON_SUMMARY: ' + json.dumps(summary, sort_keys=True, default=str), flush=True)
        return rc
