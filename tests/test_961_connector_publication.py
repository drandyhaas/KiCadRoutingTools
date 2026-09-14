"""Actual intent-driven CLI publication: accepted, refused and unevaluated dry candidates."""
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for folder in ('py_router', 'py_placer'):
    sys.path.insert(0, str(ROOT / folder))
import pcbnew
from placement import provenance
from copy_board import copy_board

SOURCE = ROOT / 'kicad_files/esp_prog.kicad_pcb'
RUN_ALL_TIMEOUT = 600


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ConnectorPublication(unittest.TestCase):
    def test_native_final_board_and_refused_dry_destinations(self):
        with tempfile.TemporaryDirectory() as tmp:
            for tool, extra in (('place_seed.py', ['--repair']),
                                ('place_reconstruct.py', ['--stages', 'classify'])):
                for mode in ('accepted', 'positive_minimum', 'dry'):
                    with self.subTest(tool=tool, mode=mode):
                        work = Path(tmp) / (tool + mode)
                        work.mkdir()
                        source = work / 'source.kicad_pcb'
                        copy_board(str(SOURCE), str(source))
                        intent = work / 'intent.json'
                        intent.write_text(json.dumps({
                            'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
                            'edge_connectors': [{'ref': 'USB1', 'edge': 'west',
                                'overhang_mm': {'min': .05 if mode != 'accepted' else 0,
                                                'max': .65}}]}), encoding='utf8')
                        before, declared = sha(source), sha(intent)
                        provenance.start_regime(str(work), str(source))
                        output = work / 'out.kicad_pcb'
                        if mode != 'accepted':
                            copy_board(str(source), str(output))
                        argv = [sys.executable, '-X', 'utf8', str(ROOT / 'py_placer' / tool),
                                str(source), str(output), '--intent', str(intent),
                                '--clearance', '.2', '--board-edge-clearance', '.25', *extra]
                        if mode == 'dry':
                            argv.append('--dry-run')
                        result = subprocess.run(argv, cwd=ROOT, capture_output=True,
                                                text=True, encoding='utf8', timeout=180)
                        summaries = [json.loads(line.split(': ', 1)[1])
                                     for line in result.stdout.splitlines()
                                     if line.startswith('JSON_SUMMARY: ')]
                        self.assertEqual(len(summaries), 1, result.stdout + result.stderr)
                        summary = summaries[0]
                        rows = provenance.read_ledger(str(work))
                        self.assertEqual(sha(source), before)
                        self.assertEqual(sha(intent), declared)
                        if mode == 'accepted':
                            self.assertEqual(result.returncode, 0, result.stdout)
                            self.assertTrue(summary['connector_requirements']['accepted'])
                            self.assertEqual(len(rows), 1)
                            self.assertEqual(rows[0]['candidate_sha256'], sha(output))
                            board = pcbnew.LoadBoard(str(output))
                            usb = next(f for f in board.GetFootprints() if f.GetReference() == 'USB1')
                            self.assertEqual([pcbnew.ToMM(usb.GetPosition().x),
                                              pcbnew.ToMM(usb.GetPosition().y),
                                              usb.GetOrientationDegrees() % 360], [117.5, 100., 180.])
                        else:
                            self.assertEqual(rows, [])
                            self.assertEqual(sha(output), before)
                            self.assertFalse(summary['published'])
                            self.assertFalse(summary['complete'])
                            if mode == 'dry':
                                self.assertEqual(summary['status'], 'dry_run')
                                self.assertIsNone(summary['connector_requirements']['accepted'])
                                self.assertFalse(summary['engineering_clean'])
                            else:
                                self.assertEqual(result.returncode, 4)
                                self.assertFalse(summary['connector_requirements']['accepted'])


if __name__ == '__main__':
    unittest.main()
