"""Recoverable publication of a finished placement board.

Cooperating publishers serialize via exclusive directory creation. A journal
and backups survive an uncatchable process death or failed rollback; subsequent
writers refuse until an operator reconciles that evidence. This is not a
multi-file atomic read, a security boundary, or a power-loss durability promise.
"""
from __future__ import annotations

import json
import os
import shutil
import stat
import tempfile

from placement import provenance as pv


class PublicationError(OSError):
    def __init__(self, message, *, output_state='unchanged', rollback_errors=None,
                 recovery_journal=None):
        super().__init__(message)
        self.details = dict(output_state=output_state,
                            rollback_errors=rollback_errors or [],
                            recovery_journal=recovery_journal)


def input_identity(board, siblings=True):
    from copy_board import SIBLING_EXTS
    paths = [board]
    if siblings:
        paths += [os.path.splitext(board)[0] + ext for ext in SIBLING_EXTS]
    return {os.path.abspath(p): pv.sha256_file(p) if os.path.isfile(p) else None
            for p in paths}


def publish_board(staged, output, *, input_file=None, siblings=True,
                  expected_input=None):
    """Authorize, snapshot, publish and commit; roll back on catchable failure.

The caller must finish geometry checks before entry. This function does not
choose or legalize any pose. `expected_input` detects a cooperating writer
changing the board/requirements while the caller computed its candidate.
    """
    from copy_board import SIBLING_EXTS
    staged, output = os.path.abspath(staged), os.path.abspath(output)
    input_file = os.path.abspath(input_file or staged)
    root = pv.regime_for(output)
    lock_paths = [output + '.krt-publish-lock']
    if root:
        lock_paths.insert(0, os.path.join(root, '.pose-publication-lock'))
    acquired = []
    backups, temps, attempted = {}, [], []
    prior_pending = pv._PENDING.get(pv._key(output))
    retain = False
    committed = False
    journal = None
    pairs = [(staged, output)]
    try:
        # No destination board or sibling is touched before authorization.
        # record_write also repeats this check when preparing the final row.
        if root:
            lever = pv.active_lever()
            if not lever or lever['lever'] not in pv.LEVER_REGISTRY:
                raise pv.UnaidedViolation(
                    'publication refused before writing: no registered lever '
                    'in LEVER_REGISTRY for armed destination %s; declared=%r; caller=%s' % (
                        output, (lever or {}).get('lever'), pv._caller()))
        for lock in lock_paths:
            try:
                os.mkdir(lock)
            except FileExistsError as exc:
                raise PublicationError('publication busy or recovery required: ' + lock,
                                       recovery_journal=lock) from exc
            acquired.append(lock)
        journal = os.path.join(acquired[0], 'journal.json')
        if expected_input is not None:
            now = {p: pv.sha256_file(p) if os.path.isfile(p) else None
                   for p in expected_input}
            if now != expected_input:
                raise OSError('input board or requirement siblings changed while candidate was prepared')
        preserve_baseline = None
        if root:
            with open(os.path.join(root, pv.REGIME_NAME), encoding='utf-8') as f:
                regime = json.load(f)
            baseline = regime.get('staged_board')
            lever = pv.active_lever()
            if baseline and pv._key(baseline) == pv._key(output) and (
                    lever['lever'] not in pv.FENCE_SENSITIVE_LEVERS):
                if pv.sha256_file(baseline) != regime.get('staged_sha256'):
                    raise OSError('armed regime baseline identity changed; cannot preserve it')
                preserve_baseline = regime
        src_base, dst_base = os.path.splitext(staged)[0], os.path.splitext(output)[0]
        if siblings:
            extra = [dst_base + ext for ext in SIBLING_EXTS if ext != '.kicad_prl'
                     and os.path.exists(dst_base + ext) and not os.path.isfile(src_base + ext)]
            if extra:
                raise OSError('output has requirement siblings absent from the graded input: '
                              + ', '.join(extra))
            pairs += [(src_base + ext, dst_base + ext) for ext in SIBLING_EXTS
                      if os.path.isfile(src_base + ext)]
        if preserve_baseline is not None:
            # In-place on the original staged filename must not redefine the
            # audit's baseline. Preserve its exact bytes and declarations and
            # retarget the manifest as part of this same recoverable write.
            import uuid
            frozen = os.path.join(root, '.pose-baseline-' + uuid.uuid4().hex + '.kicad_pcb')
            pairs.append((output, frozen))
            for ext in SIBLING_EXTS:
                src = os.path.splitext(output)[0] + ext
                if os.path.isfile(src):
                    pairs.append((src, os.path.splitext(frozen)[0] + ext))
            manifest_candidate = os.path.join(acquired[0], 'manifest.json')
            preserve_baseline['staged_board'] = frozen
            with open(manifest_candidate, 'w', encoding='utf-8') as f:
                json.dump(preserve_baseline, f, indent=1, sort_keys=True)
            pairs.append((manifest_candidate, os.path.join(root, pv.REGIME_NAME)))
        targets = [dst for _, dst in pairs]
        if root:
            targets.append(os.path.join(root, pv.LEDGER_NAME))
        for dst in targets:
            if os.path.lexists(dst) and (not os.path.isfile(dst) or os.path.islink(dst)):
                raise OSError('destination is not a regular file: ' + dst)
        # Unique files; no shared .krt-tmp name. Everything is prepared before
        # any replacement. Backups include the ledger, including its absence.
        for src, dst in pairs:
            fd, tmp = tempfile.mkstemp(prefix='.krt-candidate-', dir=os.path.dirname(dst))
            os.close(fd)
            temps.append((tmp, dst))
            shutil.copyfile(src, tmp)
        for dst in targets:
            backups[dst] = None
            if os.path.exists(dst):
                fd, backup = tempfile.mkstemp(prefix='.krt-backup-', dir=os.path.dirname(dst))
                os.close(fd)
                backups[dst] = backup
                shutil.copy2(dst, backup)
        candidate = temps[0][0]
        row = pv.record_write(input_file, output, [], pending=True, candidate_file=candidate)
        state = dict(output=output, input=input_file, candidate_sha256=pv.sha256_file(candidate),
                     backups=backups, candidates=dict(temps), phase='prepared')
        with open(journal, 'w', encoding='utf-8') as f:
            json.dump(state, f, indent=2, sort_keys=True)
            f.flush()
            os.fsync(f.fileno())
        # Siblings first, board last, ledger after board. Readers that ignore
        # the journal can see intermediate states; audit explicitly refuses.
        for tmp, dst in reversed(temps):
            attempted.append(dst)  # before syscall: an interrupt can follow its success
            os.replace(tmp, dst)
        if root:
            attempted.append(os.path.join(root, pv.LEDGER_NAME))
            pv.commit_write(output)
        committed = True
        return row
    except BaseException as exc:
        errors = []
        for dst in reversed(attempted):
            try:
                backup = backups[dst]
                if backup is None:
                    if os.path.exists(dst):
                        os.remove(dst)
                elif os.path.isfile(dst) and pv.sha256_file(dst) == pv.sha256_file(backup):
                    pass  # syscall failed without mutation (e.g. Windows read-only output)
                else:
                    os.replace(backup, dst)
                    backups[dst] = None
            except BaseException as restore_error:
                errors.append(dict(path=dst, error=str(restore_error), backup=backups[dst]))
        # record_write can be interrupted after storing the row but before
        # returning. Cancel by ownership identity, not a post-call flag.
        if pv._PENDING.get(pv._key(output)) is not prior_pending:
            pv.cancel_write(output)
        retain = bool(errors)
        if errors:
            raise PublicationError(
                'Output partially changed; restoration failed. Retained recovery evidence: '
                + str(journal), output_state='partial', rollback_errors=errors,
                recovery_journal=journal) from exc
        if isinstance(exc, (pv.UnaidedViolation, PublicationError)):
            raise
        if not isinstance(exc, Exception):
            raise
        raise PublicationError('cannot write %s: %s. Nothing was written: previous output '
                               'files were preserved or restored.' % (output, exc)) from exc
    finally:
        if not retain:
            # Remove the exclusion marker before discarding backups. Failure
            # here is AFTER commit/rollback and must disclose which occurred.
            try:
                if journal and os.path.exists(journal):
                    os.remove(journal)
                if acquired:
                    manifest_candidate = os.path.join(acquired[0], 'manifest.json')
                    if os.path.exists(manifest_candidate):
                        os.remove(manifest_candidate)
                for lock in reversed(acquired):
                    os.rmdir(lock)
            except OSError as cleanup_error:
                evidence = dict(output=output, phase='committed' if committed else 'rolled_back',
                                backups=backups, cleanup_error=str(cleanup_error))
                if journal and os.path.isdir(os.path.dirname(journal)):
                    try:
                        with open(journal, 'w', encoding='utf-8') as f:
                            json.dump(evidence, f, indent=2)
                    except OSError:
                        pass
                raise PublicationError(
                    'Publication %s; cleanup failed; retained backups: %s; %s' % (
                        evidence['phase'], json.dumps(backups), cleanup_error),
                    output_state='committed' if committed else 'unchanged',
                    recovery_journal=journal) from cleanup_error
            for path in [p for p in backups.values() if p] + [p for p, _ in temps]:
                try:
                    if os.path.exists(path):
                        os.chmod(path, os.stat(path).st_mode | stat.S_IWUSR)
                        os.remove(path)
                except OSError:
                    pass
