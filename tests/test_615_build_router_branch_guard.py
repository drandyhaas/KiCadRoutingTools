"""
Tests for build_router's branch guard (issue #615).

Prebuilt release assets are built from main's crate. A checkout whose
rust_router/ differs from main (the `placement` branch at crate 0.20.0, or a
dirty crate on any branch) can download a binary that satisfies -- or, worse,
after a hand-bump of Cargo.toml, *passes* -- the startup version gate while
carrying a different ABI: #615's reporter got `TypeError: ...
add_stub_proximity_costs_batch takes 3 positional arguments but 4 were given`
on every real run. build_router now detects the divergence with git and goes
straight to a source build; release-zip installs (no git) and explicit --tag
requests keep the old download path.
"""

import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import build_router


def _git(repo, *argv):
    return subprocess.run(
        ['git', '-C', str(repo),
         '-c', 'user.email=test@test', '-c', 'user.name=test'] + list(argv),
        capture_output=True, text=True, check=True)


def _make_repo(tmp_path):
    """A git repo with a committed rust_router/ crate on branch main."""
    repo = tmp_path / 'repo'
    src = repo / 'rust_router' / 'src'
    src.mkdir(parents=True)
    (repo / 'rust_router' / 'Cargo.toml').write_text(
        '[package]\nname = "grid_router"\nversion = "0.20.0"\n')
    (src / 'lib.rs').write_text('// v1\n')
    (repo / '.gitignore').write_text('*.so\n*.pyd\nrust_*/target/\n')
    _git(repo, 'init', '-q', '-b', 'main')
    _git(repo, 'add', '-A')
    _git(repo, 'commit', '-q', '-m', 'crate')
    return repo


def test_clean_main_checkout_matches(tmp_path):
    repo = _make_repo(tmp_path)
    assert build_router._crate_matches_release_source(str(repo)) is True


def test_branch_with_crate_changes_differs(tmp_path):
    repo = _make_repo(tmp_path)
    _git(repo, 'checkout', '-q', '-b', 'placement')
    (repo / 'rust_router' / 'src' / 'lib.rs').write_text('// block_vias era\n')
    _git(repo, 'commit', '-qam', 'crate change on branch')
    assert build_router._crate_matches_release_source(str(repo)) is False


def test_python_only_branch_still_matches(tmp_path):
    repo = _make_repo(tmp_path)
    _git(repo, 'checkout', '-q', '-b', 'py-feature')
    (repo / 'route.py').write_text('# python-only change\n')
    _git(repo, 'add', '-A')
    _git(repo, 'commit', '-qm', 'python change only')
    assert build_router._crate_matches_release_source(str(repo)) is True


def test_dirty_tracked_crate_edit_differs(tmp_path):
    repo = _make_repo(tmp_path)
    (repo / 'rust_router' / 'src' / 'lib.rs').write_text('// uncommitted edit\n')
    assert build_router._crate_matches_release_source(str(repo)) is False


def test_untracked_crate_file_differs(tmp_path):
    repo = _make_repo(tmp_path)
    (repo / 'rust_router' / 'src' / 'extra.rs').write_text('// new file\n')
    assert build_router._crate_matches_release_source(str(repo)) is False


def test_gitignored_build_outputs_do_not_trip_it(tmp_path):
    repo = _make_repo(tmp_path)
    (repo / 'rust_router' / 'grid_router.so').write_text('binary')
    target = repo / 'rust_router' / 'target' / 'release'
    target.mkdir(parents=True)
    (target / 'libgrid_router.dylib').write_text('binary')
    assert build_router._crate_matches_release_source(str(repo)) is True


def test_prefers_origin_main_over_stale_local_main(tmp_path):
    """A clone that checked out a branch has origin/main but maybe no local
    main; and a stale local main must not mask upstream crate changes."""
    repo = _make_repo(tmp_path)
    _git(repo, 'update-ref', 'refs/remotes/origin/main', 'main')
    _git(repo, 'checkout', '-q', '-b', 'feature')
    _git(repo, 'branch', '-q', '-D', 'main')
    assert build_router._crate_matches_release_source(str(repo)) is True
    (repo / 'rust_router' / 'src' / 'lib.rs').write_text('// diverged\n')
    _git(repo, 'commit', '-qam', 'diverge from origin/main')
    assert build_router._crate_matches_release_source(str(repo)) is False


def test_no_git_checkout_returns_none(tmp_path):
    plain = tmp_path / 'zip-install'
    plain.mkdir()
    assert build_router._crate_matches_release_source(str(plain)) is None


def test_repo_without_main_ref_returns_none(tmp_path):
    repo = _make_repo(tmp_path)
    _git(repo, 'checkout', '-q', '-b', 'only-branch')
    _git(repo, 'branch', '-q', '-D', 'main')
    assert build_router._crate_matches_release_source(str(repo)) is None


def _run_main(monkeypatch, argv, crate_matches):
    """Drive build_router.main() with the network/build stages stubbed out.

    Returns (downloads, builds) call counts.
    """
    calls = {'download': 0, 'build': 0}
    monkeypatch.setattr(build_router, '_crate_matches_release_source',
                        lambda script_dir: crate_matches)
    monkeypatch.setattr(build_router, 'try_download_prebuilt',
                        lambda *a: calls.__setitem__('download', calls['download'] + 1) or True)
    monkeypatch.setattr(build_router, 'build_from_source',
                        lambda *a: calls.__setitem__('build', calls['build'] + 1))
    # Post-download self-heal comparison: pretend versions agree.
    monkeypatch.setattr(build_router, '_cargo_version', lambda rust_dir: '0.20.0')
    monkeypatch.setattr(build_router, '_installed_version', lambda rust_dir: '0.20.0')
    monkeypatch.setattr(sys, 'argv', ['build_router.py'] + argv)
    build_router.main()
    return calls['download'], calls['build']


def test_main_goes_straight_to_source_when_crate_diverges(monkeypatch):
    downloads, builds = _run_main(monkeypatch, [], crate_matches=False)
    assert (downloads, builds) == (0, 1)


def test_main_downloads_when_crate_matches(monkeypatch):
    downloads, builds = _run_main(monkeypatch, [], crate_matches=True)
    assert (downloads, builds) == (1, 0)


def test_main_downloads_when_git_cannot_tell(monkeypatch):
    downloads, builds = _run_main(monkeypatch, [], crate_matches=None)
    assert (downloads, builds) == (1, 0)


def test_explicit_tag_overrides_the_guard(monkeypatch):
    downloads, builds = _run_main(monkeypatch, ['--tag', 'v0.20.1'],
                                  crate_matches=False)
    assert (downloads, builds) == (1, 0)


if __name__ == '__main__':
    # run_all.py runs each test file as a script. Without this block the file
    # defined its tests and ran none of them, and run_all reported a pass. The
    # two pytest fixtures are supplied from the stdlib so pytest stays
    # optional; under pytest the functions still receive pytest's own.
    import inspect
    import pathlib
    import shutil
    import stat
    import tempfile
    import traceback

    def _rmtree(path):
        """Remove a fixture dir that holds a git repo. Git writes its object
        files read-only, and on Windows rmtree cannot unlink a read-only file
        -- ignore_errors=True left every repo behind in TEMP, one per test per
        run. Clear the bit and retry; say so if anything still survives."""
        def _writable_retry(func, p, _exc):
            os.chmod(p, stat.S_IWRITE)
            func(p)
        handler = ({'onexc': _writable_retry} if sys.version_info >= (3, 12)
                   else {'onerror': _writable_retry})
        try:
            shutil.rmtree(path, **handler)
        except OSError as exc:
            print(f'  (could not remove {path}: {exc})')

    class _MonkeyPatch:
        """The one fixture method these tests use: setattr, undone after."""

        def __init__(self):
            self._undo = []

        def setattr(self, target, name, value):
            self._undo.append((target, name, getattr(target, name)))
            setattr(target, name, value)

        def undo(self):
            for target, name, value in reversed(self._undo):
                setattr(target, name, value)

    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith('test_') and inspect.isfunction(f)]
    failed = []
    for name, fn in tests:
        params = inspect.signature(fn).parameters
        tmp, mp = tempfile.mkdtemp(prefix='t615_'), _MonkeyPatch()
        kwargs = {}
        if 'tmp_path' in params:
            kwargs['tmp_path'] = pathlib.Path(tmp)
        if 'monkeypatch' in params:
            kwargs['monkeypatch'] = mp
        try:
            fn(**kwargs)
            print(f'  PASS  {name}')
        except Exception:                                  # noqa: BLE001
            failed.append(name)
            print(f'  FAIL  {name}')
            traceback.print_exc()
        finally:
            mp.undo()
            _rmtree(tmp)
    print(f'{len(tests) - len(failed)}/{len(tests)} passed')
    sys.exit(1 if failed or not tests else 0)
