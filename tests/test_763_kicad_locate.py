"""Finding KiCad on any drive, any version, any platform (#763).

The reported defects, all three of which were invisible to the test suite
because the platform they occur on is not the platform this repo is developed
on:

1. **Drive C: was hardcoded.** `install_plugin.py` carried a literal list of
   `C:\\Program Files\\KiCad\\<ver>\\bin\\python.exe` paths and
   `kicad_exact_fill.kicad_python_candidates()` globbed the same two roots. A
   KiCad on D: matched neither, so the installer announced "no KiCad version",
   fell back to the system python (which cannot import pcbnew), and every
   exact-fill consumer degraded silently.

2. **The failure was silent.** An empty version list printed
   `KiCad versions found: ` and failed at the end with only an unrelated
   symlink hint -- from the outside, a bad SEARCH and a bad INSTALL look
   identical.

3. **OneDrive's decoy Documents won.** The Windows branch returned
   `%OneDrive%\\Documents\\KiCad` whenever it merely EXISTED. OneDrive creates
   that folder on machines where the Documents known folder is NOT redirected
   into it, so the plugin landed where KiCad never looks.

Everything here SIMULATES the platform layouts -- injected `isdir`/`glob`/
`environ`/registry seams -- because a Windows drive scan is otherwise
untestable on macOS/Linux, which is precisely how these shipped. Every
assertion is behavioural: an earlier version of this file grepped the module
source for 'Program Files (x86)', which passes just as happily when the string
sits in a comment (the PR692 lesson).

No pcbnew, no subprocess, milliseconds.
"""
import fnmatch
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

import kicad_locate as kl  # noqa: E402

RUN_ALL_FAST_OK = True


# ---------------------------------------------------------------------------
# a fake filesystem: a set of directories, matched component-wise
# ---------------------------------------------------------------------------

class FakeFS:
    """Directories that 'exist', with a glob that respects separators.

    fnmatch alone is wrong here: its `*` spans separators, so
    `D:\\Program Files\\KiCad\\*` would also match `...\\KiCad\\10.0\\bin` and
    a one-level glob would silently become a recursive one.
    """

    def __init__(self, dirs):
        self.dirs = {self._norm(d) for d in dirs}

    @staticmethod
    def _norm(p):
        return p.replace('/', '\\') if ':' in p[:3] else p

    @staticmethod
    def _split(p):
        return [c for c in p.replace('/', '\\').split('\\') if c]

    def isdir(self, p):
        return self._norm(p) in self.dirs

    def glob(self, pattern):
        pat = self._split(self._norm(pattern))
        out = []
        for d in self.dirs:
            parts = self._split(d)
            if len(parts) != len(pat):
                continue
            if all(fnmatch.fnmatch(a, b) for a, b in zip(parts, pat)):
                out.append(d)
        return sorted(out)


def _drives(*letters):
    return ['{}:\\'.format(c) for c in letters]


# ---------------------------------------------------------------------------
# 1. the reported defect: KiCad is not on C:
# ---------------------------------------------------------------------------

def test_kicad_on_drive_d_is_found():
    """#763's headline. KiCad on D:, nothing on C:."""
    fs = FakeFS([r'C:\Program Files',
                 r'D:\Program Files',
                 r'D:\Program Files\KiCad',
                 r'D:\Program Files\KiCad\10.0',
                 r'D:\Program Files\KiCad\10.0\bin'])
    roots = kl.kicad_install_roots(
        system='Windows', environ={'ProgramFiles': r'C:\Program Files'},
        globber=fs.glob, isdir=fs.isdir, drives=_drives('C', 'D'),
        winreg_mod=False)
    assert r'D:\Program Files\KiCad\10.0' in roots, roots
    pys = kl.kicad_python_candidates(
        system='Windows', environ={'ProgramFiles': r'C:\Program Files'},
        globber=fs.glob, isdir=fs.isdir, drives=_drives('C', 'D'),
        winreg_mod=False)
    assert r'D:\Program Files\KiCad\10.0\bin\python.exe' in pys, pys
    print("  PASS: KiCad on D: is found (the #763 report)")


def test_no_c_drive_at_all():
    """A machine with no C: KiCad must not depend on C: existing.

    NEGATIVE CONTROL for the hardcoded-path bug: the old code's entire Windows
    candidate list was rooted at C:, so this configuration returned nothing.
    """
    fs = FakeFS([r'E:\Program Files (x86)',
                 r'E:\Program Files (x86)\KiCad',
                 r'E:\Program Files (x86)\KiCad\9.0',
                 r'E:\Program Files (x86)\KiCad\9.0\bin'])
    roots = kl.kicad_install_roots(
        system='Windows', environ={}, globber=fs.glob, isdir=fs.isdir,
        drives=_drives('E'), winreg_mod=False)
    assert roots == [r'E:\Program Files (x86)\KiCad\9.0'], roots
    print("  PASS: discovery works with no C: drive present")


def test_portable_install_at_drive_root():
    """A hand-extracted D:\\KiCad\\10.0 (no 'Program Files' component)."""
    fs = FakeFS([r'D:\KiCad', r'D:\KiCad\10.0', r'D:\KiCad\10.0\bin'])
    roots = kl.kicad_install_roots(
        system='Windows', environ={}, globber=fs.glob, isdir=fs.isdir,
        drives=_drives('D'), winreg_mod=False)
    assert r'D:\KiCad\10.0' in roots, roots
    print("  PASS: a portable install at a drive root is found")


def test_program_files_env_var_beats_the_drive_scan():
    """A redirected Program Files root is followed, and comes FIRST.

    The scan is the backstop; the environment is the machine's own answer.
    """
    fs = FakeFS([r'X:\Apps', r'X:\Apps\KiCad', r'X:\Apps\KiCad\10.0',
                 r'C:\Program Files', r'C:\Program Files\KiCad',
                 r'C:\Program Files\KiCad\9.0'])
    roots = kl.kicad_install_roots(
        system='Windows', environ={'ProgramW6432': r'X:\Apps'},
        globber=fs.glob, isdir=fs.isdir, drives=_drives('C'),
        winreg_mod=False)
    assert roots[0] == r'X:\Apps\KiCad\10.0', roots
    print("  PASS: ProgramW6432 is honoured and outranks the drive scan")


# ---------------------------------------------------------------------------
# 2. versions: globbed, never a fixed list; sorted numerically
# ---------------------------------------------------------------------------

def test_future_version_is_found_without_a_code_change():
    """KiCad 11 must be found by a build that predates it (#763 'versions')."""
    fs = FakeFS([r'C:\Program Files', r'C:\Program Files\KiCad',
                 r'C:\Program Files\KiCad\11.0',
                 r'C:\Program Files\KiCad\10.0'])
    roots = kl.kicad_install_roots(
        system='Windows', environ={'ProgramFiles': r'C:\Program Files'},
        globber=fs.glob, isdir=fs.isdir, drives=_drives('C'), winreg_mod=False)
    assert roots[0] == r'C:\Program Files\KiCad\11.0', roots
    assert '11.0' not in kl.KNOWN_VERSIONS, \
        "KNOWN_VERSIONS must stay a preference list, not a gate"
    print("  PASS: an unknown future version is found and sorts newest-first")


def test_numeric_not_string_version_order():
    """'9.0' string-sorts ABOVE '10.0'; that hands a KiCad 10 user KiCad 9."""
    assert kl.version_key('10.0') > kl.version_key('9.0')
    assert kl.path_version_key(r'C:\Program Files\KiCad\10.0\bin\python.exe') \
        == (10, 0)
    # separator-agnostic: an os.path-based key collapses off-Windows
    assert kl.path_version_key('C:/Program Files/KiCad/10.0/bin/python.exe') \
        == (10, 0)
    # unversioned sorts below every real version, so it is never a winner
    assert kl.path_version_key(r'C:\Program Files\KiCad\bin\python.exe') \
        < kl.path_version_key(r'C:\Program Files\KiCad\9.0\bin\python.exe')
    print("  PASS: versions order numerically")


def test_user_versions_are_globbed():
    fs = FakeFS([r'C:\Users\x\Documents\KiCad',
                 r'C:\Users\x\Documents\KiCad\9.0',
                 r'C:\Users\x\Documents\KiCad\10.0',
                 r'C:\Users\x\Documents\KiCad\11.0',
                 r'C:\Users\x\Documents\KiCad\templates'])
    vs = kl.kicad_user_versions(r'C:\Users\x\Documents\KiCad',
                                globber=fs.glob, isdir=fs.isdir)
    assert vs == ['11.0', '10.0', '9.0'], vs   # 'templates' is not a version
    print("  PASS: user version dirs are globbed, newest first")


# ---------------------------------------------------------------------------
# 3. the registry path the reporter suggested
# ---------------------------------------------------------------------------

class FakeWinreg:
    """Just enough winreg to serve one KiCad.kicad_pcb.<ver> open command."""
    HKEY_CLASSES_ROOT = 'HKCR'
    HKEY_LOCAL_MACHINE = 'HKLM'
    HKEY_CURRENT_USER = 'HKCU'

    def __init__(self, keys):
        self.keys = keys                 # {(hive, subkey): value-or-list}

    class _K:
        def __init__(self, outer, hive, sub):
            self.outer, self.hive, self.sub = outer, hive, sub

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def Close(self):
            pass

    def OpenKey(self, hive, sub):
        if (hive, sub) not in self.keys:
            raise OSError('missing {} {}'.format(hive, sub))
        return self._K(self, hive, sub)

    def EnumKey(self, k, i):
        v = self.keys[(k.hive, k.sub)]
        if not isinstance(v, list) or i >= len(v):
            raise OSError('no more')
        return v[i]

    def QueryValueEx(self, k, name):
        v = self.keys[(k.hive, k.sub)]
        if isinstance(v, list):
            raise OSError('not a value')
        return (v, 1)


def test_registry_lookup_finds_a_non_c_install():
    """HKCR\\KiCad.kicad_pcb.10.0\\shell\\open\\command -> the version dir."""
    reg = FakeWinreg({
        ('HKCR', ''): ['KiCad.kicad_pcb.10.0', 'SomethingElse'],
        ('HKCR', r'KiCad.kicad_pcb.10.0\shell\open\command'):
            r'"D:\Programs\KiCad\10.0\bin\pcbnew.exe" "%1"',
    })
    fs = FakeFS([r'D:\Programs\KiCad\10.0'])
    roots = kl.kicad_install_roots(
        system='Windows', environ={}, globber=fs.glob, isdir=fs.isdir,
        drives=[], winreg_mod=reg)
    assert roots == [r'D:\Programs\KiCad\10.0'], roots
    print("  PASS: the registry open-command locates a non-C: install")


def test_registry_outranks_the_scan():
    """When both answer, the registry's install is preferred."""
    reg = FakeWinreg({
        ('HKCR', ''): ['KiCad.kicad_pcb.10.0'],
        ('HKCR', r'KiCad.kicad_pcb.10.0\shell\open\command'):
            r'"D:\Programs\KiCad\10.0\bin\pcbnew.exe" "%1"',
    })
    fs = FakeFS([r'D:\Programs\KiCad\10.0',
                 r'C:\Program Files', r'C:\Program Files\KiCad',
                 r'C:\Program Files\KiCad\9.0'])
    roots = kl.kicad_install_roots(
        system='Windows', environ={'ProgramFiles': r'C:\Program Files'},
        globber=fs.glob, isdir=fs.isdir, drives=_drives('C'), winreg_mod=reg)
    assert roots[0] == r'D:\Programs\KiCad\10.0', roots
    print("  PASS: the registry outranks the drive scan")


def test_registry_absence_is_not_fatal():
    """No winreg (or a hive that raises) must degrade, never explode."""
    assert kl._winreg_kicad_roots(FakeWinreg({})) == []
    fs = FakeFS([r'C:\Program Files\KiCad\10.0'])
    roots = kl.kicad_install_roots(
        system='Windows', environ={}, globber=fs.glob, isdir=fs.isdir,
        drives=_drives('C'), winreg_mod=FakeWinreg({}))
    assert roots == [r'C:\Program Files\KiCad\10.0'], roots
    print("  PASS: a missing/broken registry degrades to the scan")


# ---------------------------------------------------------------------------
# 4. the OneDrive decoy -- the reporter's second bug
# ---------------------------------------------------------------------------

def test_onedrive_decoy_documents_does_not_win():
    """The #763 mis-install.

    Documents is NOT redirected (the known folder is the local one), but
    OneDrive has made its own `Documents` folder. The old code returned the
    OneDrive path because it existed; KiCad reads the local one.
    """
    fs = FakeFS([r'C:\Users\gg\Documents',
                 r'C:\Users\gg\Documents\KiCad',
                 r'C:\Users\gg\Documents\KiCad\10.0',
                 r'C:\Users\gg\OneDrive\Documents',
                 r'C:\Users\gg\OneDrive\Documents\KiCad'])
    base = kl.kicad_user_base(
        system='Windows',
        environ={'OneDrive': r'C:\Users\gg\OneDrive'},
        isdir=fs.isdir, globber=fs.glob, home=r'C:\Users\gg',
        documents=r'C:\Users\gg\Documents')
    assert base == r'C:\Users\gg\Documents\KiCad', base
    print("  PASS: an empty OneDrive Documents does not beat the real one")


def test_genuinely_redirected_documents_is_honoured():
    """When Documents IS redirected, the OneDrive path is the right answer."""
    fs = FakeFS([r'C:\Users\gg\OneDrive\Documents',
                 r'C:\Users\gg\OneDrive\Documents\KiCad',
                 r'C:\Users\gg\OneDrive\Documents\KiCad\10.0'])
    base = kl.kicad_user_base(
        system='Windows',
        environ={'OneDrive': r'C:\Users\gg\OneDrive'},
        isdir=fs.isdir, globber=fs.glob, home=r'C:\Users\gg',
        # the known folder resolves INTO OneDrive on a redirected machine
        documents=r'C:\Users\gg\OneDrive\Documents')
    assert base == r'C:\Users\gg\OneDrive\Documents\KiCad', base
    print("  PASS: a genuinely redirected Documents is honoured")


def test_evidence_beats_mere_existence():
    """A base holding a VERSION dir wins over one that merely exists.

    This is the tie-break that makes the decoy case above work without having
    to trust any single signal: KiCad's own version directory is proof.
    """
    fs = FakeFS([r'C:\Users\gg\Documents', r'C:\Users\gg\Documents\KiCad',
                 r'C:\Users\gg\OneDrive\Documents',
                 r'C:\Users\gg\OneDrive\Documents\KiCad',
                 r'C:\Users\gg\OneDrive\Documents\KiCad\9.0'])
    base = kl.kicad_user_base(
        system='Windows', environ={'OneDrive': r'C:\Users\gg\OneDrive'},
        isdir=fs.isdir, globber=fs.glob, home=r'C:\Users\gg',
        documents=r'C:\Users\gg\Documents')
    assert base == r'C:\Users\gg\OneDrive\Documents\KiCad', base
    print("  PASS: a base with a version dir beats one that merely exists")


def test_fresh_machine_falls_back_to_where_kicad_would_look():
    """Nothing exists yet: return the known-folder path, not a OneDrive guess."""
    fs = FakeFS([])
    base = kl.kicad_user_base(
        system='Windows', environ={'OneDrive': r'C:\Users\gg\OneDrive'},
        isdir=fs.isdir, globber=fs.glob, home=r'C:\Users\gg',
        documents=r'C:\Users\gg\Documents')
    assert base == r'C:\Users\gg\Documents\KiCad', base
    print("  PASS: a first-ever install lands where KiCad looks")


def test_documents_dir_prefers_the_known_folder_over_env():
    """The known-folder API is authoritative; env vars are the fallback."""
    got = kl.windows_documents_dir(
        environ={'OneDrive': r'C:\Users\gg\OneDrive'},
        known_folder=lambda: r'C:\Users\gg\Documents',
        winreg_mod=False, home=r'C:\Users\gg')
    assert got == r'C:\Users\gg\Documents', got
    # and when the API is unavailable, the registry mirror answers
    reg = FakeWinreg({('HKCU', r'Software\Microsoft\Windows\CurrentVersion'
                               r'\Explorer\User Shell Folders'):
                      r'D:\MyDocs'})
    got = kl.windows_documents_dir(environ={}, known_folder=lambda: None,
                                   winreg_mod=reg, home=r'C:\Users\gg')
    assert got == r'D:\MyDocs', got
    print("  PASS: Documents comes from the OS, not from a guess")


# ---------------------------------------------------------------------------
# 5. the other platforms
# ---------------------------------------------------------------------------

def test_macos_sibling_launchers_are_not_candidates():
    """/Applications/KiCad holds GerbView.app etc; none ships a python.

    They were reaching the candidate list, putting ~12 dead paths in front of
    every caller -- and find_kicad_python() PROBES each candidate.
    """
    fs = FakeFS(['/Applications/KiCad/KiCad.app',
                 '/Applications/KiCad/GerbView.app',
                 '/Applications/KiCad/PCB Calculator.app',
                 '/Applications/KiCad/PCB Editor.app'])
    roots = kl.kicad_install_roots(system='Darwin', environ={},
                                   globber=fs.glob, isdir=fs.isdir,
                                   home='/Users/x')
    assert roots == ['/Applications/KiCad/KiCad.app'], roots
    print("  PASS: macOS sibling launcher bundles are excluded")


def test_macos_side_by_side_versions_newest_first():
    fs = FakeFS(['/Applications/KiCad 9.app', '/Applications/KiCad 10.app'])
    roots = kl.kicad_install_roots(system='Darwin', environ={},
                                   globber=fs.glob, isdir=fs.isdir,
                                   home='/Users/x')
    assert roots[0] == '/Applications/KiCad 10.app', roots
    print("  PASS: side-by-side macOS bundles sort newest-first")


def test_macos_user_applications_install():
    """A bundle in ~/Applications (a non-admin install) is found."""
    fs = FakeFS(['/Users/x/Applications/KiCad/KiCad.app'])
    roots = kl.kicad_install_roots(system='Darwin', environ={},
                                   globber=fs.glob, isdir=fs.isdir,
                                   home='/Users/x')
    assert roots == ['/Users/x/Applications/KiCad/KiCad.app'], roots
    print("  PASS: a ~/Applications install is found")


def test_linux_flatpak_and_snap():
    """Linux used to return None unconditionally -- 'check for flatpak/snap'
    was a comment describing code that was never written."""
    fs = FakeFS(['/snap/kicad/current/usr'])
    roots = kl.kicad_install_roots(system='Linux', environ={},
                                   globber=fs.glob, isdir=fs.isdir,
                                   home='/home/x')
    assert '/snap/kicad/current/usr' in roots, roots
    pys = kl.kicad_python_candidates(system='Linux', environ={},
                                     globber=fs.glob, isdir=fs.isdir,
                                     home='/home/x')
    assert '/snap/kicad/current/usr/bin/python3' in pys, pys
    fs2 = FakeFS(['/var/lib/flatpak/app/org.kicad.KiCad/current/active/files'])
    roots2 = kl.kicad_install_roots(system='Linux', environ={},
                                    globber=fs2.glob, isdir=fs2.isdir,
                                    home='/home/x')
    assert roots2, 'flatpak install not found'
    print("  PASS: Linux flatpak/snap installs are found")


def test_unsupported_os_still_raises():
    try:
        kl.kicad_user_bases(system='Plan9', environ={}, home='/home/x')
    except RuntimeError as e:
        assert 'Plan9' in str(e)
        print("  PASS: an unsupported OS raises with its name")
        return
    raise AssertionError('expected RuntimeError')


# ---------------------------------------------------------------------------
# 6. KICAD_PYTHON / KICAD_INSTALL_DIR overrides, and the diagnostics
# ---------------------------------------------------------------------------

def test_env_overrides_come_first():
    fs = FakeFS([r'C:\Program Files\KiCad\10.0'])
    pys = kl.kicad_python_candidates(
        system='Windows', environ={'KICAD_PYTHON': r'Q:\my\python.exe'},
        globber=fs.glob, isdir=fs.isdir, drives=_drives('C'),
        winreg_mod=False)
    assert pys[0] == r'Q:\my\python.exe', pys
    roots = kl.kicad_install_roots(
        system='Windows', environ={'KICAD_INSTALL_DIR': r'Q:\KiCad\10.0'},
        globber=fs.glob, isdir=lambda p: fs.isdir(p) or p == r'Q:\KiCad\10.0',
        drives=[], winreg_mod=False)
    assert roots[0] == r'Q:\KiCad\10.0', roots
    print("  PASS: KICAD_PYTHON / KICAD_INSTALL_DIR override the search")


def test_describe_search_names_what_it_tried():
    """#763's 'fails more or less silently'. The report must name the roots it
    searched -- otherwise a bad search and a bad install look identical."""
    lines = kl.describe_search(
        system='Windows', environ={'ProgramFiles': r'C:\Program Files'},
        drives=_drives('C', 'D'), winreg_mod=FakeWinreg({}),
        home=r'C:\Users\gg')
    blob = '\n'.join(lines)
    assert 'Windows' in blob
    assert 'D:\\' in blob, 'the report must show that D: was searched:\n' + blob
    assert 'registry' in blob.lower()
    assert 'Documents' in blob
    # and it must not explode on the platforms without a registry
    assert kl.describe_search(system='Darwin', environ={})
    assert kl.describe_search(system='Linux', environ={})
    print("  PASS: a failed search reports where it looked")


def test_no_hardcoded_c_drive_survives_in_the_search():
    """CHANGE DETECTOR, behavioural not textual: with no C: among the drives
    and no C: entry in the environment, nothing under C: may be searched."""
    seen = []

    def spy_glob(pattern):
        seen.append(pattern)
        return []

    kl.kicad_install_roots(system='Windows', environ={}, globber=spy_glob,
                           isdir=lambda p: False, drives=_drives('D'),
                           winreg_mod=False)
    stray = [p for p in seen if p.upper().startswith('C:')]
    assert not stray, 'C: is still hardcoded somewhere: {}'.format(stray)
    assert any(p.upper().startswith('D:') for p in seen), seen
    print("  PASS: no C:-rooted path is searched on a C:-less machine")


def main():
    test_kicad_on_drive_d_is_found()
    test_no_c_drive_at_all()
    test_portable_install_at_drive_root()
    test_program_files_env_var_beats_the_drive_scan()
    test_future_version_is_found_without_a_code_change()
    test_numeric_not_string_version_order()
    test_user_versions_are_globbed()
    test_registry_lookup_finds_a_non_c_install()
    test_registry_outranks_the_scan()
    test_registry_absence_is_not_fatal()
    test_onedrive_decoy_documents_does_not_win()
    test_genuinely_redirected_documents_is_honoured()
    test_evidence_beats_mere_existence()
    test_fresh_machine_falls_back_to_where_kicad_would_look()
    test_documents_dir_prefers_the_known_folder_over_env()
    test_macos_sibling_launchers_are_not_candidates()
    test_macos_side_by_side_versions_newest_first()
    test_macos_user_applications_install()
    test_linux_flatpak_and_snap()
    test_unsupported_os_still_raises()
    test_env_overrides_come_first()
    test_describe_search_names_what_it_tried()
    test_no_hardcoded_c_drive_survives_in_the_search()
    print("PASS: KiCad discovery across drives, versions and platforms (#763)")


if __name__ == '__main__':
    main()
