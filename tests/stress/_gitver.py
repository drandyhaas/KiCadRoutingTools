"""Record the git version of the code under test, for result provenance.

Stress-test output dirs otherwise carry no trace of WHICH commit produced them,
which makes an A/B comparison ambiguous after the fact ("was this wave HEAD or
HEAD~5?"). `write_git_version()` drops a `git_version.txt` (+ machine-readable
`git_version.json`) into an output dir naming the exact checkout, so every
results/wave dir self-documents the code that created it.
"""
import datetime
import json
import subprocess
from pathlib import Path


def git_version(repo):
    """Provenance dict for the checkout at `repo`.

    Every field is None (and `dirty` False) if `repo` is not a git repo or git
    is unavailable -- recording provenance must never break a stress run.
    """
    def g(*args):
        try:
            return subprocess.check_output(
                ["git", "-C", str(repo), *args],
                text=True, stderr=subprocess.DEVNULL).strip()
        except Exception:
            return None

    return {
        "describe": g("describe", "--tags", "--always", "--dirty"),
        "commit": g("rev-parse", "HEAD"),
        "branch": g("rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": bool(g("status", "--porcelain")),
        "subject": g("log", "-1", "--pretty=%s"),
        "committed_at": g("log", "-1", "--date=iso", "--pretty=%cd"),
    }


def write_git_version(out_dir, repo, label=None):
    """Write git_version.txt + git_version.json into out_dir; return the dict."""
    ver = git_version(repo)
    if label:
        ver["label"] = label
    ver["captured_at"] = datetime.datetime.now().isoformat(timespec="seconds")

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    out.joinpath("git_version.json").write_text(json.dumps(ver, indent=2))

    lines = ([f"label:     {label}"] if label else []) + [
        f"describe:  {ver['describe']}",
        f"commit:    {ver['commit']}",
        f"branch:    {ver['branch']}",
        f"dirty:     {ver['dirty']}",
        f"subject:   {ver['subject']}",
        f"committed: {ver['committed_at']}",
        f"captured:  {ver['captured_at']}",
    ]
    out.joinpath("git_version.txt").write_text("\n".join(lines) + "\n")
    return ver


def load_git_version(near_path):
    """Read git_version.json sitting next to `near_path` (a file or dir); None if
    absent. Used by the compare view to annotate each wave with its checkout."""
    p = Path(near_path)
    base = p if p.is_dir() else p.parent
    f = base / "git_version.json"
    if not f.exists():
        return None
    try:
        return json.loads(f.read_text())
    except Exception:
        return None


def format_version(ver):
    """One-line summary for logs / compare headers ('label=v0.17.3-5-gabc1234')."""
    if not ver or not ver.get("describe"):
        return "unknown"
    return f"{ver['label']}={ver['describe']}" if ver.get("label") else ver["describe"]
