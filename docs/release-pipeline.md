# Release Pipeline

This document describes how a new version of KiCad Routing Tools is built, published on GitHub, and submitted to the official KiCad Plugin and Content Manager (PCM) repository.

The release pipeline is mostly automated by `.github/workflows/release.yml`. A maintainer's job is:

1. Bump the version.
2. Tag and push.
3. Wait for CI to publish the GitHub Release.
4. Submit / update the PCM metadata via a merge request to `gitlab.com/kicad/addons/metadata`.

---

## Components involved

| File | Purpose |
|---|---|
| `VERSION` | Source of truth for the project version (e.g. `0.15.5`). |
| `metadata.json` | KiCad PCM package manifest at repo root. CI patches `download_*` fields after building. |
| `package_pcm.py` | Builds one PCM-compatible zip per version, bundling all 4 prebuilt Rust binaries. |
| `update_metadata.py` | Patches `metadata.json` with sha256 / sizes from `package_pcm.py`'s sidecar. |
| `.github/workflows/release.yml` | CI: builds Rust binaries, runs `package_pcm.py`, publishes GitHub Release. |
| `__init__.py` (root) | `_resolve_rust_binary()` picks the right platform binary at plugin startup. |
| `kicad_routing_plugin/deps_check.py` | First-launch wx dialog that pip-installs missing Python deps (read from `requirements.txt`) into KiCad's Python. |

---

## Versioning rules

- **`VERSION`**: 2- or 3-part dotted decimal, e.g. `0.15.5`. The KiCad PCM regex `^\d{1,4}(\.\d{1,4}(\.\d{1,6})?)?$` rejects 4-part versions.
- **`rust_router/Cargo.toml`**: bumped only when the Rust crate itself changes (see the comment at the top of that file). Python-only changes keep the Rust version constant and only bump `VERSION`.
- Git tag must be `v<VERSION>`, e.g. `v0.15.5`.

---

## Adding a Python dependency

`requirements.txt` at the repo root is the **single source of truth** for the plugin's Python dependencies. It's consumed by three things:

1. `install_plugin.py` — pip-installs everything into KiCad's Python during manual install.
2. `kicad_routing_plugin/deps_check.py` — parses it at plugin startup, checks each package is importable, and offers a wx pip-install dialog for anything missing (the PCM install path skips pip otherwise).
3. CLI scripts (`startup_checks.py`) — fails loudly if a required package isn't importable when running from the terminal.

To add a dependency:

1. Append it to `requirements.txt` (with a version specifier, e.g. `mypkg>=1.2`).
2. (Optional) Add a more thorough import test to the `IMPORT_TESTS` dict in `kicad_routing_plugin/deps_check.py` — by default a plain `import <pkgname>` is used, which is fine for most packages but doesn't catch broken installs of compiled extensions. The existing entries (`scipy`, `shapely`) import specific submodules the plugin actually uses.
3. (Optional) Add it to `startup_checks.check_python_dependencies()` if you also want the CLI scripts to fail with a clearer message.

You do **not** need to update `metadata.json` — Python deps live entirely outside the PCM manifest.

## Cutting a release

### 1. Bump and edit metadata

```bash
# Edit:
#   VERSION                                  -> new x.y.z
#   metadata.json                            -> bump versions[].version to x.y.z
#                                               update download_url path (sha/sizes are CI-patched)
#   rust_router/Cargo.toml (only if Rust changed)
#   rust_router/README.md  (only if Rust changed, version-history table)
```

### 2. Commit and tag

```bash
git add -u
git commit -m "v0.15.5: <short summary>"
git tag -a v0.15.5 -m "v0.15.5 - <short summary>"
git push origin main
git push origin v0.15.5
```

The tag push triggers `.github/workflows/release.yml`.

### 3. Watch the CI

```bash
gh run watch $(gh run list --workflow=release.yml --limit 1 --json databaseId -q '.[0].databaseId')
```

The workflow has three jobs:

1. **`build`** (matrix, ~2–3 min each): builds the Rust binary for `linux-x86_64`, `macos-arm64`, `macos-x86_64`, and `windows-x86_64`. The macOS x86_64 build is cross-compiled from the macOS arm64 runner (cheaper / available capacity than `macos-13`).
2. **`package`** (~1 min): runs `package_pcm.py` to produce one `KiCadRoutingTools-<ver>.zip` bundling all 4 binaries under platform-suffix filenames, then runs `update_metadata.py` to patch the real sha256 / size into `metadata.json`.
3. **`release`**: creates / updates the GitHub Release at `v<VERSION>` with all 4 raw binaries, the PCM zip, and the patched `metadata.json` attached.

When `release` completes successfully, the public Release URL is:

```
https://github.com/drandyhaas/KiCadRoutingTools/releases/tag/v<VERSION>
```

---

## Submitting to the KiCad PCM

The KiCad PCM repository at `gitlab.com/kicad/addons/repository` is **read-only**; it is generated from `gitlab.com/kicad/addons/metadata`. Submissions and updates go to the metadata repo as merge requests.

### One-time prerequisites

1. A GitLab account verified for shared runners (free tier requires identity verification before CI minutes work).
2. `glab` CLI installed and authenticated:

   ```bash
   brew install glab
   glab auth login --hostname gitlab.com   # pick Web auth
   ```

### First submission

1. **Download the patched `metadata.json`** from the GitHub Release:

   ```bash
   curl -fsSL -o /tmp/metadata.json \
     https://github.com/drandyhaas/KiCadRoutingTools/releases/download/v0.15.5/metadata.json
   ```

2. **Fork** `kicad/addons/metadata` (one time):

   ```bash
   glab api -X POST 'projects/kicad%2Faddons%2Fmetadata/fork'
   ```

3. **Clone the fork** and create a feature branch (do not commit to `main`; the validator pipeline rejects pushes to `main`):

   ```bash
   git clone https://gitlab.com/<your-user>/metadata.git /tmp/kicad-addons-fork
   cd /tmp/kicad-addons-fork
   git checkout -b add-com.github.drandyhaas.kicadroutingtools
   ```

4. **Drop in the package files** at `packages/<identifier>/`:

   ```bash
   mkdir -p packages/com.github.drandyhaas.kicadroutingtools
   cp /tmp/metadata.json   packages/com.github.drandyhaas.kicadroutingtools/metadata.json
   cp ~/path/to/icon_64.png packages/com.github.drandyhaas.kicadroutingtools/icon.png
   ```

5. **Validate locally** (catches the same errors the upstream CI catches):

   ```bash
   git remote add target https://gitlab.com/kicad/addons/metadata.git
   git fetch -n target
   pip install jsonschema pillow munch requests tqdm

   export MERGE_BASE_SHA=$(git merge-base target/main HEAD)
   export DIFF_FILES=$(git diff --name-status "${MERGE_BASE_SHA}..HEAD")
   ./ci/validate.sh
   ```

   Both files should report `Validation passed`. If not, fix locally and re-run before pushing.

6. **Commit and push** to your fork:

   ```bash
   git add packages/com.github.drandyhaas.kicadroutingtools/
   git commit -m "Add com.github.drandyhaas.kicadroutingtools v0.15.5"
   git push -u origin add-com.github.drandyhaas.kicadroutingtools
   ```

7. **Open the merge request** against the upstream:

   ```bash
   glab mr create \
     --repo kicad/addons/metadata \
     --target-branch main \
     --source-branch add-com.github.drandyhaas.kicadroutingtools \
     --remove-source-branch \
     --title "Add com.github.drandyhaas.kicadroutingtools v0.15.5" \
     --description "..."
   ```

8. **Smoke-test the package** end-to-end before maintainers review. The successful `build` CI job on the MR exposes a temporary PCM repository URL. Find it with:

   ```bash
   PIPE_ID=<the head_pipeline id from the MR>
   BUILD_JOB=$(glab api projects/<your-fork-id>/pipelines/$PIPE_ID/jobs \
     | python3 -c "import sys,json; print([j['id'] for j in json.load(sys.stdin) if j['name']=='build'][0])")
   glab api projects/<your-fork-id>/jobs/$BUILD_JOB/trace | grep "Repository should be available"
   ```

   The URL looks like `https://gitlab.com/<you>/metadata/-/jobs/<id>/artifacts/raw/artifacts/repository.json`. Add it to KiCad: **Plugin Manager → Manage… → +**, paste the URL, install the package, and verify the plugin loads and works on Pcbnew.

A KiCad maintainer will then review. After it merges, expect up to one day for the package to appear in the public PCM catalogue (the public `kicad/addons/repository` is regenerated on a schedule).

### Updating an existing PCM submission (subsequent releases)

After the initial MR is merged, the metadata file `packages/com.github.drandyhaas.kicadroutingtools/metadata.json` already exists upstream. To add a new version:

1. Build and publish the new GitHub Release as above (steps 1–3 of *Cutting a release*).
2. In your `kicad-addons-fork` clone, pull upstream and create a new branch:

   ```bash
   git fetch target
   git checkout target/main
   git checkout -b update-kicadroutingtools-v0.15.6
   ```

3. **Append** the new version object to `versions[]` in `packages/com.github.drandyhaas.kicadroutingtools/metadata.json` (keep older entries — PCM history shows them).

4. Validate locally (`./ci/validate.sh`), commit, push, open MR — same as before.

---

## Common pitfalls

- **Duplicate `version` strings in `versions[]`** — PCM rejects two entries with the same version string regardless of their `platforms` arrays. Ship one zip per version that supports multiple platforms via runtime resolver, not one zip per platform.
- **Inner `metadata.json` with `download_*` fields** — the `metadata.json` *inside* the PCM zip must have exactly one version entry and no `download_*` fields. `package_pcm.py` strips these automatically; don't bypass it.
- **4-part `VERSION`** (e.g. `0.15.3.1`) — PCM regex rejects this. Use 3-part versions.
- **GitLab pipeline fails before any job runs** (status `failed`, no jobs, no `started_at`) — the GitLab account isn't identity-verified for shared compute yet. Try <https://gitlab.com/-/identity_verification>. After verifying, retrigger with a force-push (see next item).
- **MR widget keeps showing the old failed pipeline** — GitLab's MR `head_pipeline` only updates when a new `merge_request_event`-source pipeline runs. Manually creating a pipeline via `POST /projects/<id>/pipeline?ref=...` produces a `push`-source pipeline that does *not* update the MR widget. To get a fresh head_pipeline, force-push the same commit with a new timestamp:

  ```bash
  git commit --amend --no-edit --date=now
  git push --force-with-lease origin <branch>
  ```

- **`schema-v2.json` showing up in your commits** — the local validator overwrites this when running. Always `git checkout target/main -- schema-v2.json` (or stash it) before committing.
- **macOS x86_64 runners stuck queued** — `macos-13` runners are scarce. The workflow cross-compiles x86_64 from `macos-14` (arm64) instead.

---

## See also

- KiCad addon submission guide: <https://dev-docs.kicad.org/en/addons/>
- Upstream metadata repo: <https://gitlab.com/kicad/addons/metadata>
- PCM schema v1: <https://go.kicad.org/pcm/schemas/v1>
