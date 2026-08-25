# Community alpha release

KiCad Track Gloss is packaged as a KiCad 10 SWIG ActionPlugin and follows the
official Plugin and Content Manager archive layout:

```text
metadata.json
plugins/
  __init__.py
  action_plugin.py
  ...
resources/
  icon.png
```

The archive metadata targets PCM schema v2. The package identifier is
`com.github.fca1.kicadtrackgloss`, the version status is `testing`, and the
numeric PCM version is used because the KiCad schema does not accept a
prerelease suffix. The corresponding GitHub release is marked as a prerelease.
Compatibility is explicitly bounded to KiCad 10.x because this alpha uses the
legacy SWIG ActionPlugin runtime.

Build the archive and the metadata directory ready to copy into a fork of the
official KiCad addons metadata repository:

```powershell
py -3.12 kicad_track_gloss\package_pcm.py --release-tag v0.3.28-alpha
```

Generated files:

```text
dist/KiCadTrackGloss-0.3.28.zip
dist/KiCadTrackGloss-0.3.28.meta.json
dist/kicad-official/packages/com.github.fca1.kicadtrackgloss/metadata.json
dist/kicad-official/packages/com.github.fca1.kicadtrackgloss/icon.png
```

The `metadata.json` inside the archive intentionally omits all `download_*`
fields. The separately generated official-repository metadata contains the
public download URL, SHA-256 digest, compressed size, and installed size.

Before submitting to KiCad, publish the GitHub prerelease and verify that its
archive URL is publicly downloadable. Then copy the generated package
directory into `packages/` of a fork of
<https://gitlab.com/kicad/addons/metadata> and submit a merge request. Do not
submit it to the generated public-facing addons repository.

The package is MIT-licensed and preserves the copyright and primary code
provenance of DrAndyHaas. Frantz is co-author and maintainer of the standalone
adaptation. Diagnostic dialog labels are currently in French, as disclosed in
the English PCM metadata.
