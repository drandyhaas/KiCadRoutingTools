"""KiCad Track Gloss standalone ActionPlugin package."""

try:
    import pcbnew  # noqa: F401
except ImportError:
    # Unit tests run outside KiCad, where pcbnew is intentionally absent.
    pcbnew = None

if pcbnew is not None:
    # Do not hide errors from our own modules: KiCad must report a broken plugin
    # instead of silently omitting it from Tools -> External Plugins.
    from .action_plugin import KiCadTrackGlossPlugin

    KiCadTrackGlossPlugin().register()
