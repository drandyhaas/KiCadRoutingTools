"""KiCad Track Gloss standalone ActionPlugin package."""

try:
    from .action_plugin import KiCadTrackGlossPlugin
    KiCadTrackGlossPlugin().register()
except ImportError:
    # Unit tests run outside KiCad, where pcbnew/wx are intentionally absent.
    pass

