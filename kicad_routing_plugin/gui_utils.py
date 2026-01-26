"""
KiCad Routing Tools - GUI Utilities

Shared utilities for the plugin GUI.
"""


class StdoutRedirector:
    """Redirects stdout to a callback function while preserving original output."""

    def __init__(self, callback, original_stdout):
        self.callback = callback
        self.original = original_stdout

    def write(self, text):
        if text:
            # Write to original stdout
            if self.original:
                self.original.write(text)
            # Also send to callback
            self.callback(text)

    def flush(self):
        if self.original:
            self.original.flush()
