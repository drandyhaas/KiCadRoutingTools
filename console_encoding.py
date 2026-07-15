"""UTF-8 console safety (issue #152, extended).

Windows consoles default to cp1252, which cannot encode the non-ASCII glyphs some
log lines use -- arrows in bus order, the Ohm sign in impedance, the warning sign
in the fab-floor escalation message, and more. A bare ``print()`` of such a glyph
raises ``UnicodeEncodeError`` and crashes the whole run.

Reconfiguring stdout/stderr to UTF-8 with ``errors="replace"`` makes a ``print``
never crash: the glyph is emitted as UTF-8 where the console supports it, and
replaced (never raised) where it does not.

Call :func:`enable_utf8_console` once at the top of every CLI entry point (each
``if __name__ == "__main__"`` block). It is idempotent and defensive -- a no-op on
older Pythons without ``reconfigure`` or on a stream that is not a real text
stream (e.g. an already-wrapped capture buffer under a test harness).
"""
import sys


def enable_utf8_console():
    """Reconfigure stdout/stderr to UTF-8 (errors='replace') so a non-ASCII
    ``print`` can never crash on a cp1252 console. Idempotent; swallows any
    failure so it is always safe to call unconditionally at startup."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass
