"""CLI helper: let nets named like negative rails parse as VALUES (#597).

Boards legitimately carry nets named -3V3 / -12V / -5V. Whether
``--power-nets -3V3 +1V1`` parses depends on the PYTHON VERSION's argparse:
3.14's negative-number matcher (``-\\.?\\d``, prefix match) already treats a
dash-digit token as a value, while 3.12's (``^-\\d+$|^-\\d*\\.\\d+$``)
classifies ``-3V3`` as an unknown option -- so the same recorded manifest
replays cleanly on a dev machine and dies with ``unrecognized arguments:
-3V3`` inside a 3.12 container (the eit_mux failure, every arm of the #584
sweep). Same lesson as the math.fsum incident: interpreter-version behavior
divergence, this time in argparse.

``pin_dash_digit_values(parser)`` pins the lenient behavior on every
version: any token starting with ``-<digit>`` is a value, never an option.
Safe because no option string in any front starts with a digit, and it can
only widen what parses (a former hard error). The GUI is unaffected -- it
passes net lists directly to the engine functions, no argparse involved.

Call it on every parser whose options accept net names, right before
``parse_args``. The private attribute is stable across CPython 3.8-3.14;
the hasattr guard makes a future rename degrade to the old behavior
instead of crashing.
"""
from __future__ import annotations

import re

_DASH_DIGIT = re.compile(r"^-\d")


def pin_dash_digit_values(parser):
    if hasattr(parser, "_negative_number_matcher"):
        parser._negative_number_matcher = _DASH_DIGIT
    return parser


def _string_values(value):
    """Every str inside a parsed argument value (a str, or a list/tuple of
    them, nested to any depth)."""
    if isinstance(value, str):
        yield value
    elif isinstance(value, (list, tuple)):
        for item in value:
            yield from _string_values(item)


def refuse_joined_option_words(parser, args):
    """Refuse an argument that is an option and its values fused into ONE word.

    A shell (or a copied command) can hand over ``--layer-costs 1.0 1.5`` as a
    single argv entry. argparse cannot tell it from a net name, since a net
    may contain spaces, so ``--nets`` swallowed it, the router warned the net
    "does not exist" and then routed at the default layer costs and exited 0
    (#1410). A token that contains whitespace AND whose first word is one of
    this parser's own option strings (or an unambiguous abbreviation of a
    long one) is that mistake, never a net name; a dash-digit net such as
    ``-3V3`` has no whitespace and stays legal (#597).
    """
    options = list(getattr(parser, '_option_string_actions', {}))
    for dest, value in sorted(vars(args).items()):
        for tok in _string_values(value):
            words = tok.split()
            if len(words) < 2 or not words[0].startswith('-'):
                continue
            first = words[0]
            if first in options or (
                    first.startswith('--') and len(first) > 2
                    and any(o.startswith(first) for o in options)):
                parser.error(
                    f"argument value {tok!r} is the option {first} and its "
                    f"value(s) joined into one word; pass them as separate "
                    f"arguments (quote only the net names that need it)")


def refuse_unusable_inputs(parser, args):
    """The usage errors a routing CLI can name before it touches anything:
    an input board that is not a file, and a joined option word (#1410).
    Exit 2 with a one-line message, instead of the project banner followed
    by a traceback from deep inside the parser."""
    import os
    path = getattr(args, 'input_file', None)
    if path is not None and not os.path.isfile(path):
        parser.error(f"input board {path!r} does not exist (or is not a file)")
    refuse_joined_option_words(parser, args)
