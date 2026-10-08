"""awx_settings -- what awx's modules read by name: a policy (PLAN_PAGES, BRAID_PAIRS), a knob (BRAID_ATTEMPTS), a
stage's hand-off (HINT, GEO_FLIPS_FROM, SNAP_KEEP), a cache's switch (TAUT_MEMO).

They used to read os.environ, which the drivers set for each stage's process. A routing call inside KiCad's own
process cannot write os.environ: there it is every module's. So a module asks here -- `awx_settings.get('BRAID_PAIRS',
'0')`, its own default kept -- and a caller running the whole route in its own process GIVES the values for the call
(`with awx_settings.given(values):`), the whole of them, as an environment is the whole of a child process's. With
nothing given the environment answers, as it always has: the harness's scripts, and a stage run as a process of its
own, read exactly what they read before.

A value a module reads when it is imported is fixed by the first import in a process; whole_route's in-process runner
gives every stage the same policy, so what an early stage's import read is what a later stage would have.
"""
import contextlib
import os

_GIVEN = []                 # a stack of the values given: the innermost answers


def _source():
    return _GIVEN[-1] if _GIVEN else os.environ


def get(name, default=None):
    """the value NAME has, else DEFAULT (os.environ.get)"""
    return _source().get(name, default)


def req(name):
    """the value NAME has, a KeyError when it has none (os.environ[name])"""
    return _source()[name]


def environ():
    """a copy of every value: a child process's environment, or a key made of every variable"""
    return dict(_source())


@contextlib.contextmanager
def given(values):
    """VALUES answer for every awx module inside the block, the environment unread and unwritten"""
    _GIVEN.append(dict(values))
    try:
        yield
    finally:
        _GIVEN.pop()
