#!/usr/bin/env python3
"""A frame SINK for the routing movie: frames live on disk, not in RAM (#1036).

`make_movie` used to hold every frame of a film as a Pillow image in one list,
then run four post-passes over that list (compose into the planned frame, the
attempts band, the run clock, the iso panel) and hand it to the encoder. Run 32
(glasgow_revC, 22 boards, per-segment traces over ~9000 segments) produced
~6100 frames and the process working set reached **29.5 GB** before it was
stopped. Memory was O(frames).

`FrameSpool` is a drop-in for that list, and it keeps memory O(1) in the frame
count:

* ``append(img)`` writes the frame as a PNG (the pattern the owner's
  ``awx/evolve_movie.py`` ``emit`` uses, ``f%06d.png``) and keeps only its
  SIZE in memory;
* the post-passes do not rewrite the spool. Every one of them needs only the
  frame COUNT and the step marks -- both known before any pixel -- so each is
  registered as a per-frame transform with ``map(fn)`` and applied lazily, on
  the one frame being read, while the encoder streams;
* ``frames[i]`` and iteration load one frame at a time; ``frames[i] = img``
  (the placement camera's mirror pass does this) writes it back.

A plain ``list`` is still accepted everywhere a spool is: ``transform`` and
``frame_sizes`` below take either, and ``animate_route.save_movie`` still takes
the list ``awx/evolve_movie.py`` hands it. The in-process callers that build a
short film in memory (the GUI recorder, ``run_plan``, ``place_route_loop``,
``render_run``) keep working unchanged -- they reach the spool through
``make_movie`` and never see it.
"""
from __future__ import annotations

import os
import shutil
import tempfile

#: Pillow's zlib level for spooled frames. 1 is the fastest level that still
#: compresses; a spooled frame is read back exactly once per encode, so encode
#: speed matters more than its size on disk.
SPOOL_COMPRESS_LEVEL = 1


class FrameSpool:
    """A list-like frame store backed by a directory of PNGs.

    ``directory=None`` makes a private temp dir that ``close()`` removes.
    """

    def __init__(self, directory=None, keep=False):
        self._own = directory is None
        self.dir = directory or tempfile.mkdtemp(prefix='krt_frames_')
        os.makedirs(self.dir, exist_ok=True)
        self.keep = keep
        self._sizes = []          # the size of each STORED image
        self._applied = []        # how many maps each stored image already has
        self._maps = []           # [(fn, out_size or None)]
        #: Peak number of decoded frames held at once by this object -- it is
        #: always 0 or 1, and the streaming test asserts that.
        self.loaded = 0

    # -- the list protocol -------------------------------------------------
    def _path(self, i):
        return os.path.join(self.dir, 'f%06d.png' % i)

    def append(self, img):
        i = len(self._sizes)
        img.save(self._path(i), compress_level=SPOOL_COMPRESS_LEVEL)
        self._sizes.append(tuple(img.size))
        self._applied.append(len(self._maps))

    def __len__(self):
        return len(self._sizes)

    def __bool__(self):
        return bool(self._sizes)

    def _index(self, i):
        n = len(self._sizes)
        if i < 0:
            i += n
        if not 0 <= i < n:
            raise IndexError('FrameSpool index %d out of range (%d frames)'
                             % (i, n))
        return i

    def __getitem__(self, i):
        if isinstance(i, slice):
            return [self[k] for k in range(*i.indices(len(self)))]
        i = self._index(i)
        from PIL import Image
        with Image.open(self._path(i)) as im:
            img = im.convert('RGB')
        for fn, _sz in self._maps[self._applied[i]:]:
            img = fn(i, img)
        self.loaded = 1
        return img

    def __setitem__(self, i, img):
        i = self._index(i)
        img.save(self._path(i), compress_level=SPOOL_COMPRESS_LEVEL)
        self._sizes[i] = tuple(img.size)
        self._applied[i] = len(self._maps)

    def __iter__(self):
        for i in range(len(self)):
            yield self[i]

    # -- lazy transforms ---------------------------------------------------
    def map(self, fn, out_size=None):
        """Register ``fn(i, img) -> img`` for every frame, applied on read.

        ``out_size``, when the transform makes every frame one size, lets
        ``frame_sizes`` answer without decoding a single frame.
        """
        self._maps.append((fn, tuple(out_size) if out_size else None))

    def sizes(self):
        """The set of frame sizes AFTER every registered transform, decoding
        at most one frame per distinct stored size."""
        if not self._sizes:
            return set()
        # When the LAST transform declared its output size, every frame that
        # still has a transform pending ends at that size. A frame with none
        # pending is its stored size. Anything else is probed, once per
        # (stored size, transforms already applied) pair.
        nmaps = len(self._maps)
        tail = self._maps[-1][1] if self._maps else None
        out = set()
        probe = {}
        for i, sz in enumerate(self._sizes):
            if self._applied[i] == nmaps:
                out.add(sz)
                continue
            if tail:
                out.add(tail)
                continue
            key = (sz, self._applied[i])
            if key not in probe:
                probe[key] = self[i].size
            out.add(probe[key])
        return out

    def close(self):
        if self._own and not self.keep:
            shutil.rmtree(self.dir, ignore_errors=True)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


#: Spooled PNG bytes per frame PIXEL, measured by PR C's verifier: ~1.27 MB
#: for a 1400x788 frame at SPOOL_COMPRESS_LEVEL 1 (~1.15 B/px). A 6000-frame
#: film at that size spools ~7.6 GB -- disk traded for the RAM it saved.
SPOOL_BYTES_PER_PX = 1.15


def disk_check(directory, n_frames, frame_px):
    """`(fits, need_bytes, free_bytes)` for spooling `n_frames` frames of
    `frame_px` pixels each into `directory` (the temp dir when None).

    `fits` keeps a 20% margin. `free_bytes` is None when the free space
    cannot be read, and then `fits` is True -- an unanswerable question
    refuses nothing, but the caller can say it was not asked.
    """
    need = int(n_frames * frame_px * SPOOL_BYTES_PER_PX)
    try:
        free = shutil.disk_usage(directory or tempfile.gettempdir()).free
    except OSError:
        return True, need, None
    return free >= need * 1.2, need, free


def is_spool(frames):
    return isinstance(frames, FrameSpool)


def transform(frames, fn, out_size=None):
    """Apply ``fn(i, img) -> img`` to every frame of ``frames``.

    A spool registers it lazily (nothing decoded now); a list is rewritten IN
    PLACE, one frame at a time, exactly as every post-pass did before -- so an
    in-process caller holding a list sees what it always saw.
    """
    if is_spool(frames):
        frames.map(fn, out_size=out_size)
        return frames
    for i in range(len(frames)):
        frames[i] = fn(i, frames[i])
    return frames


def frame_sizes(frames):
    """The set of frame sizes, for a list or a spool."""
    if is_spool(frames):
        return frames.sizes()
    return {f.size for f in frames}
