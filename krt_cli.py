#!/usr/bin/env python3
"""Standalone KiCadRoutingTools command-line entry point."""

import multiprocessing

from krt_standalone.cli import main


if __name__ == '__main__':
    multiprocessing.freeze_support()
    raise SystemExit(main())
