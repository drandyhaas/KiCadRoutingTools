"""Stage-by-stage wall clock for batch_route's setup.

Setup on a big board (parse, zone refill, obstacle maps, cost fields) can run
for minutes before the first net is tried, and most of it printed nothing.
``mark(stage)`` closes the stage that just ran and logs it when it took 1 s or
more; ``total()`` always logs, so every run says when routing actually
started. Reporting only: the clock decides what is PRINTED, never what is
routed.
"""
import time


class SetupClock:
    def __init__(self, run_t0: float):
        self.run_t0 = run_t0
        self.started = self.stage_start = time.monotonic()

    def mark(self, stage: str) -> None:
        now = time.monotonic()
        took = now - self.stage_start
        self.stage_start = now
        if took >= 1.0:
            print(f"  [setup] {stage}: {took:.1f}s "
                  f"(run clock {now - self.run_t0:.0f}s)", flush=True)

    def total(self) -> None:
        now = time.monotonic()
        self.stage_start = now
        print(f"Setup done in {now - self.started:.1f}s "
              f"(run clock {now - self.run_t0:.0f}s); routing starts",
              flush=True)
