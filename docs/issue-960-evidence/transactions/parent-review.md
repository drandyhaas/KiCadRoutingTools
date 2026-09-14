# Independent provenance and transaction review — parent revision

Reviewer 2 used a detached isolated worktree at `cbc819b6b862f6ec14ecb30b8659819e800fce76`, independently read issue #960 and its single expanded acceptance comment, and checked PR #968 through GitHub (open; edgehero `fix/967-resolved-edge-clearance`, matching SHA). No production file was modified. `CONTRIBUTING.md` and `CLAUDE.md` were read; no `AGENTS.md` exists in this checkout.

The public `kicad_files/esp_prog.kicad_pcb` SHA256 is `165302e6a4f7aacdd64b3174df27ed8ddb19fd5f1f7effadd8d92205e25a120e`. The real stager produced baseline SHA256 `e9ec45eae4ea7f332825522d195fc05960a8e257b56a6101e5ea56e715528034`. Python was KiCad 10.0's 3.11.5. All board mutations occurred under `reviewer-runtime` or system temporary directories; the tracked fixture was preserved. Siblings were copied through `copy_siblings` for meaningful board copies; the interrupt test intentionally substitutes distinguishable JSON project bytes solely to witness rollback (no engineering claim).

Exact parent reproduction commands (run at isolated worktree root):

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 tests/stress/stage_unaided.py kicad_files/esp_prog.kicad_pcb reviewer-runtime/work reviewer-runtime/truth
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 py_placer/place_pose.py reviewer-runtime/work/board.kicad_pcb reviewer-runtime/work/pose.kicad_pcb set R1 136.4 98.8 --rot 270
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 tests/stress/provenance_audit.py --workdir reviewer-runtime/work --delivered reviewer-runtime/work/pose.kicad_pcb --json docs/issue-960-evidence/transactions/parent-audit.json
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-960-evidence/transactions/parent_adversarial.py
& 'C:/Program Files/KiCad/10.0/bin/python.exe' -X utf8 docs/issue-960-evidence/transactions/parent_concurrent.py
```

Observed independent failures:

- The sanctioned command exits 0 at effective clearance 0.25 mm and edge clearance 0.55 mm (both fixed defaults). Independently parsed written R1 pose is `[136.4,98.8,270]`. Pad conflicts improve 83 to 72, shortfall 27.1331 to 22.5783 mm; `forced=false`, `no_worse=true`, `legal=false`. Audit exits 4 with R1 unclaimed and zero ledger rows. This establishes availability and accounting failure; it establishes neither engineering-clean placement nor model quality.
- An undeclared writer stages with real `write_placed_output` in `%TEMP%`, then calls real `_promote` into the armed directory. No exception; output exists at requested R1 pose; zero rows.
- Injected ledger append failure in direct registered writer changes an existing output SHA to the moved board while emitting no row; pending state has already been popped.
- Two pending records for one output silently overwrite one another. A declaration in one thread is visible as active to a second undeclared thread.
- Injected `KeyboardInterrupt` immediately after the first successful `os.replace` leaves project bytes changed, board unchanged, no ledger row, and no recovery backups. This tests a Python interruption window; it does not simulate a power failure.
- A deterministic two-thread collision at parent `.krt-tmp` causes writer A to return success with writer B's coordinates. A requests x=136.4 and observes x=137.4. B refuses. The barriers isolate a possible interleaving; frequency under uninstrumented scheduling is not measured.
- `regime_for` stops after 24 ancestors. A board 25 child directories below an armed root has no detected regime and an undeclared direct write succeeds. The JSON captures this separate witness.

Independent acceptance derived from these observations and the issue: authorization must use the final destination before any deliverable replacement; final snapshots must follow snap/lock/side stamping; all failure handling must revoke pending claims; current output and requirement siblings must either be restored or disclosed with retained evidence. Thread/process exclusion must protect both candidate bytes and ledger publication. Interrupted ambiguous replaces must be tracked before invoking the replace syscall, because an exception can follow actual mutation. A persistent incomplete-transaction marker must make audit unproven until recovery is resolved. Model-chosen coordinates need an execution attribution without an invented optimizer decision claim.

Alternate-path audit identified seed initial post-write lock stamping and polish/reseat renames, reconstruct promotion and repair renames, route-loop final copies, and board-store materialization. These observations are review scope inputs, not assertions that an arbitrary raw filesystem copier can become a security boundary. Supported cooperating transactions and unsupported raw file mutation must be distinguished explicitly.

