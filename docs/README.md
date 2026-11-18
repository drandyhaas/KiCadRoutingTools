# Documentation index

Use this index to find instructions for the current checkout. Start with the
[project README](../README.md#quick-start) to install the tools.

## Choose a task

| Task | Start here |
|---|---|
| Install or use the KiCad plugin | [Plugin installation](../README.md#installation) |
| Route a board from the command line | [CLI workflow](../README.md#command-line-interface), then [Configuration](configuration.md) |
| Route with an AI plan or replay a saved plan | [Claude skills and plans](claude-skills.md) |
| Place parts by function | [Placement guide](../py_placer/placement/README.md) |
| Declare connector positions and mechanical facts | [Design brief](design-brief.md) |
| Grade a declared floorplan | [Floorplan intent](floorplan-intent.md) |
| Check a routed board or render it | [Utilities](utilities.md) |
| Make a 3D placement or routing movie | [3D movie guide](../movie3d/README.md) |
| Use the modules from Python | [Python API overview](python-api.md) |
| Contribute a fix or report a board failure | [Contributing](../CONTRIBUTING.md) |
| Run tests | [Test guide](../tests/README.md) and [GUI parity gates](../tests/gui_parity/README.md) |
| Build or publish a release | [Rust router](../rust_router/README.md) and [Release pipeline](release-pipeline.md) |

## Routing references

| Topic | Reference |
|---|---|
| CLI options and defaults | [Configuration](configuration.md) |
| User-layer guide paths | [Guide corridors](configuration.md#guide-corridor-options-preferred-route) |
| Engine structure and data flow | [Routing architecture](routing-architecture.md) |
| Differential pairs | [Differential pairs](differential-pairs.md) |
| Bus routing | [Bus routing](bus-routing.md) |
| Ordering and rip-up | [Net ordering](net-ordering.md), [Rip-up and reroute](rip-up-reroute.md) |
| Length and delay | [Length matching](length-matching.md) |
| Power traces and planes | [Power nets](power-nets.md), [Plane routing](route-plane.md) |
| Package escape routing | [BGA fanout](../py_router/bga_fanout/README.md), [QFN fanout](../py_router/qfn_fanout/README.md) |
| Standalone bus solver | [AWX](../awx/README.md) |

The [Python API overview](python-api.md) links to the parser, writer, geometry,
routing configuration, net analysis, and impedance references.

## Resolve conflicting instructions

Documentation describes the checked-out code. When the two disagree, use these
sources to establish the current contract and update the affected reference:

- A tool's parser and `--help` define its accepted CLI options. `krt_registry.py`
  lists the runnable tools and `krt_capabilities.py` inspects their options.
- `py_router/routing_defaults.py` owns shared routing defaults.
  `py_router/routing_config.py` defines `GridRouteConfig` fields. A CLI can resolve
  omitted values from the board, so a field initializer alone does not establish
  the effective value for a run.
- `requirements.txt` lists Python dependencies.
- `VERSION` defines the project release. `rust_router/Cargo.toml` defines the
  native module version, which can stay behind during Python-only patch releases.
  [Release pipeline](release-pipeline.md) explains the relationship.
- [CLAUDE.md](../CLAUDE.md) contains contributor and agent working conventions.
  Use the API pages for module signatures and the feature guides for user workflows.

## Historical material

[Design-rule proposal](design-rules-proposal.md) records the original audit,
decisions, and implementation exceptions. Its pre-implementation audit is
historical. Use [Configuration](configuration.md) and the
[routing configuration API](api-routing-config.md) for current behavior.

Fixture READMEs, stress-run reports, and preregistrations describe their named
boards or experiments. Their commands and measurements are scoped to that
context. A dated benchmark is not a performance guarantee for this checkout.

## Verify documentation changes

Run these checks from the repository root with the CLI dependencies and native
router installed:

```bash
python3 -X utf8 tests/test_doc_constants.py
python3 -X utf8 tests/test_doc_flag_liveness.py
python3 -X utf8 tests/test_431_skill_commands.py
python3 -X utf8 tests/run_doc_examples.py
```

These checks cover selected skill instructions, quoted constants, CLI arguments,
configuration fields, and complete API examples. The example runner skips
signature listings and incomplete fragments. A passing run does not verify all
prose, every Markdown link, or live KiCad GUI behavior. Compare changed claims
with the relevant source and run the feature's own checks when behavior changes.
