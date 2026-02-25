# AGENTS.md

## Purpose

This repository is a PHIDL-based refactor for superconducting/quantum-circuit layout building, with current focus on:

- a validated design-layer model (`components/layerset`)
- process/layer-group defaults (optical + e-beam)
- a `DolanJunction` device builder under active refactor (`components/devices/josephson_junction.py`)

Use this file as project-specific guidance when making changes.

## Repo Map (Current)

- `components/layerset/design_layer.py`
  - `DesignLayer` dataclass (canonical layer metadata + cached PHIDL `Layer`)
- `components/layerset/design_layer_taxonomy.py`
  - taxonomy tree, `DesignLayerType`, type validation helpers
- `components/layerset/design_layer_group_policy.py`
  - optical/e-beam group policy validation rules
- `components/layerset/design_layer_group.py`
  - `DesignLayerGroup.build(...)` validation and grouping
- `components/layerset/design_layer_set.py`
  - `DesignLayerSet.build(...)`, duplicate checks, PHIDL `LayerSet` creation
- `components/layerset/layer_groups/*.py`
  - concrete layer definitions for optical and e-beam processes
- `components/layerset/defaults.py`
  - selects active groups and builds `DEFAULT_DESIGN_LAYER_SET`
- `components/devices/josephson_junction.py`
  - `DolanJunctionParams` + static geometry builders for Dolan JJ pieces
- `tests/dolan_junction.ipynb`
  - exploratory/manual visualization notebook for JJ sub-builders
- `structure.md`
  - target GDS hierarchy and notebook workflow vision

## Generated Files

- Ignore `__pycache__/` and `*.pyc` for implementation decisions.
- Do not edit generated cache files.

## Architecture Notes

### Layer System

- `DesignLayer` is the project-level source of truth (name, GDS layer/datatype, metadata, taxonomy type).
- PHIDL `Layer` objects are derived from `DesignLayer.layer`.
- `DesignLayerGroup` validates:
  - leaf taxonomy membership
  - unique names and GDS layer numbers (within group)
  - policy constraints for optical/e-beam groups
- `DesignLayerSet` validates uniqueness across selected groups and provides:
  - `layer_groups` by group name
  - `layers` by layer name
  - cached PHIDL `LayerSet`

### Device Building (JJ)

- `components/devices/josephson_junction.py` is currently a mixed state:
  - contains reusable static sub-builders (rectangles, lines, undercuts, stress relief)
  - contains dispatch methods (`build_undercut`, `build_stress_relief`)
  - not yet a complete final assembly pipeline
- `tests/dolan_junction.ipynb` exercises these sub-builders directly with `quickplot`.

## Coding Guidance For Agents

### Preserve Refactor Intent

- Prefer improving structure and validation over adding ad-hoc shortcuts.
- Keep layer metadata and PHIDL geometry concerns separated.
- Avoid collapsing `DesignLayer` into raw PHIDL `Layer` usage across the codebase.

### Be Careful With Existing Inconsistencies

The repo is mid-refactor. Before changing behavior, inspect the exact file state.

Known examples:

- `DolanJunctionParams` currently contains legacy and new fields side-by-side.
- Some dataclass fields are duplicated (later definitions override earlier ones).
- `DolanJunctionParams` layer defaults use PHIDL `LayerSet` indexing (`default_ls[...]`) rather than `DesignLayerSet.layers[...]`.
- Some layer group filenames use uppercase naming (`Optical.py`) which matters on case-sensitive systems.

### Validation Changes

When editing validation logic:

- keep errors explicit and actionable (list duplicates/missing names)
- preserve current construction APIs (`build(...)`) unless the task asks for breaking changes
- prefer validating once at build/factory boundaries

### Static vs Instance Methods in Device Builders

- Use `@staticmethod` for pure geometry sub-builders that depend only on params.
- Use instance methods for orchestration/assembly that stores refs on `self`.
- Selector/dispatcher methods (e.g. `build_undercut`) are a good pattern and should stay small.

## Testing / Verification Expectations

There is currently no formal automated test suite in `tests/` (only a notebook).

When making changes:

- at minimum, verify imports still work for touched modules
- if touching JJ geometry code, note whether `tests/dolan_junction.ipynb` scenarios are still consistent
- if touching layer definitions/validation, ensure `components/layerset/defaults.py` can still build `DEFAULT_DESIGN_LAYER_SET`

## Documentation Expectations

- Keep `README.md` aligned with current capabilities (sub-builders and layer infrastructure) and clearly mark incomplete areas.
- Use `structure.md` as the long-term workflow/architecture direction, not as a guarantee of implemented features.

## Change Scope Discipline

- If the user asks for a narrow change, edit only the requested function/module.
- Do not "clean up" unrelated issues unless explicitly asked.
- If you notice bugs outside scope, mention them briefly instead of silently changing them.
