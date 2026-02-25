# QRCSJ_refactor

PHIDL-based refactor for building superconducting/quantum-circuit GDS components, with a validated design-layer system and an in-progress Dolan Josephson junction device module.

## Current Status

This repository is actively being refactored.

Implemented and usable today:

- design-layer taxonomy + validation
- optical / e-beam layer-group definitions
- combined `DEFAULT_DESIGN_LAYER_SET`
- reusable Dolan junction sub-geometry builders (arms, Dolan bridge variants, undercut variants, stress-relief variants)
- manual visualization notebook (`tests/dolan_junction.ipynb`)

Still in progress:

- full higher-level device/chip assembly flow described in `structure.md`
- polished `DolanJunction` final assembly API
- automated tests

## Project Structure

- `components/layerset/`
  - Layer model, taxonomy, validation policies, groups, and defaults
- `components/devices/`
  - Device generators (currently `josephson_junction.py`)
- `tests/`
  - Notebook-based manual checks and geometry visualization
- `structure.md`
  - Intended GDS hierarchy and notebook workflow roadmap

## Layer System Overview

The layer system separates process metadata from PHIDL geometry objects:

- `DesignLayer`
  - canonical layer definition (name, GDS layer/datatype, color, alpha, taxonomy type)
- `DesignLayerGroup`
  - validated collection of `DesignLayer` objects for one process context (optical or e-beam)
- `DesignLayerSet`
  - validated union of selected groups, plus cached PHIDL `LayerSet`

Default layer groups are selected in:

- `components/layerset/defaults.py`

and combined into:

- `DEFAULT_DESIGN_LAYER_SET`

## Dependencies

The code imports:

- `phidl`
- `gdspy`
- `numpy`

For notebook usage:

- `jupyter` / `ipykernel`

Install in your preferred environment (for example, a Conda env used for PHIDL work).

## Quick Start (Layers)

```python
from components.layerset.defaults import DEFAULT_DESIGN_LAYER_SET

# Project-level layer metadata registry
design_ls = DEFAULT_DESIGN_LAYER_SET

# PHIDL LayerSet for visualization/export workflows
phidl_layer_set = design_ls.layer_set

# PHIDL layer by name (via the PHIDL LayerSet)
ebeam_jj_layer = phidl_layer_set["ebeam_junctions"]
```

Note:

- `DesignLayerSet.layers` stores `DesignLayer` objects by name.
- `DesignLayerSet.layer_set` is a PHIDL `LayerSet`.

## Quick Start (Dolan Junction Building Blocks)

`components/devices/josephson_junction.py` currently exposes reusable builders for substructures.

```python
from components.devices.josephson_junction import DolanJunction, DolanJunctionParams

params = DolanJunctionParams()

arm = DolanJunction.build_arm_rect(params)
dolan = DolanJunction.build_dolan_rect(params)
undercut = DolanJunction().build_undercut(params)
stress = DolanJunction().build_stress_relief(params)
```

The module includes multiple undercut/stress variants and small dispatcher methods that choose a variant from params.

## Manual Testing / Visualization

Use the notebook:

- `tests/dolan_junction.ipynb`

It currently demonstrates:

- arm rectangle
- Dolan rectangle
- Dolan line (via `gdspy.FlexPath`)
- undercut rectangle / H undercut
- stress relief rectangle / line
- asymmetric stress polygon side helper

## Design Roadmap Context

`structure.md` describes the intended design hierarchy and notebook workflow, including:

- wafer/chip/test-structure organization
- frame/feedline/test-structure generation steps
- quantum circuit and resonator build sequence

Treat it as the target architecture plan, not a complete implementation status report.

## Development Notes

- The codebase is in a refactor state; some modules contain legacy and new parameters side by side.
- `tests/` currently contains a notebook, not unit tests.
- `__pycache__/` files are generated artifacts and should not be edited.

## Recommended Next Steps (for contributors)

1. Add automated tests for `components/layerset` validation and defaults import/build.
2. Finalize the `DolanJunction` assembly API (`build(...)`) on top of existing sub-builders.
3. Introduce typed layer bundles/accessors for better IDE autocomplete in device parameter dataclasses.
