from dataclasses import dataclass
from typing import Literal, Union

from .design_layer_taxonomy import DesignLayerType
from .design_layer import DesignLayer

@dataclass(frozen=True)
class OpticalDesignGroup:
    kind: Literal["optical_design_group"] = "optical_design_group"
    # add optical-specific metadata later if needed (e.g. mask writer, tone, etc.)


def _forbid_prefixes(layers: list[DesignLayer], prefixes: list[DesignLayerType], reason: str) -> None:
    violations: list[tuple[DesignLayer, DesignLayerType]] = []
    for L in layers:
        for p in prefixes:
            if L.layer_type.is_under(p):
                violations.append((L, p))

    if violations:
        lines = [f"Forbidden layer types for {reason}:"]
        for L, p in violations:
            lines.append(f"  - {L.name}: {L.layer_type} (under forbidden '{p}')")
        raise ValueError("\n".join(lines))
    
def _require_prefixes(layers: list[DesignLayer], prefixes: list[DesignLayerType], reason: str) -> None:
    missing = []
    for p in prefixes:
        if not any(L.layer_type.is_under(p) for L in layers):
            missing.append(p)

    if missing:
        lines = [f"Missing required layer types for {reason}:"]
        for p in missing:
            lines.append(f"  - need at least one layer under '{p}'")
        raise ValueError("\n".join(lines))



def _validate_optical_design_group(
    layers: list[DesignLayer],
    design_group: OpticalDesignGroup,
) -> None:

    forbidden_prefixes = [
        DesignLayerType.of("exposure", "ebeam"),
        DesignLayerType.of("exposure", "ebeam_large"),
        DesignLayerType.of("utility", "ebeam"),
    ]

    _forbid_prefixes(layers, forbidden_prefixes, reason="optical design")

    required_prefixes = [
        DesignLayerType.of("exposure", "optical"),
    ]

    _require_prefixes(layers, required_prefixes, reason="ebeam design")




@dataclass(frozen=True)
class EbeamDesignGroup:
    kind: Literal["ebeam_design_group"] = "ebeam_design_group"
    nominal_dose_uC_cm2: int = 200
    acceleration_voltage_kV: int = 100
    writefield_um: float = 100.0  # square write fields


def _validate_ebeam_design_group(
    layers: list[DesignLayer],
    design_group: EbeamDesignGroup,
) -> None:

    forbidden_prefixes = [
        DesignLayerType.of("exposure", "optical"),
        DesignLayerType.of("utility", "optical")
    ]

    _forbid_prefixes(layers, forbidden_prefixes, reason="ebeam design")

    required_prefixes = [
        DesignLayerType.of("utility", "ebeam"),
        DesignLayerType.of("exposure", "ebeam")
    ]

    _require_prefixes(layers, required_prefixes, reason="ebeam design")

    

DesignLayerGroupType = Union[OpticalDesignGroup, EbeamDesignGroup]
"""Group of DesignLAyers can be either of type optical or ebeam."""


def validate_design_layer_group(layers: list[DesignLayer], design_type: DesignLayerGroupType) -> None:
    """
    Validate a layer set against the design context/type.

    This enforces project/process rules like:
    - which layer categories are allowed for optical vs ebeam designs
    - required layer categories for a given design context
    """
    if design_type.kind == "ebeam_design_group":
        _validate_ebeam_design_group(layers, design_type)
    elif design_type.kind == "optical_design_group":
        _validate_optical_design_group(layers, design_type)
    else:
        raise ValueError(f"Unknown design_type.kind: {design_type.kind}")
