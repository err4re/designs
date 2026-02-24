from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .design_layer_taxonomy import DesignLayerType, DESIGN_LAYER_TYPE_TAXONOMY, validate_leaf_type
from .design_layer import DesignLayer
from .design_layer_group_policy import DesignLayerGroupType, validate_design_layer_group


@dataclass(frozen=True)
class DesignLayerGroup:
    """
    Class used to group together layers organized by the purpose they serve in the design.
    Validates layers and the group according to design_layer_group_policy.py

    - `name` of the group
    - `layers` dictionary of DesignLayers that belong to the group
    - `design_type` describes the purpose of the layer group, e.g. ebeam design for XL30 at 30keV

    Example
    -------
    DEFAULT_OPTICAl_DESIGN_LAYER_GROUP = DesignLayerGroup.build("default_optical_group", optical_layers, design_type, DESIGN_LAYER_TYPE_TAXONOMY)
    """
    
    name: str
    layers: dict[str, DesignLayer]
    design_type: DesignLayerGroupType

    # multiple construction paths possible in the future:
    # DesignLayerGroup.build(...)
    # DesignLayerGroup.from_yaml(...)
    # DesignLayerGroup.from_pdk(...)
    # DesignLayerGroup.from_defaults(...)

    @classmethod
    def build(cls, name, layer_defs: Iterable[DesignLayer], design_type: DesignLayerGroupType, taxonomy=DESIGN_LAYER_TYPE_TAXONOMY) -> DesignLayerGroup:
        """
        Builds a `DesignLayerGroup` from a list of DesignLayers and a DesignLayerGroupType after validating:
        1. layer types
        2. layer uniqueness
        3. layer group type

        Returns
        -------
        DesignLayerGroup
            A validated `DesignLayerGroup` instance.
        """
        layer_defs = list(layer_defs)

        # 1) validate layer types
        for L in layer_defs:
            validate_leaf_type(L.layer_type, taxonomy)

        # 2) validate layer uniqueness
        layers_by_name = {L.name: L for L in layer_defs}
        if len(layers_by_name) != len(layer_defs):
            raise ValueError("Duplicate layer names detected.")
        
        layers_by_number = {L.gds_layer: L for L in layer_defs}
        if len(layers_by_number) != len(layer_defs):
            raise ValueError("Duplicate layer numbers detected.")

        # 3) validate design type
        validate_design_layer_group(list(layers_by_name.values()), design_type)

        return cls(name=name, design_type=design_type, layers=layers_by_name)
    