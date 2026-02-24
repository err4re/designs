from __future__ import annotations
from dataclasses import dataclass
from functools import cached_property
from typing import Iterable

from phidl import LayerSet

from .design_layer_group import DesignLayerGroup
from .design_layer import DesignLayer

@dataclass(frozen=True)
class DesignLayerSet:

    layer_groups: dict[str, DesignLayerGroup]
    """all DesignLayerGroups in a dict by name"""

    layers: dict[str, DesignLayer]
    """all DesignLayers in a dict by name"""

    @cached_property
    def layer_set(self) -> LayerSet:
        """
        This property creates a `phidl.device_layout.LayerSet` using the GDS layer,
        datatype, and visualization metadata stored in the layers in `layer_groups`.

        The returned object is cached on first access. Because `DesignLayerGroup`
        instances are immutable (`frozen=True`), this cached value is safe
        and consistent for the lifetime of the object.

        Returns
        -------
        LayerSet
            A PHIDL `LayerSet` instance corresponding to this design layer set.
        """
        layer_set = LayerSet()

        for group in self.layer_groups.values():
            for design_layer in group.layers.values():
                layer_set.add_layer(design_layer.layer)
                print(f'added layer {design_layer.layer.name}')

        return layer_set
    


    # multiple construction paths possible in the future:
    # DesignLayerSet.build(...)
    # DesignLayerSet.from_yaml(...)
    # DesignLayerSet.from_pdk(...)
    # DesignLayerSet.from_defaults(...)

    @classmethod
    def build(cls, layer_groups: Iterable[DesignLayerGroup]) -> DesignLayerSet:
        """
        Builds a `DesignLayerSet` from a list of DesignLayerGroups after validating:
        
        1. layer uniqueness

        Returns
        -------
        DesignLayerSet
            A validated `DesignLayerSet` instance.
        """
        layer_groups = list(layer_groups)

        all_layers = [L
                      for group in layer_groups
                      for L in group.layers.values()]

        layers_by_name = {L.name: L
                          for group in layer_groups 
                          for L in group.layers.values()}
        
        if len(layers_by_name) != len(all_layers):
            # find layers with same names
            name_to_layers: dict[str, list[DesignLayer]] = {}
            for layer in all_layers:
                name_to_layers.setdefault(layer.name, []).append(layer)

            duplicate_names = {
                name: [f"{layer.gds_layer}/{layer.gds_datatype}" for layer in layers]
                for name, layers in name_to_layers.items()
                if len(layers) > 1
            }

            raise ValueError(f"Duplicate layer names detected: {duplicate_names}")
        
        layers_by_number = {L.gds_layer: L
                          for group in layer_groups 
                          for L in group.layers.values()}
        
        if len(layers_by_number) != len(all_layers):
            # find layers with same numbers
            number_to_layers: dict[int, list[DesignLayer]] = {}
            for layer in all_layers:
                number_to_layers.setdefault(layer.gds_layer, []).append(layer)

            duplicate_numbers = {
                gds_layer: [layer.name for layer in layers]
                for gds_layer, layers in number_to_layers.items()
                if len(layers) > 1
            }

            raise ValueError(f"Duplicate layer numbers detected: {duplicate_numbers}")


        return cls({group.name : group for group in layer_groups}, layers_by_name)
