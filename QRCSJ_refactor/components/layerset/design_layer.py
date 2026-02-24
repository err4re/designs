from dataclasses import dataclass
from functools import cached_property

from phidl import LayerSet, Layer

from .design_layer_taxonomy import DesignLayerType

@dataclass(frozen=True)
class DesignLayer:
    """
    Canonical design-time definition of a GDS layer with metadata.

    This class is the source of truth for:
    - GDS layer/datatype (ebeam dose)
    - human-readable name/description
    - visualization metadata (color/alpha)
    - taxonomy classification (`type`)

    A PHIDL `Layer` object is exposed via the `layer` cached property for
    convenience when interacting with PHIDL APIs.
    """

    name: str = "layer"

    gds_layer: int = 0
    """GDSII layer number."""

    gds_datatype: int = 0
    """GDSII datatype number (used for ebeam dose)."""
    
    description: str = "layer description"
    color: str = "black"
    alpha: float = 0.6

    layer_type : DesignLayerType = DesignLayerType.of("exposure", "optical")
    """Layer type defines role of this layer in the design, e.g. `exposure/optical`."""

    @cached_property
    def layer(self) -> Layer:
        """
        This property creates a `phidl.device_layout.Layer` using the GDS layer,
        datatype, and visualization metadata stored in this `DesignLayer`.

        The returned object is cached on first access. Because `DesignLayer`
        instances are immutable (`frozen=True`), this cached value is safe
        and consistent for the lifetime of the object.

        Returns
        -------
        Layer
            A PHIDL `Layer` instance corresponding to this design layer.
        """
        return Layer(
            self.gds_layer,
            self.gds_datatype,
            self.name,
            self.description,
            color=self.color,
            alpha=self.alpha,
        )
    
    def __post_init__(self) -> None:
        # Intrinsic validation only (does not depend on taxonomy/policy)
        if not self.name or not self.name.strip():
            raise ValueError("DesignLayer.name must be non-empty.")
        if self.gds_layer < 0:
            raise ValueError("DesignLayer.gds_layer must be >= 0.")
        if self.gds_datatype < 0:
            raise ValueError("DesignLayer.gds_datatype must be >= 0.")
        if not (0.0 <= self.alpha <= 1.0):
            raise ValueError("DesignLayer.alpha must be in [0, 1].")
        
    def __str__(self) -> str:
        return f"{self.name} ({self.gds_layer}/{self.gds_datatype}) [{self.type}]"