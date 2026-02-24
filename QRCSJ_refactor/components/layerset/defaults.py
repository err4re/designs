from .design_layer import DesignLayer, DesignLayerType
from .design_layer_taxonomy import DESIGN_LAYER_TYPE_TAXONOMY
from .design_layer_group import DesignLayerGroup
from .design_layer_group_policy import OpticalDesignGroup, EbeamDesignGroup
from .design_layer_set import DesignLayerSet

## import optical layer groups
from .layer_groups.Optical import DEFAULT_OPTICAL_DESIGN_LAYER_GROUP

## import ebeam layer groups
from .layer_groups.XL_30_small_current_30kV import XL_30_SMALL_CURRENT_30kV_GROUP




## include/choose all layer groups that will be used/available in the design
LAYER_GROUPS = [DEFAULT_OPTICAL_DESIGN_LAYER_GROUP, XL_30_SMALL_CURRENT_30kV_GROUP]



## build layer set from chosen layer groups, a single layer is chosen and then used in the design process
DEFAULT_DESIGN_LAYER_SET = DesignLayerSet.build(LAYER_GROUPS)