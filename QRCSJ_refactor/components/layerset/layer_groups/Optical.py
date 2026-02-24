### Group for optical layers ###
################################
from ..design_layer import DesignLayer, DesignLayerType
from ..design_layer_taxonomy import DESIGN_LAYER_TYPE_TAXONOMY
from ..design_layer_group import DesignLayerGroup
from ..design_layer_group_policy import OpticalDesignGroup
from ..design_layer_set import DesignLayerSet



optical_layers = []

## add layers to optical layer group
OPTICAL_LAYER_NUMBER = 0
optical_layer = DesignLayer(
                    name = 'optical', 
                    gds_layer = OPTICAL_LAYER_NUMBER,
                    gds_datatype = 0,
                    description = 'layer for laser lithography',
                    color = 'lightcoral',
                    alpha = 1,
                    layer_type = DesignLayerType.of('exposure', 'optical')
                    )

optical_layers.append(optical_layer)

ROUTING_LAYER_NUMBER = 1
routing_layer = DesignLayer(
                    name = 'routing',
                    gds_layer = ROUTING_LAYER_NUMBER,
                    gds_datatype = 0,
                    description = 'layer for routing, i.e. designing positive tone optical structures',
                    color = 'gold',
                    alpha = 0.5,
                    layer_type = DesignLayerType.of('utility', 'optical')
                    )

optical_layers.append(routing_layer)

GROUND_AVOIDANCE_LAYER_NUMBER = 2
ground_avoidance_layer = DesignLayer(
                    name = 'ground_avoidance',
                    gds_layer = GROUND_AVOIDANCE_LAYER_NUMBER,
                    gds_datatype = 0,
                    description = 'layer for ground avoidance',
                    color = 'yellow',
                    alpha = 0.5,
                    layer_type = DesignLayerType.of('utility', 'optical')
                    )

optical_layers.append(ground_avoidance_layer)

PCB_LAYER_NUMBER = 3
pcb_layer = DesignLayer(
                    name = 'pcb',
                    gds_layer = PCB_LAYER_NUMBER,
                    gds_datatype = 0,
                    description = 'layer for alignment with pcb pads',
                    color = 'black',
                    alpha = 1,
                    layer_type = DesignLayerType.of('utility', 'general')
                    )

optical_layers.append(pcb_layer)

## define design type for this optical layer group
design_type = OpticalDesignGroup()

## build optical layer group that can be used in the design
DEFAULT_OPTICAL_DESIGN_LAYER_GROUP = DesignLayerGroup.build("default_optical_group", 
                                                            optical_layers, 
                                                            design_type, 
                                                            DESIGN_LAYER_TYPE_TAXONOMY
                                                            )