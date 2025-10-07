### phidl imports
from phidl import Device, Layer, LayerSet, Path, CrossSection
from phidl.device_layout import DeviceReference
from phidl import quickplot as qp

import phidl.geometry as pg
import phidl.utilities as pu
import phidl.routing as pr
import phidl.path as pp

### general python imports
import numpy as np
import numpy.typing as npt

from scipy.spatial.distance import pdist, squareform
from scipy.optimize import linear_sum_assignment

from dataclasses import dataclass, field, replace
from typing import Tuple, Optional, Union

import itertools

from components.default_layerset import default_ls


def sync_attributes(source, target) -> None:
    """
    Update attributes in the target object with values from the source object 
    for all matching attribute names.
    The method described here does a shallow copy of attribute values. 
    If the attributes include mutable objects and you need a deep copy,
    you might need to use the copy.deepcopy() method where appropriate.

    Args:
    - source: object from which to copy attributes.
    - target: object to which attributes are copied.
    """
    for attr in vars(source):
        if hasattr(target, attr):
            setattr(target, attr, getattr(source, attr))


def subtract_layers(device: Device, layer_a: Layer, layer_b: Layer, offset: float = 0) -> None:
    Polys_A = pg.extract(device, layers=[layer_a])
    Polys_B = pg.extract(device, layers=[layer_b])

    device.remove_layers(layers=[layer_a])

    device << pg.boolean(A=Polys_A, B=pg.offset(Polys_B, distance = offset), operation='A-B', layer=layer_a)


def subtract_overlap_from_layer(A: Device, B: Union[Device, DeviceReference, list[Device], list[DeviceReference]], layer: Layer, offset: float = 0) -> None:
    Polys_A = pg.extract(A, layers=[layer])

    A.remove_layers(layers=[layer])

    A << pg.boolean(A=Polys_A, B=pg.offset(B, offset), operation='A-B', layer=layer)

def unify_layer(device: Device, layer: Layer) -> None:

    Polys = pg.extract(device, layers=[layer])

    Unified = pg.union(Polys, by_layer=True, layer=layer)

    device.remove_layers(layers=[layer])

    device << Unified

def pack_devices_x(devices: list[Device], spacing: float = 50) -> Device:
    main_device = Device('packed-devices')
    current_x = 0  # Start placing devices at x = 0

    ref = main_device.add_ref(devices[0])
    ref.move(ref.center, (0,0))
    ref.movex(-ref.xmin)
    current_x = ref.xmax + spacing  

    for i,device in enumerate(devices[1:]):
        ref = main_device.add_ref(device)
        ref.move(ref.center, (0,0))
        ref.movex(-ref.xmin)

        ref.movex(current_x)

        current_x = ref.xmax + spacing  

    return main_device

def pack_devices_y(devices: list[Device], spacing: float = 80) -> Device:
    main_device = Device('packed-devices')
    current_y = 0  # Start placing devices at x = 0

    ref = main_device.add_ref(devices[0])
    ref.move(ref.center, (0,0))
    ref.movex(-ref.xmin)
    ref.movey(-ref.ymin)
    current_y = ref.ymax + spacing  

    for i,device in enumerate(devices[1:]):
        ref = main_device.add_ref(device)
        ref.move(ref.center, (0,0))
        ref.movex(-ref.xmin)
        ref.movey(-ref.ymin)

        ref.movey(current_y)

        current_y = ref.ymax + spacing  

    return main_device

def find_closest_point(point, points):
    """
    Find the closest point in a list to the given point.

    Parameters:
    point (tuple): The given point (x, y).
    points (list of tuples): The list of points [(x1, y1), (x2, y2), ...].

    Returns:
    tuple: The closest point (x, y) in the list to the given point.
    """
    point_array = np.array(point)
    points_array = np.array(points)
    
    # Calculate the Euclidean distance from the given point to all points in the list
    distances = np.linalg.norm(points_array - point_array, axis=1)
    
    # Find the index of the closest point
    closest_index = np.argmin(distances)

    return tuple(points_array[closest_index])

def find_closest_point_in_x(given_point, points):
    """
    Find the point in a list that is closest in x-coordinate to the given point.

    Parameters:
    given_point (tuple): The given point (x, y).
    points (list of tuples): The list of points [(x1, y1), (x2, y2), ...].

    Returns:
    tuple: The point in the list that is closest in x-coordinate to the given point.
    """
    given_x = given_point[0]
    
    # Calculate the absolute difference in x-coordinates
    distances = [abs(point[0] - given_x) for point in points]
    
    # Find the index of the closest point in x
    closest_index = distances.index(min(distances))
    
    return points[closest_index]

def find_closest_point_in_x_with_y_tolerance(given_point, points, y_tolerance):
    """
    Find the point in the list that is closest in x-coordinate to the given point,
    among those within a y-coordinate tolerance interval.

    Parameters:
    given_point (tuple): The given point (x, y).
    points (list of tuples): The list of points [(x1, y1), (x2, y2), ...].
    y_tolerance (float): The tolerance interval for y-coordinates.

    Returns:
    tuple: The point in the list that is closest in x-coordinate within the y-coordinate tolerance interval.
    """
    given_x, given_y = given_point
    
    # Filter points within the y tolerance interval
    filtered_points = [point for point in points if abs(point[1] - given_y) <= y_tolerance]
    
    # If no points are within the y tolerance, return None or handle accordingly
    if not filtered_points:
        return None
    
    # Calculate the absolute differences in x-coordinates for the filtered points
    x_distances = [abs(point[0] - given_x) for point in filtered_points]
    
    # Find the index of the closest point in x among the filtered points
    closest_index = x_distances.index(min(x_distances))
    
    return filtered_points[closest_index]



### TODO add writefield??

@dataclass
class WritefieldParams:
    writefield_width: float = 100
    writefield_height: float = 100

    adapt_working_area_size: bool = False

    Alignment_mark: Optional[Device] = None

    mark_positions: list[tuple[float, float]] = field(default_factory=list)

    writefield_layer: Layer = default_ls['writefield_ebeam']
    working_area_layer: Layer = default_ls['working_area_ebeam']


def add_writefield(writefield_params: WritefieldParams, Ebeam_Device: Device = None) -> None:

        if writefield_params.writefield_width < Ebeam_Device.xsize:
            raise ValueError('Writefield width smaller than device width!')
        elif writefield_params.writefield_height < Ebeam_Device.ysize:
            raise ValueError('Writefield heigth smaller than device heigth!')          

        Writefield = Device('Writefield')
        Box = Writefield << pg.rectangle(size=(writefield_params.writefield_width, writefield_params.writefield_height),
                                         layer = writefield_params.writefield_layer)
        
        Box.move(origin=Box.center, destination=Ebeam_Device.center)

        #add alignment marks
        for position in writefield_params.mark_positions:
            Mark = Writefield << writefield_params.Alignment_mark
            Mark.move(Ebeam_Device.center + position)

        add_working_area(writefield_params, Ebeam_Device)

        Ebeam_Device << Writefield

def add_working_area(writefield_params: WritefieldParams, Ebeam_Device: Device = None) -> None:

        Working_Area = Device('Working area')

        if writefield_params.adapt_working_area_size:
            Box = Working_Area << pg.rectangle(size=(Ebeam_Device.xsize,Ebeam_Device.ysize),
                                               layer = writefield_params.working_area_layer)
        else:
            Box = Working_Area << pg.rectangle(size=(writefield_params.writefield_width, writefield_params.writefield_height),
                                               layer = writefield_params.working_area_layer)
        
        Box.move(origin=Box.center, destination=Ebeam_Device.center)

        Ebeam_Device << Working_Area


def extract_corners(device: Device, layers: list[Layer], decimals: int = 4) -> list[npt.NDArray]:
    Fields = pg.extract(D=device, layers=layers)
    Field_Polys = Fields.get_polygonsets()

    corners = [np.round([poly.xmin, poly.ymin, poly.xmax, poly.ymax], decimals) for poly in Field_Polys]

    return corners




def write_position_list(file_name: str, gds_path: str,
                        device: Device,
                        writefield_layers: list[Layer],
                        working_area_layers: list[Layer],
                        exposition_layers: list[Layer],
                        sort: bool = False) -> None:
    
    writefield_corners = extract_corners(device, writefield_layers)
    working_area_corners = extract_corners(device, working_area_layers)

    writefield_centers = np.array([[(r[0] + r[2]) / 2, (r[1] + r[3]) / 2] for r in writefield_corners])

    # check that all the working areas are inside their respective writefields
    for i, (writefield_rect, working_area_rect) in enumerate(zip(writefield_corners, working_area_corners)):
        if writefield_rect[0] <= working_area_rect[0] and writefield_rect[1] <= working_area_rect[1] and writefield_rect[2] >= working_area_rect[2] and writefield_rect[3] >= working_area_rect[3]:
            #print("working area inside write field")
            pass
        else: 
            raise ValueError(f"Working area {i} at {working_area_rect} is outside of write field {i} at {writefield_rect}.")

    
    # Generate sorted indices
    if sort:
        # sort by y coordinate first and then x coordinate
        sorted_indices = sorted(range(len(writefield_corners)), key=lambda i: (writefield_corners[i][1], writefield_corners[i][0]))

        # # Compute the distance matrix
        # distance_matrix = squareform(pdist(writefield_centers))

        # # Solve the TSP using the linear_sum_assignment, which actually solves an assignment problem,
        # # this is not a true TSP solver, but it can give a good approximation
        # row_ind, col_ind = linear_sum_assignment(distance_matrix)

        # sorted_indices = col_ind


        # Rearrange both lists
        writefield_corners = [writefield_corners[i] for i in sorted_indices]
        working_area_corners = [working_area_corners[i] for i in sorted_indices]
    
   
    # write positionlist header, open it write mode to clear the new positionlist
    with open('position_lists/positionlist_header.pls','r') as header, open(f'position_lists/{file_name}.pls','w') as positionlist:
        for line in header:
            positionlist.write(line)        

    with open(f'position_lists/{file_name}.pls', 'a') as f:
        f.write('\n')
        f.write('[DATA]')
        f.write('\n')
        
        for i, (writefield_rect, working_area_rect) in enumerate(zip(writefield_corners, working_area_corners)):

            writefield_width = np.round(writefield_rect[2] - writefield_rect[0])
            writefield_height = np.round(writefield_rect[3] - writefield_rect[1])

            working_area_width = np.round(working_area_rect[2] - working_area_rect[0])
            working_area_height = np.round(working_area_rect[3] - working_area_rect[1])

            writefield_center = [(writefield_rect[2] + writefield_rect[0])/2, 
                                 (writefield_rect[3] + writefield_rect[1])/2]
            
            f.write(str(i+1))
            f.write(',0.000000,0.000000,0.000000,0.000000,0.000000,')

            #writefield center
            if writefield_width > working_area_width or writefield_height > working_area_height:
                f.write("{:.6f}".format((writefield_center[0] + (writefield_width - working_area_width)/2)/1000))
                f.write(',')
                f.write("{:.6f}".format((writefield_center[1]+ (writefield_height - working_area_height)/2)/1000))
            else:
                f.write("{:.6f}".format(writefield_center[0]/1000))
                f.write(',')
                f.write("{:.6f}".format(writefield_center[1]/1000))
            
            f.write(',0.000000,XN,UV,toplevel,,EXPOSURE,')
            
            #Gives the length and width of the exposed area only for information purposes
            f.write(str(writefield_width) + '.000,' + str(writefield_height) + '.000,')
            f.write(',,,,')

            #half of writefield size
            f.write(str(writefield_width/2) + '00,' + str(writefield_height/2) + '00,')
            f.write(',,')
            
            f.write(gds_path) # f.write('D:\Alex\Ebeam litho\QRCSJ_R7\Run 2\QRCSJ_R7_v2.gds')

            #layers to be exposed
            exposition_layers_string = ''

            for exposition_layer in exposition_layers:
                exposition_layers_string += (str(exposition_layer.gds_layer) + ',')
            exposition_layers_string = exposition_layers_string[:-1]

            f.write(',"' + exposition_layers_string + '",') # f.write(',"1,2,3",') exposition layers....

            #lower left and upper right corner of working area
            working_area = '00;'.join(map(str, working_area_rect))
            working_area += '00,'
            f.write(working_area)

            #dose (should be set in layer not here, so leave it at 1)
            f.write('1.000,')

            #spldot setting/flag, spl = single point line
            #(unclear how flags are attributed, with 19 lines, curved structures and dots should be enabled)
            #f.write(',,,,,,,,,19,,,,,,')
            f.write(',,,,,,,,,,,,,,,')

            f.write('\n')


        #add end of lithography script!
        # f.write(str(len(corners)+1))
        # f.write(',0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,MN,dUV,,STAY,MACRO,,,,,,,,,,,%UserRoot%SCRIPT\ENDOFLITHO.JS,,,,,,,,,,,,,,,,,,,')