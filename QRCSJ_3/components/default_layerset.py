from phidl import Device, Layer, LayerSet
import numpy as np

### default layer set for examples
default_ls = LayerSet()

OPTICAL = 0

ROUTING = 1

EBEAM = 11
EBEAM_DOSE = 1350

EBEAM_LOW = 12
EBEAM_LOW_DOSE = 1300

EBEAM_HIGH = 13
EBEAM_HIGH_DOSE = 1400

EBEAM_STRONG = 14
EBEAM_STRONG_DOSE = 1300

JJ_UNDERCUT = 15
JJ_UNDERCUT_DOSE = 360

UNDERCUT = 16
UNDERCUT_DOSE = 300

EBEAM_TEST = 20
UNDERCUT_TEST = 30

num_ebeam_test_doses = 5
start_dose = 1250
end_dose = 1450
ebeam_test_doses = np.linspace(start_dose, end_dose, num_ebeam_test_doses, dtype=int)

num_undercut_test_doses = 5
start_undercut_dose = 260
end_undercut_dose = 420
undercut_test_doses = np.linspace(start_undercut_dose, end_undercut_dose, num_undercut_test_doses, dtype=int)

EBEAM_LOG_TEST = 40
UNDERCUT_LOG_TEST = 50

num_ebeam_log_test_doses = 5
start_log_dose = 800
end_log_dose = 2000
ebeam_log_test_doses = np.geomspace(start_log_dose, end_log_dose, num_ebeam_log_test_doses, dtype=int)

num_undercut_log_test_doses = 5
start_log_undercut_dose = 250
end_log_undercut_dose = 550
undercut_log_test_doses = np.geomspace(start_log_undercut_dose, end_log_undercut_dose, num_undercut_log_test_doses, dtype=int)


WRITEFIELD_EBEAM = 70
WRITEFIELD_EBEAM_STRONG = 71

WORKING_AREA_EBEAM = 72
WORKING_AREA_EBEAM_STRONG = 73

PCB = 80

GROUND_AVOIDANCE = 81



default_ls.add_layer(name = 'optical', 
                    gds_layer = OPTICAL,
                    gds_datatype = 0,
                    description = 'layer for laser lithography',
                    color = 'lightcoral')

default_ls.add_layer(name = 'routing',
                    gds_layer = ROUTING,
                    gds_datatype = 0,
                    description = 'layer for ground avoidance',
                    color = 'gold',
                    alpha=0.5)

default_ls.add_layer(name = 'pcb',
                    gds_layer = PCB,
                    gds_datatype = 0,
                    description = 'layer for alignment with PCB pads',
                    color = 'black',
                    alpha=0.75)

default_ls.add_layer(name = 'ground_avoidance',
                    gds_layer = GROUND_AVOIDANCE,
                    gds_datatype = 0,
                    description = 'layer for ground avoidance',
                    color = 'lightyellow')

default_ls.add_layer(name = 'working_area_ebeam',
                    gds_layer = WORKING_AREA_EBEAM,
                    gds_datatype = 0,
                    description = 'layer for ebeam working areas',
                    color = 'lightgreen')

default_ls.add_layer(name = 'working_area_ebeam_strong',
                    gds_layer = WORKING_AREA_EBEAM_STRONG,
                    gds_datatype = 0,
                    description = 'layer for strong ebeam working areas',
                    color = 'green')

default_ls.add_layer(name = 'writefield_ebeam',
                    gds_layer = WRITEFIELD_EBEAM,
                    gds_datatype = 0,
                    description = 'layer for writefield',
                    color = 'brown')

default_ls.add_layer(name = 'writefield_ebeam_strong',
                    gds_layer = WRITEFIELD_EBEAM_STRONG,
                    gds_datatype = 0,
                    description = 'layer for writefield at high current',
                    color = 'darkred')

default_ls.add_layer(name = 'ebeam',
                    gds_layer = EBEAM,
                    gds_datatype = EBEAM_DOSE, #ebeam dose
                    description = 'layer for ebeam lithography',
                    color = 'blue')

default_ls.add_layer(name = f'ebeam_low',
                    gds_layer = EBEAM_LOW,
                    gds_datatype = EBEAM_LOW_DOSE, #low ebeam dose
                    description = 'layer for low dose ebeam lithography',
                    color = 'blue')

default_ls.add_layer(name = f'ebeam_high',
                    gds_layer = EBEAM_HIGH,
                    gds_datatype = EBEAM_HIGH_DOSE, #high ebeam dose
                    description = 'layer for high dose ebeam lithography',
                    color = 'blue')

default_ls.add_layer(name = 'ebeam_strong',
                    gds_layer = EBEAM_STRONG,
                    gds_datatype = EBEAM_STRONG_DOSE, #ebeam dose
                    description = 'layer for high current ebeam lithography',
                    color = 'darkblue')

default_ls.add_layer(name = 'jj_undercut',
                    gds_layer = JJ_UNDERCUT,
                    gds_datatype = JJ_UNDERCUT_DOSE, #ebeam dose
                    description = 'layer for undercut',
                    color = 'red')

default_ls.add_layer(name = 'undercut',
                    gds_layer = UNDERCUT,
                    gds_datatype = UNDERCUT_DOSE, #ebeam undercut dose
                    description = 'layer for ebeam undercut',
                    color = 'darkblue')

for i, dose in enumerate(ebeam_test_doses):
    default_ls.add_layer(name = f'ebeam_test_{i}',
                    gds_layer = EBEAM_TEST+i,
                    gds_datatype = dose, #ebeam dose
                    description = 'layer for ebeam lithography dose test',
                    color = 'blue')

for i, undercut_dose in enumerate(undercut_test_doses):
    default_ls.add_layer(name = f'undercut_test_{i}',
                    gds_layer = UNDERCUT_TEST+i,
                    gds_datatype = undercut_dose, #undercut dose
                    description = 'layer for ebeam lithography',
                    color = 'blue')
    
for i, dose in enumerate(ebeam_log_test_doses):
    default_ls.add_layer(name = f'ebeam_log_test_{i}',
                    gds_layer = EBEAM_LOG_TEST+i,
                    gds_datatype = dose, #ebeam dose
                    description = 'layer for ebeam lithography dose test',
                    color = 'blue')

for i, undercut_dose in enumerate(undercut_log_test_doses):
    default_ls.add_layer(name = f'undercut_log_test_{i}',
                    gds_layer = UNDERCUT_LOG_TEST+i,
                    gds_datatype = undercut_dose, #undercut dose
                    description = 'layer for ebeam lithography',
                    color = 'blue')