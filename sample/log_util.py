## PYTHONPATH=$PYTHONPATH:$(dirname $(which choreonoid))/../lib/choreonoid-1.8/python python3
import irsl_choreonoid.hrpsyslog_util as hu

import irsl_choreonoid.robot_util as ru
from cnoid.IRSLCoords import coordinates

# names
base = 'gopos_1_0_0_PL0_20220716120647'
sufx = ['PL0(Robot)0_WAIST', 'RobotHardware_choreonoid0_q']
names = [ base + '.' + s for s in sufx ]

# merge files
hu.filterFiles(names, 'merged', filter_function = hu.simple_merge_without_time)

# use zip file
hu.filterFiles(names, 'merged2', zip_filename = 'test.zip', return_if_exist = True,
               filter_function = hu.simple_merge_without_time)

## parsing log with robot model
# r = ru.loadRobot('trans_p_model/PL0Pmain_for_choreonoid.wrl')
# 
# f_waist = create_writeLinkPositionFunc(r, 0)
# f_cog = create_writeCOGPositionFunc(r, 0)
# 
# filterFiles(names, 'pos_rpy_WAIST', base, filter_function = f_waist)
# filterFiles(names, 'pos_COG', base, filter_function = f_cog)

import irsl_choreonoid.hrpsyslog_util as hu
base = 'gopos_1_0_0_PL0_20220716120647'
sufx = ['PL0(Robot)0_WAIST', 'RobotHardware_choreonoid0_q']
names = [ base + '.' + s for s in sufx ]
hu.filterFiles(names, 'merged', filter_function = hu.simple_merge_without_time)
