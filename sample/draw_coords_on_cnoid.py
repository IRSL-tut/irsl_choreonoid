## exec on choreonoid
from irsl_choreonoid.hrpsyslog_util import HrpsysLogFiles
from irsl_choreonoid.draw_coords import DrawCoordsList
import numpy as np

hrpsys_log_dir = '/tmp'
hrpsys_log_file = 'hrpsysfilename'

col = np.array([0.3, 0.3, 0.3])
cdl0 = DrawCoordsList(x_color=col, y_color=col, z_color=col, length=0.025)
cdl1 = DrawCoordsList(length=0.05)
cdl2 = DrawCoordsList(length=0.05)

hh = HrpsysLogFiles(file_names=['{}.pos_COG'.format(hrpsys_log_file),
                                '{}.pose_rleg_eef'.format(hrpsys_log_file),
                                '{}.pose_lleg_eef'.format(hrpsys_log_file) ],
                    zip_filename='{}/{}.zip'.format(hrpsys_log_dir, hrpsys_log_file))

func0 = cdl0.generatePointFunction(flush=False,index=0)
func1 = cdl1.generateCoordsFunction(flush=False,index=1)
func2 = cdl2.generateCoordsFunction(flush=False,index=2)
hh.applyFunction([func0, func1, func2], skip=5)
cdl0.flush()
cdl1.flush()
cdl2.flush()
