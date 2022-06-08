
# DISPLAY=:0 choreonoid
import irsl_choreonoid.sample_robot as sr
import numpy as np
import cnoid.IRSLUtil as iu
import math

rr = sr.init_sample_robot()
rr.set_pose('default')
rr.flush()
rr.foot_mid_coords()

rr.fix_leg_to_coords(np.identity(4))
rr.flush()

rr.move_centroid_on_foot()
rr.flush()
rr.foot_mid_coords()

rr.robot.mass
rr.robot.calcCenterOfMass()
rr.robot.centerOfMass

# print joints
#for j in rr.joint_list():
#    print('{}:{:f}'.format(j.name, j.q * 180 / math.pi))
