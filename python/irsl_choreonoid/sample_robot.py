import irsl_choreonoid.robot_util as ru
import cnoid.IRSLUtil as iu
import cnoid.Util
import numpy as np

import math

def init_sample_robot(world = True):
    fname = cnoid.Util.getShareDirectory() + '/model/SR1/SR1.body'
    if ru.isInChoreonoid():
        ### in choreonoid
        ru.loadRobotItem(fname, 'SampleRobot', world)
        i = ru.findItem('SampleRobot')
        return SampleRobot(i)
    else:
        ### not in choreonoid
        rb = ru.loadRobot(fname)
        rb.calcForwardKinematics()
        return SampleRobot(rb)

class SampleRobot(ru.RobotModel):
    def __init__(self, robot):
        super(SampleRobot, self).__init__(robot)

        self.rleg_tip_link = self.robot.link('RLEG_ANKLE_R')
        self.lleg_tip_link = self.robot.link('LLEG_ANKLE_R')

        self.rleg_tip_to_eef = iu.cnoidPosition(np.array([0.0, 0.0, -0.055]))
        self.lleg_tip_to_eef = iu.cnoidPosition(np.array([0.0, 0.0, -0.055]))

        self.rarm_tip_link = self.robot.link('RARM_WRIST_R')
        self.larm_tip_link = self.robot.link('LARM_WRIST_R')

        self.rarm_tip_to_eef = iu.cnoidPosition(iu.angleAxisNormalized(math.pi/2, np.array([0, 1, 0])),
                                                np.array([0.0, 0.0, -0.14]))
        self.larm_tip_to_eef = iu.cnoidPosition(iu.angleAxisNormalized(math.pi/2, np.array([0, 1, 0])),
                                                np.array([0.0, 0.0, -0.14]))

    def default_pose__(self):
        return np.array([ 0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0, ## rleg
                          0.523599, 0.0, 0.0, -1.74533, 0.15708, -0.113446, 0.637045, ## rarm
                          0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0, ## lleg
                          0.523599, 0.0, 0.0, -1.74533, -0.15708, -0.113446, -0.637045, ## larm
                          0.0, 0.0, 0.0 ]);

    def lleg_mask(self):
        return (0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                1, 1, 1, 1, 1, 1,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0 )
    def rleg_mask(self):
        return (1, 1, 1, 1, 1, 1,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0 )
    def leg_mask(self):
        return (1, 1, 1, 1, 1, 1,
                0, 0, 0, 0, 0, 0, 0,
                1, 1, 1, 1, 1, 1,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0 )
    def larm_mask(self):
        return (0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                1, 1, 1, 1, 1, 1, 1,
                0, 0, 0 )
    def rarm_mask(self):
        return (0, 0, 0, 0, 0, 0,
                1, 1, 1, 1, 1, 1, 1,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0 )
    def torso_mask(self):
        return (0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                1, 1, 1 )
