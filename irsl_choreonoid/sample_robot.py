from . import robot_util as ru
from . import cnoid_util as cu
from . import cnoid_base as cb
import cnoid.IRSLCoords as ic
import cnoid.Util
import numpy as np

import math

class SampleRobot(ru.ImportedRobotModel):
    def __init__(self, robot=None, item=True, world=False, **kwargs):
        super().__init__(robot=robot, item=item, world=world, **kwargs)

    def _init_ending(self, **kwargs):
        #
        self.registerEndEffector('rleg', ## end-effector
                                 'RLEG_ANKLE_R', ## tip-link
                                 tip_link_to_eef = ic.coordinates(np.array([0, 0, -0.055])),
                                 joint_tuples = (('RLEG_HIP_R',   'hip-r'),
                                                 ('RLEG_HIP_P',   'hip-p'),
                                                 ('RLEG_HIP_Y',   'hip-y'),
                                                 ('RLEG_KNEE',    'knee-p'),
                                                 ('RLEG_ANKLE_P', 'ankle-p'),
                                                 ('RLEG_ANKLE_R', 'ankle-r')
                                                 )
                                 )
        self.registerEndEffector('lleg', ## end-effector
                                 'LLEG_ANKLE_R', ## tip-link
                                 tip_link_to_eef = ic.coordinates(np.array([0, 0, -0.055])),
                                 joint_tuples = (('LLEG_HIP_R',   'hip-r'),
                                                 ('LLEG_HIP_P',   'hip-p'),
                                                 ('LLEG_HIP_Y',   'hip-y'),
                                                 ('LLEG_KNEE',    'knee-p'),
                                                 ('LLEG_ANKLE_P', 'ankle-p'),
                                                 ('LLEG_ANKLE_R', 'ankle-r')
                                                 )
                                 )
        self.registerEndEffector('rarm', ## end-effector
                                 'RARM_WRIST_R', ## tip-link
                                 tip_link_to_eef = ic.coordinates(np.array([0.0, 0.0, -0.14]),
                                                                  ic.angleAxisNormalized(math.pi/2, np.array([0, 1, 0]))),
                                 joint_tuples = (('RARM_SHOULDER_P', 'shoulder-p'),
                                                 ('RARM_SHOULDER_R', 'shoulder-r'),
                                                 ('RARM_SHOULDER_Y', 'shoulder-y'),
                                                 ('RARM_ELBOW',      'elbow-p'),
                                                 ('RARM_WRIST_Y',    'wrist-y'),
                                                 ('RARM_WRIST_P',    'wrist-p'),
                                                 ('RARM_WRIST_R',    'wrist-r'),
                                                 )
                                 )
        self.registerEndEffector('larm', ## end-effector
                                 'LARM_WRIST_R', ## tip-link
                                 tip_link_to_eef = ic.coordinates(np.array([0.0, 0.0, -0.14]),
                                                                  ic.angleAxisNormalized(math.pi/2, np.array([0, 1, 0]))),
                                 joint_tuples = (('RARM_SHOULDER_P', 'shoulder-p'),
                                                 ('RARM_SHOULDER_R', 'shoulder-r'),
                                                 ('RARM_SHOULDER_Y', 'shoulder-y'),
                                                 ('RARM_ELBOW',      'elbow-p'),
                                                 ('RARM_WRIST_Y',    'wrist-y'),
                                                 ('RARM_WRIST_P',    'wrist-p'),
                                                 ('RARM_WRIST_R',    'wrist-r'),
                                                 )
                                 )
        #
        self.registerNamedPose('default',
                               [ 0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0, ## rleg
                                 0.523599, 0.0, 0.0, -1.74533, 0.15708, -0.113446, 0.637045, ## rarm
                                 0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0, ## lleg
                                 0.523599, 0.0, 0.0, -1.74533, -0.15708, -0.113446, -0.637045, ## larm
                                 0.0, 0.0, 0.0, ## waist
                                ])

### settings of model_file
SampleRobot.model_file = cnoid.Util.getShareDirectory() + '/model/SR1/SR1.body'

### robot_class:
robot_class = SampleRobot

#### makeRobot(robot=None, item=True, world=True, **kwargs):
#def makeRobot(robot=None, item=True, world=True, **kwargs):
#    return robot_class(robot, item=item, world=world, **kwargs)
