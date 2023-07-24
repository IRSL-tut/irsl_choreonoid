import cnoid.Body
#import cnoid.Util

from .cnoid_util import *

import cnoid.IRSLCoords as ic
import cnoid.IKSolvers as IK

import numpy as np
import random

def make_coordinates(coords_map):
    """Generating coordinates(cnoid.IRSLCoords.coordinates) from dictionay

    Args:
        coords_map (dictionary) : dictionary of describing transformation

    Returns:
        cnoid.IRSLCoords.coordinates : generated coordinates

    Examples:

        make_coordinates( {'position' : [1, 2, 3]} )

        make_coordinates( {'pos' : [1, 2, 3], 'rot' : [math.pi/3, 0, 0]} )

        make_coordinates( {'angle-axis' : [0, 1, 0, math.pi/4] })

        make_coordinates( {'quaternion' : [0, 0, 0, 1] })
    """
    pos = None
    for key in ('position', 'translation', 'pos', 'trans'):
        if key in coords_map:
            pos = np.array(coords_map[key])
            break
    for key in ('q', 'quaternion'):
        if key in coords_map:
            q = np.array(coords_map[key])
            if pos is None:
                return ic.coordinates(q)
            else:
                return ic.coordinates(pos, q)
    for key in ('angle-axis', 'aa'):
        if key in coords_map:
            aa = coords_map[key]
            rot = ic.angleAxisNormalized(aa[3], np.array(aa[:3]))
            if pos is None:
                return ic.coordinates(rot)
            else:
                return ic.coordinates(pos, rot)
    for key in ('rotation', 'matrix', 'mat', 'rot'):
        if key in coords_map:
            rot = np.array(coords_map[key])
            if pos is None:
                return ic.coordinates(rot)
            else:
                return ic.coordinates(pos, rot)
    for key in ('rpy', 'RPY', 'roll-pitch-yaw'):
        if key in coords_map:
            if pos is None:
                ret = ic.coordinates()
            else:
                ret = ic.coordinates(pos)
            ret.setRPY(np.array(coords_map[key]))
            return ret
    if pos is not None:
        return ic.coordinates(pos)
    raise Exception('{}'.format(coords_map))

def make_coords_map(coords):
    """Generating dictonary describing transformation

    Args:
        coords (cnoid.IRSLCoords.coordinates) : input

    Returns:
        dictionary : output, can be used by make_coordinates

    Examples:

        make_coords_map( make_coordinates( {'position' : [1, 2, 3]} ) )
    """
    return {'pos': cds.pos.tolist(), 'aa': cds.rotationAngle().tolist()}

##
## IKWrapper
##
class IKWrapper(object):
    """IKWrapper(class)

    """
    def __init__(self, robot, tip_link, tip_to_eef = None, use_joints = None):
        """IKWrapper(initializer)

        Args:

        Returns:

        """
        self.__robot = robot
        #
        self.__tip_link = self.__parseLink(tip_link)
        if self.__tip_link is None:
            raise Exception('invalid tip_link')
        #
        if tip_to_eef is None:
            self.__tip_to_eef = ic.coordinates()
        elif type(tip_to_eef) is map:
            self.__tip_to_eef = make_coordinates(tip_to_eef)
        else:
            self.__tip_to_eef = tip_to_eef
        if type(self.__tip_to_eef) is not ic.coordinates:
            raise Exception('invalid tip_to_eef')
        self.__tip_to_eef_cnoid = self.__tip_to_eef.toPosition()
        #
        if use_joints is None:
            self.__default_joints = self.__robot.jointList()
        else:
            self.__default_joints = [ self.__parseJoint(j) for j in use_joints ]
        self.resetJointWeights()
        self.__default_pose = self.angleVector()

    def flush(self):
        """flush

        Args:

        Returns:

        """
        self.__robot.calcForwardKinematics()
        if isInChoreonoid(): ## only have effects while running in choreonoid
            ret = findBodyItem(self.__robot)
            if ret is not None:
                ret.notifyKinematicStateUpdate()

    def updateDefault(self):
        """updateDefault(self):

        Args:

        Returns:

        """
        self.__default_joints = self.__current_joints
        self.__default_pose = self.angleVector()
        self.resetJointWeights()

    def __parseLink(self, id_name_link):
        if type(id_name_link) is int:
            return self.__robot.link(id_name_link)
        elif type(id_name_link) is str:
            return self.__robot.link(id_name_link)
        elif type(id_name_link) is cnoid.Body.Link:
            return id_name_link
        return None

    def __parseJoint(self, id_name_joint):
        if type(id_name_joint) is int:
            return self.__robot.joint(id_name_joint)
        elif type(id_name_joint) is str:
            return self.__robot.joint(id_name_joint)
        elif type(id_name_joint) is cnoid.Body.Link:
            return id_name_joint
        return None

    def endEffector(self, **kwargs):
        """endEffector(self, **kwargs):

        Args:

        Returns:

        """
        return ic.coordinates(self.__tip_link.getPosition().dot(self.__tip_to_eef_cnoid))

    def resetJointWeights(self):
        """resetJointWeights(self):

        Args:

        Returns:

        """
        self.__current_joints = [j for j in self.__default_joints]
        self.updateJointWeights()

    def updateJointWeights(self):
        """updateJointWeights(self):

        Args:

        Returns:

        """
        self.__joint_weights = np.zeros(self.__robot.numJoints)
        for idx in range(self.__robot.numJoints):
            if self.__robot.joint(idx) in self.__current_joints:
                self.__joint_weights[idx] = 1

    def setJoints(self, jlist, enable = True):
        """setJoints(self, jlist, enable = True):

        Args:

        Returns:

        """
        for j in jlist:
            self.__setJoint(j, enable = enable)
        self.updateJointWeights()

    def __setJoint(self, joint_or_id, enable = True):
        if type(joint_or_id) == cnoid.Body.Link:
            pass
        elif type(joint_or_id) == int:
            if joint_or_id >= self.__robot.getNumJoints():
                raise Exception('number of joints')
            joint_or_id = self.__robot.joint(joint_or_id)
        else:
            raise Exception('wrong type')

        if enable:
            if not joint_or_id in self.__current_joints:
                self.__current_joints.append(joint_or_id)
        else:
            while joint_or_id in self.__current_joints:
                self.__current_joints.remove(joint_or_id)

    def inverseKinematics(self, coords, weight = [1,1,1, 1,1,1], add_noise = None, debug = False, max_iteration = 32, threshold = 5e-5, **kwargs):
        """inverseKinematics(self, coords, weight = [1,1,1, 1,1,1], add_noise = None, debug = False, max_iteration = 32, threshold = 5e-5, **kwargs):

        Args:

        Returns:

        """
        ##
        if add_noise is not None:
            if type(add_noise) is float:
                self.addNoise(max_range = add_noise, joint_list = self.__current_joints)
            else:
                self.addNoise(max_range = 0.2, joint_list = self.__current_joints)
        ##
        if weight == 'position':
            weight = [1, 1, 1, 0, 0, 0]
        elif weight == 'rotation':
            weight = [0, 0, 0, 1, 1, 1]
        elif type(weight) is str:
            ## 'xyzRPY'
            wstr = weight
            weight = [0, 0, 0, 0, 0, 0]
            for ss in wstr:
                if ss == 'x':
                    weight[0] = 1
                elif ss == 'y':
                    weight[1] = 1
                elif ss == 'z':
                    weight[2] = 1
                elif ss == 'R':
                    weight[3] = 1
                elif ss == 'P':
                    weight[4] = 1
                elif ss == 'Y':
                    weight[5] = 1

        constraints = IK.Constraints()
        ra_constraint = IK.PositionConstraint()
        ra_constraint.A_link =     self.__tip_link
        ra_constraint.A_localpos = self.__tip_to_eef_cnoid
        #constraint->B_link() = nullptr;
        ra_constraint.B_localpos = coords.toPosition()
        ra_constraint.weight = np.array(weight)
        constraints.push_back(ra_constraint)

        jlim_avoid_weight_old = np.zeros(6 + self.__robot.getNumJoints())
        ##dq_weight_all = np.ones(6 + self.robot.getNumJoints())
        dq_weight_all = np.append(np.zeros(6), self.__joint_weights)

        d_level = 0
        if debug:
            d_level = 1

        loop = IK.solveFullbodyIKLoopFast(self.__robot,
                                          constraints,
                                          jlim_avoid_weight_old,
                                          dq_weight_all,
                                          max_iteration,
                                          threshold,
                                          d_level)
        if debug:
            for cntr, const in enumerate(constraints):
                const.debuglevel = 1
                if const.checkConvergence():
                    print('constraint %d (%s) : converged'%(cntr, const))
                else:
                    print('constraint %d (%s) : NOT converged'%(cntr, const))
        return (const.checkConvergence(), loop)

    #def angleVectorOrg(self, av = None):
    #    if av is not None:
    #        for idx in range(len(self.__default_joints)):
    #            self.__default_joints[idx].q = av[idx]
    #    return np.array([ j.q for j in self.__default_joints ])
    ##
    def angleVector(self, av = None):
        """angleVector(self, av = None):

        Args:

        Returns:

        """
        return self.__angleVector(av, self.__default_joints)
    def currentAngleVector(self, av = None):
        """currentAngleVector(self, av = None):

        Args:

        Returns:

        """
        return self.__angleVector(av, self.__current_joints)
    def __angleVector(self, av, joint_list):
        if av is not None:
            for j, ang in zip(joint_list, av):
                j.q = ang
            self.__robot.calcForwardKinematics()
        return np.array([ j.q for j in joint_list])

    def resetPose(self):
        """resetPose(self):

        Args:

        Returns:

        """
        self.angleVector(self.__default_pose)

    def addNoise(self, max_range = 0.1, joint_list = None):
        """addNoise(self, max_range = 0.1, joint_list = None):

        Args:

        Returns:

        """
        if joint_list is None:
            joint_list = self.__default_joints
        for j in joint_list:
            j.q += random.uniform(-max_range, max_range)
        self.__robot.calcForwardKinematics()

    ## read-only
    @property
    def robot(self):
        """robot(self):

        Args:

        Returns:

        """
        return self.__robot
#    @body.setter
#    def body(self, in_body):
#        self.__body = in_body
    @property
    def tip_link(self):
        """tip_link(self):

        Args:

        Returns:

        """
        return self.__tip_link
    @property
    def tip_to_eef(self):
        """tip_to_eef(self):

        Args:

        Returns:

        """
        return self.__tip_to_eef
    @property
    def joint_weights(self):
        """joint_weights(self):

        Args:

        Returns:

        """
        return self.__joint_weights
    @property
    def current_joints(self):
        """current_joints(self):

        Args:

        Returns:

        """
        return self.__current_joints
    @property
    def default_joints(self):
        """default_joints(self):

        Args:

        Returns:

        """
        return self.__default_joints
    @property
    def default_pose(self):
        """default_pose(self):

        Args:

        Returns:

        """
        return self.__default_pose

## add methods to choreonoid's class
def __joint_list(self):
    return [self.joint(idx) for idx in range(self.numJoints) ]
def __link_list(self):
    return [self.link(idx) for idx in range(self.numLinks) ]

cnoid.Body.Body.jointList = __joint_list
cnoid.Body.Body.linkList = __link_list
cnoid.Body.Body.angleVector = lambda self, vec = None: ic.angleVector(self) if vec is None else ic.angleVector(self, vec)
cnoid.Body.Link.getCoords = lambda self: ic.getCoords(self)
cnoid.Body.Link.setCoords = lambda self, cds: ic.setCoords(self, cds)
cnoid.Body.Link.getOffsetCoords = lambda self: ic.getOffsetCoords(self)
cnoid.Body.Link.setOffsetCoords = lambda self, cds: ic.setOffsetCoords(self, cds)

class RobotModel(object):
    def __init__(self, robot):
        self.item = None
        if isinstance(robot, BodyItem):
            self.robot = robot.body
            self.item = robot
        elif isinstance(robot, cnoid.Body.Body):
            self.robot = robot
        else:
            raise TypeError('')

        self.rleg_tip_link = None
        self.rleg_tip_to_eef = None
        self.lleg_tip_link = None
        self.lleg_tip_to_eef = None

        self.rarm_tip_link = None
        self.rarm_tip_to_eef = None
        self.larm_tip_link = None
        self.larm_tip_to_eef = None

        self.head_tip_link = None
        self.head_tip_to_eef = None
        self.torso_tip_link = None
        self.torso_tip_to_eef = None

    def init_ending(self):
        for limb in ('rleg', 'lleg', 'rarm', 'larm', 'head', 'torso'):
            eef = eval('self.%s_tip_to_eef'%(limb))
            if not eef is None:
                exec('self.%s_tip_to_eef_coords = ic.coordinates(self.%s_tip_to_eef)'%(limb, limb))
                exec('type(self).%s_end_coords = lambda self : self.%s_tip_link.getCoords().transform(self.%s_tip_to_eef_coords)'%(limb, limb, limb))
    #def robot(self):
    #    return robot

    #def links(self):
    #    num = self.robot.numLinks
    #    return [ self.robot.link(n) for n in range(num) ]
    def linkList(self):
        return self.robot.linkList()
    #def joint_list(self):
    #    num = self.robot.numJoints
    #    return [ self.robot.joint(n) for n in range(num) ]
    def jointList(self):
        return self.robot.jointList()
    #def angle_vector(self, angles = None):
    #    num = self.robot.numJoints
    #    if not (angles is None):
    #        if num != len(angles):
    #            raise TypeError('length of angles does not equal with robot.numJoints')
    #        for idx in range(num):
    #            self.robot.joint(idx).q = angles[idx]
    #        return None
    #    else:
    #        return np.array([ self.robot.joint(n).q for n in range(num) ])
    def angleVector(self, angles = None):
        return self.robot.angleVector(angles)

    def flush(self):
        if self.robot is not None:
            self.robot.calcForwardKinematics()
        if self.item is not None:
            self.item.notifyKinematicStateChange()
            MessageView.instance.flush()

    def default_pose__(self):
        pass

    def init_pose__(self):
        pass

    def set_pose(self, name):
        av = eval('self.%s_pose__()'%(name))
        self.angleVector(av)
        return self.angleVector()

    def end_effector_cnoid(self, limb):
        return eval('self.%s_end_effector_cnoid()'%(limb))

    def end_effector(self, limb):
        tip_link = eval('self.%s_tip_link'%(limb))
        tip_to_eef = eval('self.%s_tip_to_eef_coords'%(limb))
        cds = tip_link.getCoords()
        cds.transform(tip_to_eef)
        return cds

    ### Position
    def rleg_end_effector_cnoid(self):
        return self.rleg_tip_link.getPosition().dot(self.rleg_tip_to_eef)
    def lleg_end_effector_cnoid(self):
        return self.lleg_tip_link.getPosition().dot(self.lleg_tip_to_eef)
    def rarm_end_effector_cnoid(self):
        return self.rarm_tip_link.getPosition().dot(self.rarm_tip_to_eef)
    def larm_end_effector_cnoid(self):
        return self.larm_tip_link.getPosition().dot(self.larm_tip_to_eef)
    def head_end_effector_cnoid(self):
        return self.head_tip_link.getPosition().dot(self.head_tip_to_eef)
    def torso_end_effector_cnoid(self):
        return self.torso_tip_link.getPosition().dot(self.torso_tip_to_eef)

    def foot_mid_coords_cnoid(self, p = 0.5):
        return ic.Position_mid_coords(p, self.rleg_end_effector_cnoid(), self.lleg_end_effector_cnoid())

    def fix_leg_to_coords_cnoid(self, position, p = 0.5):
        mc = self.foot_mid_coords_cnoid(p)
        rL = self.robot.rootLink
        rL.setPosition(
            (ic.PositionInverse(mc).dot(position)).dot(rL.getPosition())
            )
        self.robot.calcForwardKinematics()

    ### coordinates
    def rleg_end_effector(self):
        return self.rleg_tip_link.getCoords().transform(self.rleg_tip_to_eef_coords)
    def lleg_end_effector(self):
        return self.lleg_tip_link.getCoords().transform(self.lleg_tip_to_eef_coords)
    def rarm_end_effector(self):
        return self.rarm_tip_link.getCoords().transform(self.rarm_tip_to_eef_coords)
    def larm_end_effector(self):
        return self.larm_tip_link.getCoords().transform(self.larm_tip_to_eef_coords)
    def head_end_effector(self):
        return self.head_tip_link.getCoords().transform(self.head_tip_to_eef_coords)
    def torso_end_effector(self):
        return self.torso_tip_link.getCoords().transform(self.torso_tip_to_eef_coords)

    def foot_mid_coords(self, p = 0.5):
        cds = self.end_effector('rleg')
        return cds.mid_coords(0.5, self.end_effector('lleg'))

    def fix_leg_to_coords(self, coords, p = 0.5):
        mc = self.foot_mid_coords(p)
        rL = self.robot.rootLink
        cds = mc.inverse_transformation()
        cds.transform(coords)
        cds.transform(rL.getCoords())
        rL.setCoords(cds)
        self.robot.calcForwardKinematics()

    def inverse_kinematics_cnoid(self, position, limb = 'rarm', weight = [1,1,1, 1,1,1], debug = False):
        constraints = IK.Constraints()
        ra_constraint = IK.PositionConstraint()
        ra_constraint.A_link =     eval('self.%s_tip_link'%(limb))
        ra_constraint.A_localpos = eval('self.%s_tip_to_eef'%(limb))
        #constraint->B_link() = nullptr;
        ra_constraint.B_localpos = position
        ra_constraint.weight = np.array(weight)
        constraints.push_back(ra_constraint)

        jlim_avoid_weight_old = np.zeros(6 + self.robot.getNumJoints())
        ##dq_weight_all = np.ones(6 + self.robot.getNumJoints())
        dq_weight_all = np.append(np.zeros(6), np.ones(self.robot.getNumJoints()))

        d_level = 0
        if debug:
            d_level = 1

        loop = IK.solveFullbodyIKLoopFast(self.robot,
                                          constraints,
                                          jlim_avoid_weight_old,
                                          dq_weight_all,
                                          20,
                                          1e-6,
                                          d_level)
        if debug:
            for cntr, const in enumerate(constraints):
                const.debuglevel = 1
                if const.checkConvergence():
                    print('constraint %d (%s) : converged'%(cntr, const))
                else:
                    print('constraint %d (%s) : NOT converged'%(cntr, const))

        return loop

    def inverse_kinematics(self, coords, limb = 'rarm', weight = [1,1,1, 1,1,1], debug = False):
        pos = coords.toPosition()
        return inverse_kinematics_cnoid(pos, limb=limb, weight=weight, debug=debug)

    def move_centroid_on_foot_cnoid(self, p = 0.5, debug = False):
        mid_pos = self.foot_mid_coords(p)
        #mid_trans = ic.Position_translation(mid_pos)
        mid_trans = mid_pos.pos

        constraints = IK.Constraints()

        rl_constraint = IK.PositionConstraint()
        rl_constraint.A_link = self.rleg_tip_link
        rl_constraint.A_localpos = self.rleg_tip_to_eef
        #constraint->B_link() = nullptr;
        rl_constraint.B_localpos = self.rleg_end_effector().toPosition()
        constraints.push_back(rl_constraint)

        ll_constraint = IK.PositionConstraint()
        ll_constraint.A_link = self.lleg_tip_link
        ll_constraint.A_localpos = self.lleg_tip_to_eef
        #constraint->B_link() = nullptr;
        ll_constraint.B_localpos = self.lleg_end_effector().toPosition()
        constraints.push_back(ll_constraint)

        com_constraint = IK.COMConstraint()
        com_constraint.A_robot = self.robot
        com_constraint.B_localp = mid_trans
        w = com_constraint.weight
        w[2] = 0.0
        com_constraint.weight = w
        constraints.push_back(com_constraint)

        jlim_avoid_weight_old = np.zeros(6 + self.robot.getNumJoints())
        ##dq_weight_all = np.ones(6 + self.robot.getNumJoints())
        dq_weight_all = np.array( (1,1,0, 0,0,0) + self.leg_mask() )

        d_level = 0
        if debug:
            d_level = 1
            for const in constraints:
                const.debuglevel = 1

        loop = IK.solveFullbodyIKLoopFast(self.robot,
                                          constraints,
                                          jlim_avoid_weight_old,
                                          dq_weight_all,
                                          20,
                                          1e-6,
                                          d_level)
        if debug:
            for cntr, const in enumerate(constraints):
                const.debuglevel = 1
                if const.checkConvergence():
                    print('constraint %d (%s) : converged'%(cntr, const))
                else:
                    print('constraint %d (%s) : NOT converged'%(cntr, const))

        return loop

    def move_centroid_on_foot_qp(self, p = 0.5, debug = False):
        pass

    def move_centroid_on_foot(self, p = 0.5, debug = False):
        return self.move_centroid_on_foot_cnoid(p = p, debug = debug)

    def fullbody_inverse_kinematics_cnoid(self):
        pass
    def fullbody_inverse_kinematics_qp(self):
        pass
    def fullbody_inverse_kinematics(self):
        return self.fullbody_inverse_kinematics_cnoid()

def merge_mask(tp1, tp2):
    return tuple([x or y for (x, y) in zip(tp1, tp2)])
def invert_mask(tp1):
    return tuple([0 if x else 1 for x in tp1])
