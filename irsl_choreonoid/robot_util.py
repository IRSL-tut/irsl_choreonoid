import cnoid.Body
#import cnoid.Util

from .cnoid_util import *

import cnoid.IRSLCoords as ic
import cnoid.IKSolvers as IK

import numpy as np
import random
import math

def make_coordinates(coords_map):
    """Generating coordinates(cnoid.IRSLCoords.coordinates) from dictionary

    Args:
        coords_map (dict[str, list[float]]) : dictionary of describing transformation

    Returns:
        cnoid.IRSLCoords.coordinates : generated coordinates

    Raises:
        Exeption : If there is not valid keyword

    Examples:

        >>> make_coordinates( {'position' : [1, 2, 3]} )
        <coordinates[address] 1 2 3 / 0 0 0 1 >

        >>> make_coordinates( {'pos' : [1, 2, 3], 'rpy' : [math.pi/3, 0, 0]} )
        <coordinates[address] 1 2 3 / 0.5 0 0 0.866025 >

        >>> make_coordinates({'rot' : [[0, -1, 0],[1, 0, 0], [0, 0, 1]]})
        <coordinates[address] 0 0 0 / 0 0 0.707107 0.707107 >

        >>> make_coordinates( {'angle-axis' : [0, 1, 0, math.pi/4] })
        <coordinates[address] 0 0 0 / 0 0.382683 0 0.92388 >

        >>> make_coordinates( {'quaternion' : [0, 0, 0, 1] })
        <coordinates[address] 0 0 0 / 0 0 0 1 >

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
    for key in ('angle-axis', 'angle_axis', 'aa'):
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

def make_coords_map(coords, method=None):
    """Generating dictonary describing transformation

    Args:
        coords (cnoid.IRSLCoords.coordinates) : Transformation

    Returns:
        dict[str, list[float]] : Dictonary can be used by make_coordinates

    Examples:
        >>> make_coords_map( make_coordinates( {'position' : [1, 2, 3]} ) )
        {'pos': [1.0, 2.0, 3.0], 'aa': [1.0, 0.0, 0.0, 0.0]}
    """
    if method is None:
        return {'pos': coords.pos.tolist(), 'aa': coords.getRotationAngle().tolist()}
    elif method in ('RPY', 'rpy'):
        return {'pos': coords.pos.tolist(), method: coords.getRPY().tolist()}
    elif method in ('q', 'quaternion'):
        return {'pos': coords.pos.tolist(), method: coords.quaternion.tolist()}
    elif method in ('aa', 'angle_axis', 'angle-axis'):
        return {'pos': coords.pos.tolist(), method: coords.getRotationAngle().tolist()}
    elif method in ('rotation', 'matrix', 'mat', 'rot'):
        return {'pos': coords.pos.tolist(), method: coords.rot.tolist()}
    else:
        raise Exception('method:{} is invalid'.format(method))

def make_translation_rotation(coords, unit='mm', degree=True):
    """Generating dictonary describing transformation (using translation, rotation)

    Args:
        coords (cnoid.IRSLCoords.coordinates) : Transformation
        unit (str, default='mm') : Unit of length. 'm', 'mm', 'cm', or 'inch'
        degree (boolean, default=True) : Unit of angle. If True, use degree, otherwise use radian

    Returns:
        dict[str, list[float]] : Dictonary can be used in .body

    """
    res = make_coords_map(coords)
    pp = res['pos']
    aa = res['aa']
    if unit == 'mm':
        pp[0] *= 1000.0
        pp[1] *= 1000.0
        pp[2] *= 1000.0
    elif unit == 'cm':
        pp[0] *= 100.0
        pp[1] *= 100.0
        pp[2] *= 100.0
    elif unit == 'inch':
        pp[0] *= 25.4
        pp[1] *= 25.4
        pp[2] *= 25.4
    if degree:
        aa[3] = aa[3]*180/math.pi
    return {'translation': pp, 'rotation': aa}

##
## IKWrapper
##
class IKWrapper(object):
    """Utility class for solving inverse-kinematics
    """
    def __init__(self, robot, tip_link, tip_to_eef = None, use_joints = None, solver = 'QP'):
        """
        Args:
            robot (cnoid.Body.Body) : robot model using this instance
            tip_link (str, cnoid.Body.Link) : name or instance of tip_link
            tip_to_eef (cnoid.IRSLCoords.coordinates, optional) : coordinates of end-effector relative to tip_link
            use_joints (list[str, int, cnoid.Body.Link] , optional) : list of joint name, index or instance
            solver (str, default = 'QP') : type of solver

        Note:
            Coordinates system

            world -> robot(root_link) -> (using joints) -> tip_link -> tip_to_eef -> end-effector < -- > target-coords(world)

            Inverse-kinematics would be solved to make target-coords and end-effector at the same position.

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
        self.__default_coords = ic.coordinates(self.__robot.rootLink.T)

        if solver == 'QP':
            self.__inverseKinematics = self.__inverseKinematicsQP
        else:
            self.__inverseKinematics = self.__inverseKinematicsLM

    def flush(self):
        """Updating the robot in scene

        Args:
            None

        """
        self.__robot.calcForwardKinematics()
        if isInChoreonoid(): ## only have effects while running in choreonoid
            ret = findBodyItem(self.__robot)
            if ret is not None:
                ret.notifyKinematicStateUpdate()

    def updateDefault(self):
        """Updating default values (using joints, default_pose, default_coords)

        Args:
            None

        """
        self.__default_joints = self.__current_joints
        self.__default_pose = self.angleVector()
        self.__default_coords = ic.coordinates(self.__robot.rootLink.T)
        self.resetJointWeights()

    def __parseLink(self, id_name_link):
        if type(id_name_link) is int:
            return self.__robot.link(id_name_link)
        elif type(id_name_link) is str:
            res =  self.__robot.joint(id_name_link)
            if res is None:
                return self.__robot.link(id_name_link)
            else:
                return res
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

    @property
    def endEffector(self):
        """Current 6DOF coordinates of end-effector

        Returns:
            cnoid.IRSLCoords.coordinates : coordinates of current end-effector

        """
        return ic.coordinates(self.__tip_link.getPosition().dot(self.__tip_to_eef_cnoid))

    def getEndEffector(self, **kwargs):
        """Getting end-effector (function version of self.endEffector)

        Args:
            kwargs (dict[str, param]) : ignored

        Returns:
            cnoid.IRSLCoords.coordinates : end-effector

        """
        return ic.coordinates(self.__tip_link.getPosition().dot(self.__tip_to_eef_cnoid))

    def resetJointWeights(self):
        """Reset current joint-list to default joint-list (reverting effects of using setJoints)

        Args:
            None

        """
        self.__current_joints = [j for j in self.__default_joints]
        self.updateJointWeights()

    def updateJointWeights(self): ## internal method?
        jlst = self.__robot.jointList()
        for j in self.__current_joints:
            if not j in jlst:
                print('### Warning ### joint: {} (id: {}, link_name: {}) is not a valid joint'.format(j.jointName, j.joint_id, j.name))
        self.__joint_weights = np.zeros(self.__robot.numJoints)
        for idx in range(self.__robot.numJoints):
            if self.__robot.joint(idx) in self.__current_joints:
                self.__joint_weights[idx] = 1

    def setJoints(self, jlist, enable = True):
        """Adding or removing joints to/from using joint-list

        Args:
            jlist (list[str, int, cnoid.Body.Link]) : list of joint name, index or instance
            enable (boolean, default = True) : if True, joints are added, else joint are removed from using joint-list

        Raise:
            Exception : If wrong joint is passed

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
        elif type(joint_or_id) == str:
            joint_or_id = self.__robot.joint(joint_or_id)
            if joint_or_id is None:
                raise Exception('wrong joint name : {}'.format(joint_or_id))
        else:
            raise Exception('wrong type')

        if enable:
            if not joint_or_id in self.__current_joints:
                self.__current_joints.append(joint_or_id)
        else:
            while joint_or_id in self.__current_joints:
                self.__current_joints.remove(joint_or_id)

    def inverseKinematics(self, target, revert_if_failed=True, retry=100, **kwargs):
        """Top method to solve inverse-kinematics

        Args:
            target (cnoid.IRSLCoords.coordinates) : target coordinates of inverse-kinematics
            revert_if_failed (boolean, default = True) : if True, no modification (moving joint and root-link) to robot-model when calculation of IK is failed
            retry (int, defualt = 100) : number of retrying solving IK
            constraint (str or list[float], optional) : '6D', 'position', 'rotation', 'xyzRPY'
            weight (float, default = 1.0) : weight of constraint
            base_type (str or list[float], optional) : '2D', 'planer', 'position', 'rotation'
            base_weight (float, default = 1.0) : weight of base movement
            max_iteration (int, defulat = 32) : number of iteration
            threshold (float, default = 5e-5) : threshold for checking convergence
            add_noise (float or boolean, optional) : if True or number, adding noise to angle of joint before solving IK
            debug (boolean, default = False) : if True, printing debug message

        Returns:
             (boolean, int) : IK was success or not, and total count of calculation

        Note:
              constraint : [1,1,1, 1,1,1]
              base_type : [1,1,1, 1,1,1]

        """
        init_angle = self.angleVector()
        init_coords = self.rootCoords()

        succ = False
        _total = 0
        while retry >= 0:
            succ, _iter = self.__inverseKinematics(target, **kwargs)
            _total += _iter
            if succ:
                break
            retry -= 1
        if (not succ) and revert_if_failed:
            self.angleVector(init_angle)
            self.rootCoords(init_coords)
        return (succ, _total)

    def __inverseKinematicsQP(self, target, constraint = None, weight = 1.0, add_noise = None, debug = False,
                              base_type = None, base_weight = 1.0, max_iteration = 32, threshold = 5e-5,
                              use_joint_limit=True, joint_limit_max_error=1e-2, joint_limit_precision=0.1, **kwargs):
        ## add_noise
        if add_noise is not None:
            if type(add_noise) is float:
                self.addNoise(max_range = add_noise, joint_list = self.__current_joints)
            else:
                self.addNoise(max_range = 0.2, joint_list = self.__current_joints)
        ## constraint
        if constraint is None or constraint == '6D':
            constraint = [1, 1, 1, 1, 1, 1]
        elif constraint == 'position':
            constraint = [1, 1, 1, 0, 0, 0]
        elif constraint == 'rotation':
            constraint = [0, 0, 0, 1, 1, 1]
        elif type(constraint) is str:
            ## 'xyzRPY'
            wstr = constraint
            constraint = [0, 0, 0, 0, 0, 0]
            for ss in wstr:
                if ss == 'x':
                    constraint[0] = 1
                elif ss == 'y':
                    constraint[1] = 1
                elif ss == 'z':
                    constraint[2] = 1
                elif ss == 'R':
                    constraint[3] = 1
                elif ss == 'P':
                    constraint[4] = 1
                elif ss == 'Y':
                    constraint[5] = 1
        ## base_type
        if base_type == '2D' or base_type == 'planer':
            base_const = np.array([0, 0, 1, 1, 1, 0])
        elif base_type == 'position':
            base_const = np.array([0, 0, 0, 1, 1, 1])
        elif base_type == 'rotation':
            base_const = np.array([1, 1, 1, 0, 0, 0])
        elif type(base_type) is str:
            ## 'xyzRPY'
            wstr = base_type
            _bweight = [0, 0, 0, 0, 0, 0]
            for ss in wstr:
                if ss == 'x':
                    _bweight[0] = 1
                elif ss == 'y':
                    _bweight[1] = 1
                elif ss == 'z':
                    _bweight[2] = 1
                elif ss == 'R':
                    _bweight[3] = 1
                elif ss == 'P':
                    _bweight[4] = 1
                elif ss == 'Y':
                    _bweight[5] = 1
            base_const = np.array(_bweight)
        elif base_type is not None:
            base_const = np.array(base_type)
        ##
        if base_type is not None:
            base_const = base_weight * base_const

        ### constraints for IK
        constraints0 = IK.Constraints()
        a_constraint = IK.PositionConstraint()
        a_constraint.A_link =     self.__tip_link
        a_constraint.A_localpos = self.__tip_to_eef_cnoid
        #constraint.B_link() = nullptr;
        a_constraint.B_localpos = target.toPosition()
        a_constraint.weight     = weight * np.array(constraint)
        constraints0.push_back(a_constraint)
        if base_type is not None:
            if debug:
                print('use base : {}'.format(base_const))
            b_constraint = IK.PositionConstraint()
            b_constraint.A_link =     self.__robot.rootLink
            b_constraint.A_localpos = ic.coordinates().cnoidPosition
            #constraint.B_link() = nullptr;
            b_constraint.B_localpos = self.__robot.rootLink.T
            b_constraint.weight     = np.array(base_const)
            constraints0.push_back(b_constraint)
        #
        tasks = IK.Tasks()
        dummy_const = IK.Constraints()
        constraints = [ dummy_const, constraints0 ]
        ### constraint joint-limit
        if use_joint_limit:
            constraints1 = IK.Constraints()
            for j in self.__current_joints:
                const = IK.JointLimitConstraint()
                const.joint = j
                const.precision = 0.1
                constraints1.push_back(const)
            constraints.append(constraints1)
        #
        variables = []
        if base_type is not None:
            variables.append(self.__robot.rootLink)
        variables += self.__current_joints
        if debug:
            print('var: {}'.format(variables))
        #
        d_level = 0
        if debug:
            d_level = 1
        loop = IK.prioritized_solveIKLoop(variables, constraints, tasks,
                                          max_iteration, threshold, d_level)
        if debug:
            for cntr, consts in enumerate(constraints):
                for idx, const in enumerate(consts):
                    const.debuglevel = 1
                    if const.checkConvergence():
                        print('constraint {}-{} ({}) : converged'.format(cntr, idx, const))
                    else:
                        print('constraint {}-{} ({}) : NOT converged'.format(cntr, idx, const))
        conv = True
        for cntr, consts in enumerate(constraints):
            for const in consts:
                if not const.checkConvergence():
                    conv = False
                    break
            if not conv:
                break
        return (conv, loop)

    def __inverseKinematicsLM(self, target, constraint = None, weight = 1.0, add_noise = None, debug = False,
                              base_type = None, base_weight = 1.0, max_iteration = 32, threshold = 5e-5, **kwargs):
        ## add_noise
        if add_noise is not None:
            if type(add_noise) is float:
                self.addNoise(max_range = add_noise, joint_list = self.__current_joints)
            else:
                self.addNoise(max_range = 0.2, joint_list = self.__current_joints)
        ## constraint
        if constraint is None or constraint == '6D':
            constraint = [1, 1, 1, 1, 1, 1]
        elif constraint == 'position':
            constraint = [1, 1, 1, 0, 0, 0]
        elif constraint == 'rotation':
            constraint = [0, 0, 0, 1, 1, 1]
        elif type(constraint) is str:
            ## 'xyzRPY'
            wstr = constraint
            constraint = [0, 0, 0, 0, 0, 0]
            for ss in wstr:
                if ss == 'x':
                    constraint[0] = 1
                elif ss == 'y':
                    constraint[1] = 1
                elif ss == 'z':
                    constraint[2] = 1
                elif ss == 'R':
                    constraint[3] = 1
                elif ss == 'P':
                    constraint[4] = 1
                elif ss == 'Y':
                    constraint[5] = 1
        ## base_type
        if base_type == '2D' or base_type == 'planer':
            base_const = np.array([0, 0, 1, 1, 1, 0])
        elif base_type == 'position':
            base_const = np.array([0, 0, 0, 1, 1, 1])
        elif base_type == 'rotation':
            base_const = np.array([1, 1, 1, 0, 0, 0])
        elif type(base_type) is str:
            ## 'xyzRPY'
            wstr = base_type
            _bweight = [0, 0, 0, 0, 0, 0]
            for ss in wstr:
                if ss == 'x':
                    _bweight[0] = 1
                elif ss == 'y':
                    _bweight[1] = 1
                elif ss == 'z':
                    _bweight[2] = 1
                elif ss == 'R':
                    _bweight[3] = 1
                elif ss == 'P':
                    _bweight[4] = 1
                elif ss == 'Y':
                    _bweight[5] = 1
            base_const = np.array(_bweight)
        elif base_type is not None:
            base_const = np.array(base_type)
        else:
            base_const = np.ones(6)
        _base_weight = base_weight * (np.ones(6) - base_const)
        ### constraints for IK
        constraints = IK.Constraints()
        ra_constraint = IK.PositionConstraint()
        ra_constraint.A_link =     self.__tip_link
        ra_constraint.A_localpos = self.__tip_to_eef_cnoid
        #constraint->B_link() = nullptr;
        ra_constraint.B_localpos = target.toPosition()
        ra_constraint.weight = weight * np.array(constraint)
        constraints.push_back(ra_constraint)
        ##
        jlim_avoid_weight_old = np.zeros(6 + self.__robot.getNumJoints())
        ##dq_weight_all = np.ones(6 + self.robot.getNumJoints())
        dq_weight_all = np.append(_base_weight, self.__joint_weights)
        d_level = 0
        if debug:
            d_level = 1
        ## solve IK
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
        conv = False
        for cntr, const in enumerate(constraints):
            if const.checkConvergence():
                conv = True
                break
        return (conv, loop)

    #def angleVectorOrg(self, av = None):
    #    if av is not None:
    #        for idx in range(len(self.__default_joints)):
    #            self.__default_joints[idx].q = av[idx]
    #    return np.array([ j.q for j in self.__default_joints ])
    ##
    def angleVector(self, _angle_vector = None):
        """Returning current joint angle of self.robot
        ロボット(self.robot)への関節角度の指定して現状の値を返す。

        Args:
            _angle_vector (numpy.array, optional) : 1 x len(self.default_joints) vector
            各エレメントが関節角度になっているvector(numpy.array)。要素数はself.default_jointと同じ。

        Returns:
            numpy.array : 1 x len(self.default_joints) vector
            現在のロボットの状態で、各エレメントが関節角度になっているvector(numpy.array)。要素数はself.default_jointと同じ。

        """
        return self.__angleVector(_angle_vector, self.__default_joints)
    def currentAngleVector(self, _angle_vector = None):
        """Returning current joint angle of self.robot

        Args:
            _angle_vector (numpy.array, optional) : 1 x len(self.current_joints) vector

        Returns:
            numpy.array : 1 x len(self.current_joints) vector

        Note:
            self.current_joints would be changed by setJoints method

        """
        return self.__angleVector(_angle_vector, self.__current_joints)
    def __angleVector(self, av, joint_list):
        if av is not None:
            for j, ang in zip(joint_list, av):
                j.q = ang
            self.__robot.calcForwardKinematics()
        return np.array([ j.q for j in joint_list])

    def rootCoords(self, cds = None):
        """Setting and getting coordinates of rootLink

        Args:
            cds (cnoid.IRSLCoords.coordinates, optional) : If argument is set, coordinates of rootLink is updated by it

        Returns:
            cnoid.IRSLCoords.coordinates : coordinates of rootLink

        """
        if cds is not None:
            self.__robot.rootLink.T = cds.cnoidPosition
            self.__robot.calcForwardKinematics()
        return self.__robot.rootLink.getCoords()

    def resetPose(self):
        """Resetting pose (angleVector and rootCoords)

        Args:
            None

        Returns:
            numpy.array : current angleVector

        """
        self.__robot.rootLink.T = self.__default_coords.cnoidPosition
        return self.angleVector(self.__default_pose)

    def addNoise(self, max_range = 0.1, joint_list = None):
        """addNoise(self, max_range = 0.1, joint_list = None):

        Args:
            max_range (float, default = 0.1) : range of adding noise
            joint_list (list[str, int, cnoid.Body.Link], optional) : If argument is not set, self.default_joints would be used

        """
        if joint_list is None:
            joint_list = self.__default_joints
        for j in joint_list:
            j.q += random.uniform(-max_range, max_range)
        self.__robot.calcForwardKinematics()

    ## read-only
    @property
    def robot(self):
        """Robot model using this instance

        Returns:
            cnoid.Body.Body : robot model

        """
        return self.__robot
#    @body.setter
#    def body(self, in_body):
#        self.__body = in_body
    @property
    def tip_link(self):
        """Tip link (direct parent of end-effector)

        Returns:
            cnoid.Body.Link : tip link

        """
        return self.__tip_link
    @property
    def tip_to_eef(self):
        """Transformation between origin of tip_link and end-effector

        Returns:
           cnoid.IRSLCoords.coordinates : Transformation to end-effector at tip_link coordinates

        """
        return self.__tip_to_eef
    @property
    def joint_weights(self):
        """Weight of joints

        Returns:
            list[float] : weight of joints

        """
        return self.__joint_weights
    @property
    def current_joints(self):
        """Current joint-list

        Returns:
            list[cnoid.Body.Link] : current_joints using IK, currentAngleVector

        """
        return self.__current_joints
    @property
    def default_joints(self):
        """Default joint-list (stored while initialization)

        Returns:
            list[cnoid.Body.Link] : default_joints

        """
        return self.__default_joints
    @property
    def default_pose(self):
        """AngleVector stored while initialization

        Returns:
            numpy.array : 1 x len(self.default_joints) vector

        """
        return self.__default_pose
    @property
    def default_coords(self):
        """RootCoords stored while initialization

        Returns:
            cnoid.IRSLCoords.coordinates : Transformation to end-effector at tip_link coordinates

        """
        return self.__default_coords

## add methods to choreonoid's class
def __joint_list(self):
    return [self.joint(idx) for idx in range(self.numJoints) ]
#def __link_list(self):
#    return [self.link(idx) for idx in range(self.numLinks) ]

cnoid.Body.Body.jointList = __joint_list
#cnoid.Body.Body.linkList = __link_list
cnoid.Body.Body.angleVector = lambda self, vec = None: ic.angleVector(self) if vec is None else ic.angleVector(self, vec)
cnoid.Body.Link.getCoords = lambda self: ic.getCoords(self)
cnoid.Body.Link.setCoords = lambda self, cds: ic.setCoords(self, cds)
cnoid.Body.Link.getOffsetCoords = lambda self: ic.getOffsetCoords(self)
cnoid.Body.Link.setOffsetCoords = lambda self, cds: ic.setOffsetCoords(self, cds)

class RobotModel(object):
    def __init__(self, robot):
        self.item = None
        if hasattr(robot, 'body'): ## check BodyItem
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
        return self.robot.links()
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

from .irsl_draw_object import coordsWrapper
class RobotModelWrapped(coordsWrapper): ## with wrapper
    """RobotModel for programming Inteactively

    RobotModel is displayed as cnoid.Base.RobotItem
    """
    def __init__(self, robot, **kwargs):
        """
        Args:
            robot (cnoid.Body.Body or cnoid.BodyPlugin.BodyItem) : robot model using this class

        """
        self.__keep_limit = False

        self.__item = None
        if hasattr(robot, 'body'): ## check BodyItem ##if isinstance(robot, BodyItem):
            self.__robot = robot.body
            self.__item  = robot
        elif isinstance(robot, cnoid.Body.Body):
            self.__robot = robot
        else:
            raise TypeError('')

        if isInChoreonoid():
            self.__mode = 1 ## 1: drawing
        else:
            self.__mode = 0 ## 0: kinematics
        ## initialize super after setting __robot
        super().__init__(target = self, update_callback = lambda : self.hook() )
        self.newcoords(ic.coordinates(self.__robot.rootLink.T))

        self.pose_angle_map = {}
        self.pose_coords_map = {}

        ##
        self.__joint_list = self.__robot.jointList()
        self.__joint_map = {}
        for j in self.__joint_list:
            self.__joint_map[j.jointName] = j
        ##
        self.__link_list = self.__robot.links
        self.__link_map = {}
        for lk in self.__link_list:
            self.__link_map[lk.name] = lk
        ##
        self.__device_list = self.__robot.devices
        self.__device_map = {}
        for d in self.__device_list:
            name = d.name
            if len(name) < 1:
                name = '{}{}_{}_{}'.format(d.typeName, d.id, d.index, d.link().name)
            while name in self.__device_map:
                name+='+'
            self.__device_map[name] = d
            #d.name=name
        self.eef_map = {}

    def registerNamedPose(self, name, angles = None, root_coords = None):
        """Registering named pose for using with irsl_choreonoid.robot_util.RobotModelWrapped.setNamedPose

        Args:
            name (str) : Name of registered pose
            angles (numpy.array or list[float]) : Angle-vector to be able to pass irsl_choreonoid.robot_util.RobotModelWrapped.angleVector
            root_coords (cnoid.IRSLCoords.coordinates) : Coordinates of Root

        """
        if angles is not None:
            self.pose_angle_map[name] = np.array(angles)
        if root_coords is not None:
            self.pose_coords_map[name] = root_coords

    def registerEndEffector(self, name, tip_link, tip_link_to_eef=None, joint_list=None, joint_tuples=None):
        """Registering named pose for using with irsl_choreonoid.robot_util.RobotModelWrapped.getLimb, etc.

        Args:
            name (str) : Name of limb as registered end-effector
            tip_link (str or cnoid.Body.Link) : Link which is direct parent of coordinates of end-effector
            tip_link_to_eef (cnoid.IRSLCoords.coordinates) : Coordinates tip_link to end_effector represented on link's coordinate
            joint_list (list[str] or list[cnoid.Body.Link]) :
            joint_tuples ( tuple((actual_name:str, nickname:str)) ) : Tuple of actual_name and nick_name

        """
        eef = self.EndEffector(self.__robot, name, tip_link, tip_link_to_eef, joint_list, joint_tuples,
                               hook = lambda : self.hook() )
        self.eef_map[name] = eef

    class EndEffector(object):
        """EndEffector class for manipulating end-effector, limb, joint-group
        """
        def __init__(self, robot, name, tip_link, tip_link_to_eef=None, joint_list=None, joint_tuples=None, hook=None):
            """Use RobotModelWrapped.registerEndEffector
            """
            self.__robot = robot
            self.__name = name
            if tip_link_to_eef is None:
                self.__tip_link_to_eef = ic.coordinates()
            else:
                self.__tip_link_to_eef = tip_link_to_eef
            self.__joint_list = []
            self.__joint_map = {}
            self.__rename_map = {}
            self.__hook = hook
            ##
            if joint_tuples is not None:
                self.__genJointList(joint_tuples)
            elif joint_list is not None:
                for jnm in joint_list:
                    j = self.__robot.joint(jnm)
                    if j is None:
                        ### warning
                        pass
                    else:
                        self.__joint_map[jnm] = j
                        self.__joint_list.append(j)
            ##
            if type(tip_link) is str:
                lk = self.__robot.link(tip_link)
                if lk is None:
                    lk = self.__robot.joint(tip_link)
                    if lk is None:
                        lk = self.__robot.joint(self.rename(tip_link))
                self.__tip_link = lk
            else:
                self.__tip_link = tip_link
            ##
            self.__ikw = IKWrapper(self.__robot, self.__tip_link, tip_to_eef=self.__tip_link_to_eef,
                                   use_joints=self.__joint_list if self.__joint_list is not None else None)

        def __genJointList(self, joint_tuples): ## joint_tuples is not None ((real_name, nick_name), ...
            _lst = []
            _jmap = {}
            _rmap = {}
            for jt in joint_tuples:
                if type(jt) is tuple and len(jt) > 1:
                    _rmap[jt[1]] = jt[0]
                    jt = jt[0]
                elif type(jt) is str:
                    pass ## jt = jt
                else:
                    ### warning
                    continue
                j = self.__robot.joint(jt)
                if j is None:
                    ### warning
                    pass
                else:
                    _lst.append(j)
                    _jmap[jt] = j
            self.__joint_list = _lst
            self.__joint_map  = _jmap
            self.__rename_map = _rmap

        def rename(self, nick_name):
            """Renaming using registered nick-name

            Args:
                nick_name (str) : nick-name

            Returns:
                str : actual name

            """
            if self.__rename_map is None:
                return nick_name
            if nick_name in self.__rename_map:
                return self.__rename_map[nick_name]
            else:
                return nick_name

        @property
        def endEffector(self):
            """Getting coordinate of the end-effector (return value is generated on demand)

            Returns:
                cnoid.IRSLCoords.coordinates : Coordinate of the end-effector

            """
            ret = self.__tip_link.getCoords()
            ret.transform(self.__tip_link_to_eef)
            return ret

        @property
        def IK(self):
            """Getting IKWrapper instance using this EndEffector

            Returns:
                irsl_choreonoid.robot_util.IKWrapper : IKWrapper instance using this EndEffector

            """
            return self.__ikw

        @property
        def jointList(self):
            return self.__joint_list

        @property
        def jointNames(self):
            return [ j.jointName for j in self.__joint_list ]

        @property
        def renameMap(self):
            return self.__rename_map

        @property
        def angleMap(self):
            """Getting dictionary [ key='jointName', value=jointAngle ]

            Returns:
                dict [str, float ] : Dictionary of key='jointName', value=jointAngle for joints in this Limb

            """
            amap = {}
            for j in self.__joint_list:
                amap[j.jointName] = j.q
            return amap

        def inverseKinematics(self, coords, **kwargs):
            """Solving inverse kinematic on this limb

            Args:
                coords (cnoid.IRSLCoords.coordinates) : Coordinate of the target
                \*\*kwargs ( dict ) : Just passing to IKWrapper

            Returns:
                 (boolean, int) : IK was success or not, and total count of calculation

            """
            result = self.__ikw.inverseKinematics(coords, **kwargs)
            if self.__hook is not None:
                self.__hook()
            return result

        def jointAngle(self, name, angle = None):
            """Setting or getting angle of the joint (limb version)

            Args:
                name (str) : Name of a joint, 'nickname' is acceptable.
                angle (float, optional) : Angle [radian] to be set

            Returns:
                float : Angle of the joint, if do not find the name, None would be returned

            """
            if self.__joint_list is None:
                ### warning
                return None
            res_ = None
            nm = self.rename(name)
            if nm in self.__joint_map:
                if angle is not None:
                    self.__joint_map[nm].q = angle
                    if self.__hook is not None:
                        self.__hook()
                res_ = self.__joint_map[nm].q
            else:
                pass
                ### warning
            return res_

        def setAngleMap(self, angle_map):
            """Setting angles of the joint (limb version)

            Args:
                angle_map (dict[str, float]) : Keyword is a joint name and value is a joint angle. 'nickname' is acceptable.

            Returns:
                boolean : If angles is set, returns True

            """
            if self.__joint_list is None:
                ### warning
                return False
            res = True
            for k, v in angle_map.items():
                nm = self.rename(k)
                if nm in self.__joint_map:
                    self.__joint_map[nm].q = v
                else:
                    res = False
            if self.__hook is not None:
                self.__hook()
            return res

        def angleVector(self, angle_vector = None):
            """Setting or getting angle-vector (limb version)

            Args:
                angle_vector (numpy.array) : Vector of angle[radian] to be set

            Returns:
                numpy.array : Vector of angle[radian]

            """
            if self.__joint_list is None:
                ### warning
                return None
            if angle_vector is not None:
                for j, ang in zip(self.__joint_list, angle_vector):
                    j.q = ang
            ret = [ j.q for j in self.__joint_list ]
            if self.__hook is not None:
                self.__hook()
            return np.array(ret)

    @property
    def robot(self):
        """Robot-model using wrapped by this instance

        Returns:
            cnoid.Body.Body : Instance of robot model currently using

        """
        return self.__robot

    @property
    def T(self):
        return self.__robot.rootLink.T
    @T.setter
    def T(self, _in):
        self.__robot.rootLink.T = _in

    def setMode(self, mode = None):
        """Set updating mode

        Args:
            mode (int, optional) : Mode to be set (0\: kinematics mode, 1\: rendering mode, 2\: rendering immediately mode)

        Returns:
            int : Current mode (0\: kinematics mode, 1\: rendering mode, 2\: rendering immediately mode, -1\: do nothing while hook)

        """
        if mode is not None:
            self.__mode = mode
        return self.__mode

    @property
    def linkList(self):
        """List of links

        Returns:
            list [cnoid.Body.Link] : List of links

        """
        return self.__link_list

    @property
    def linkNames(self):
        """List of link names

        Returns:
            list [str] : List of link names

        """
        return list(self.__link_map.keys())

    @property
    def jointList(self):
        """List of joints

        Returns:
            list [cnoid.Body.Link] : List of joints

        """
        return self.__joint_list

    @property
    def jointNames(self):
        """List of joint names

        Returns:
            list [cnoid.Body.Link] : List of joint names

        """
        return list(self.__joint_map.keys())

    @property
    def deviceList(self):
        """List of devices

        Returns:
            list [cnoid.Body.Device] : List of devices

        """
        return self.__device_list

    @property
    def deviceNames(self):
        """List of device names

        Returns:
            list [cnoid.Body.Device] : List of device names

        """
        return list(self.__device_map.keys())

    def link(self, arg):
        """Instance of the link

        Args:
            arg (str, int, object) : name or index of the link

        Returns:
            cnoid.Body.Link : instance of the link

        """
        tp = type(arg)
        if tp is int:
            return self.__link_list[arg]
        elif tp is str:
            if arg in self.__link_map:
                return self.__link_map[arg]
            else:
                return self.__robot.link(arg)
        elif tp is cnoid.Body.Link:
            return arg
        return self.__robot.link(arg)

    def joint(self, arg):
        """Instance of the joint

        Args:
            arg (str, int, object) : name or index of the joint

        Returns:
            cnoid.Body.Link : instance of the joint

        """
        tp = type(arg)
        if tp is int:
            return self.__joint_list[arg]
        elif tp is str:
            if arg in self.__joint_map:
                return self.__joint_map[arg]
            else:
                self.__robot.joint(arg)
        elif tp is cnoid.Body.Link:
            if arg in self.__joint_list:
                return arg
        return self.__robot.joint(arg)

    def device(self, arg):
        """Instance of the device

        Args:
            arg (str, int, object) : name or index of the device

        Returns:
            cnoid.Body.Device : instance of the device

        """
        tp = type(arg)
        if tp is int:
            return self.__device_list[arg]
        elif tp is str:
            if arg in self.__device_map:
                return self.__device_map[arg]
            else:
                self.__robot.device(arg)
        elif tp is cnoid.Body.Device:
            return arg
        return self.__robot.device(arg)

    def linkCoords(self, str_idx_instance):
        """Coordinates of the link

        Args:
            str_index_instance (str, int, object) : name or index of the link

        Returns:
            cnoid.IRSLCoords.coordinates : coordinate of the link

        """
        lk = self.link(str_idx_instance)
        if lk is None:
            return None
        return lk.getCoords()

    def jointCoords(self, str_idx_instance):
        """Coordinates of the joint

        Args:
            str_index_instance (str, int, object) : name or index of the joint

        Returns:
            cnoid.IRSLCoords.coordinates : coordinate of the joint

        """
        jt = self.joint(str_idx_instance)
        if jt is None:
            return None
        return jt.getCoords()

    def deviceCoords(self, str_idx_instance):
        """Coordinates of the device

        Args:
            str_index_instance (str, int, object) : name or index of the device

        Returns:
            cnoid.IRSLCoords.coordinates : coordinate of the device

        """
        dev = self.device(str_idx_instance)
        if dev is None:
            return None
        p_cds = dev.getLink().getCoords()
        p_cds.transform(coordinates(dev.T_local))
        return p_cds

    def angleVector(self, angles = None):
        """Setting or getting angle-vector

        Args:
            angle_vector (numpy.array) : Vector of angle[radian] to be set

        Returns:
            numpy.array : Vector of angle[radian]

        """
        res_ = self.__robot.angleVector(angles)
        if angles is not None:
            self.hook()
        return res_

    def jointAngle(self, name, angle=None):
        """Setting or getting angle of the joint

        Args:
            name (str) : Name of a joint
            angle (float, optional) : Angle [radian] to be set

        Returns:
            float : Angle of the joint, if do not find the name, None would be returned

        """
        if self.__joint_list is None:
            ### warning
            return None
        res_ = None
        if name in self.__joint_map:
            if angle is not None:
                self.__joint_map[name].q = angle
                self.hook()
            res_ = self.__joint_map[name].q
        return res_

    def setAngleMap(self, angle_map):
        """Setting angles of the joint

        Args:
            angle_map (dict[str, float]) : Keyword is a joint name and value is a joint angle. 'nickname' is acceptable.

        Returns:
            boolean : If angles is set, returns True

        """
        if self.__joint_list is None:
            ### warning
            return False
        res = True
        for k, v in angle_map.items():
            if k in self.__joint_map:
                self.__joint_map[k].q = v
            else:
                res = False
        self.hook()
        return res

    def rootCoords(self, coords = None):
        """Getting or setting coordinates of root link of robot

        Args:
            coords (cnoid.IRSLCoords.coordinates, optional) : Coordinate to be set

        Returns:
            cnoid.IRSLCoords.coordinates : Current coordinates of root link of robot

        """
        if coords is not None:
            self.newcoords(coords)
        return self.copy()

    def hook(self):
        if self.__keep_limit:
            self.trimJointAngles()
        if self.__mode < 0:
            ### do nothing(no-hook)
            return
        if self.__mode == 0: ## kinematics only
            self.__robot.calcForwardKinematics()
        elif self.__mode == 1: ## render
            self.flush()
        elif self.__mode == 2: ## render Immediately
            self.flush(True)

    def flush(self, updateGui=False):
        if self.__robot is not None:
            self.__robot.calcForwardKinematics()
        if self.__item is not None:
            self.__item.notifyKinematicStateChange()
            if updateGui:
                cnoid.Base.App.updateGui()

    def setDefaultPose(self):
        """Setting default pose if registered
        """
        self.setNamedPose('default')

    def setInitialPose(self):
        """Setting default pose if registered
        """
        self.setNamedPose('initial')

    def setNamedPose(self, name):
        """Setting named pose, name should be registered by irsl_choreonoid.robot_util.RobotModelWrapped.registerNamedPose

        Args:
            name (str) : Name of registered pose

        """
        if name in self.pose_angle_map:
            self.angleVector(self.pose_angle_map[name])
        if name in self.pose_coords_map:
            self.rootCoords(self.pose_coords_map[name])

    def getLimb(self, limb_name):
        """Getting limb as end-effector, name should be registered by irsl_choreonoid.robot_util.RobotModelWrapped.registerEndEffector

        Args:
            limb_name (str) : Name of registered limb as end-effector

        Returns:
            irsl_choreonoid.robot_util.RobotModelWrapped.EndEffector : Instance of EndEffector

        Raises:
            Exeption : If wrong limb name is passed

        """
        if limb_name in self.eef_map:
            return self.eef_map[limb_name]
        elif limb_name == 'default':
            ret = None
            for v in self.eef_map.values():
                ret = v
                break
            return ret
        else:
            raise Exeption('unknown limb name{}'.format(limb_name))

    def getEndEffector(self, limb_name):
        """Getting coordinates of end-effector of limb (return value is generated on demand)

        Args:
            limb_name (str) : Name of registered limb as end-effector

        Returns:
            cnoid.IRSLCoords.coordinates : Current coordinates of end-effector of limb

        """
        return self.getLimb(limb_name).EndEffector

    @property
    def rleg(self):
        """Pre defined accessor of limb, same as self.getLimb('rleg')

        Returns:
            irsl_choreonoid.robot_util.RobotModelWrapped.EndEffector : Instance of EndEffector (rleg)

        """
        return self.getLimb('rleg')
    @property
    def lleg(self):
        """Pre defined accessor of limb, same as self.getLimb('lleg')

        Returns:
            irsl_choreonoid.robot_util.RobotModelWrapped.EndEffector : Instance of EndEffector (lleg)

        """
        return self.getLimb('lleg')
    @property
    def rarm(self):
        """Pre defined accessor of limb, same as self.getLimb('rarm')

        Returns:
            irsl_choreonoid.robot_util.RobotModelWrapped.EndEffector : Instance of EndEffector (rarm)

        """
        return self.getLimb('rarm')
    @property
    def larm(self):
        """Pre defined accessor of limb, same as self.getLimb('larm')

        Returns:
            irsl_choreonoid.robot_util.RobotModelWrapped.EndEffector : Instance of EndEffector (larm)

        """
        return self.getLimb('larm')
    @property
    def arm(self):
        """Pre defined accessor of limb, same as self.getLimb('arm')

        Returns:
            irsl_choreonoid.robot_util.RobotModelWrapped.EndEffector : Instance of EndEffector (arm)

        """
        return self.getLimb('arm')
    @property
    def head(self):
        """Pre defined accessor of limb, same as self.getLimb('head')

        Returns:
            irsl_choreonoid.robot_util.RobotModelWrapped.EndEffector : Instance of EndEffector (head)

        """
        return self.getLimb('head')
    @property
    def torso(self):
        """Pre defined accessor of limb, same as self.getLimb('torso')

        Returns:
            irsl_choreonoid.robot_util.RobotModelWrapped.EndEffector : Instance of EndEffector (torso)

        """
        return self.getLimb('torso')

    @property
    def rlegEndEffector(self):
        """Pre defined accessor to end-effector, same as self.getLimb('rleg').endEffector

        Returns:
            cnoid.IRSLCoords.coordinates : Current coordinates of end-effector of limb(rleg)

        """
        return self.getLimb('rleg').endEffector
    @property
    def llegEndEffector(self):
        """Pre defined accessor to end-effector, same as self.getLimb('lleg').endEffector

        Returns:
            cnoid.IRSLCoords.coordinates : Current coordinates of end-effector of limb(lleg)

        """
        return self.getLimb('lleg').endEffector
    @property
    def rarmEndEffector(self):
        """Pre defined accessor to end-effector, same as self.getLimb('rarm').endEffector

        Returns:
            cnoid.IRSLCoords.coordinates : Current coordinates of end-effector of limb(rarm)

        """
        return self.getLimb('rarm').endEffector
    @property
    def larmEndEffector(self):
        """Pre defined accessor to end-effector, same as self.getLimb('larm').endEffector

        Returns:
            cnoid.IRSLCoords.coordinates : Current coordinates of end-effector of limb(larm)

        """
        return self.getLimb('larm').EndEffector
    @property
    def armEndEffector(self):
        """Pre defined accessor to end-effector, same as self.getLimb('arm').endEffector

        Returns:
            cnoid.IRSLCoords.coordinates : Current coordinates of end-effector of limb(arm)

        """
        return self.getLimb('arm').endEffector
    @property
    def headEndEffector(self):
        """Pre defined accessor to end-effector, same as self.getLimb('head').endEffector

        Returns:
            cnoid.IRSLCoords.coordinates : Current coordinates of end-effector of limb(head)

        """
        return self.getLimb('head').endEffector
    @property
    def torsoEndEffector(self):
        """Pre defined accessor to end-effector, same as self.getLimb('torso').endEffector

        Returns:
            cnoid.IRSLCoords.coordinates : Current coordinates of end-effector of limb(torso)

        """
        return self.getLimb('torso').endEffector

    def inverseKinematics(self, coords, limb_name='default', **kwargs):
        """Solving inverse kinematic of the limb

        Args:
            coords (cnoid.IRSLCoords.coordinates) : Coordinate of the target
            limb_name (str, default='default') : Name of the limb
            \*\*kwargs ( dict ) : Just passing to IKWrapper

        Returns:
             (boolean, int) : IK was success or not, and total count of calculation

        """
        return self.getLimb(limb_name).inverseKinematics(coords, **kwargs)

    def footMidCoords(self, p = 0.5):
        cds = self.rlegEndEffector
        return cds.mid_coords(p, self.llegEndEffector)

    def fixLegToCoords(self, coords, p = 0.5):
        mc = self.foot_mid_coords(p)
        cds = mc.inverse_transformation()
        cds.transform(coords)
        cds.transform(self.__robot.rootLink.getCoords())
        self.__robot.rootLink.setCoords(cds)
        self.hook()

    def keepJointLimit(self, on_ = True):
        """Setting mode to keep joint limits

        Args:
            on_ (boolean, default=True) : 

        Returns:
            (boolean) : Returns current settings

        """
        if on_ is not None:
            self.__keep_limit = on_
        return self.__keep_limit

    def trimJointAngles(self):
        """Force setting joint angles inside the range of limits

        """
        for j in self.__robot.joints:
            q_ = j.q
            if q_ > j.q_upper:
                j.q = j.q_upper
            elif q_ < j.q_lower:
                j.q = j.q_lower

    def calcMinimumDuration(self, target_angle_vector, original_angle_vector=None, ratio=1.0):
        """Calculating duration for the fastest movement depend on a joint limit

        Args:
            target_angle_vector (numpy.array) : AngleVector to be moved to
            original_angle_vector (numpy.array, optional) : AngleVector to be moved from. If this argument is not given, current angles will be used.
            ratio (float) : Result will be multipled by this value

        Returns:
            float : Duration [ second ]

        """
        if original_angle_vector is None:
            original_angle_vector = self.angleVector()
        ##
        result = 0.0
        for j, ang0, ang1 in zip(self.__robot.joints, original_angle_vector, target_angle_vector):
            if j.dq_upper > 1e6 and j.dq_lower < -1e6:
                min_d = 0.0
            else:
                diff_angle = ang1 - ang0
                if diff_angle > 0:
                    min_d = diff_angle / j.dq_upper
                else:
                    min_d = diff_angle / j.dq_lower
            if min_d > result:
                result = min_d
        return ratio * result

    def fullbodyInverseKinematics(self, **kwargs):
        ### not implemented yet
        pass
    def moveCentroidOnFoot(self, p = 0.5, debug = False):
        ### not implemented yet
        pass
    ## wrappedMethod to cnoid.Body
    @property
    def mass(self):
        return self.__robot.mass
    @property
    def centerOfMass(self):
        self.__robot.calcCenterOfMass()
        return self.__robot.centerOfMass
    @property
    def numJoints(self):
        return self.__robot.numJoints
    @property
    def numVirtualJoints(self):
        return self.__robot.numVirtualJoints
    @property
    def numAllJoints(self):
        return self.__robot.numAllJoints
    @property
    def numLinks(self):
        return self.__robot.numLinks
    @property
    def numDevices(self):
        return self.__robot.numDevices

### flush in Base, etc.
if isInChoreonoid():
    from .cnoid_base import *
    import cnoid.Base
