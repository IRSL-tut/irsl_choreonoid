import cnoid.Body
import cnoid.Util
import cnoid.BodyPlugin as BodyPlugin

from .cnoid_util import *

import cnoid.IRSLCoords as ic
try:
    import cnoid.IKSolvers as IK
except ImportError:
    pass
import numpy as np
import random
import math

def make_coordinates(coords_map, scale=1.0):
    """Generating coordinates(cnoid.IRSLCoords.coordinates) from dictionary

    Args:
        coords_map (dict[str, list[float]]) : dictionary of describing transformation
        scale (float, default = 1.0) : Scaling for making coordinates

    Returns:
        cnoid.IRSLCoords.coordinates : generated coordinates

    Raises:
        Exception : If there is not valid keyword

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
            pos = np.array(coords_map[key]) * scale
            break
    for key in ('q', 'quaternion'):
        if key in coords_map:
            q = np.array(coords_map[key])
            if pos is None:
                return ic.coordinates(q)
            else:
                return ic.coordinates(pos, q)
    for key in ('angle-axis', 'angle_axis', 'aa', 'rotation'):
        if key in coords_map:
            aa = coords_map[key]
            rot = ic.angleAxisNormalized(aa[3], np.array(aa[:3]))
            if pos is None:
                return ic.coordinates(rot)
            else:
                return ic.coordinates(pos, rot)
    for key in ('rotation-matrix', 'matrix', 'mat', 'rot'):
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

def make_coords_map(coords, method=None, scale=1.0):
    """Generating dictonary describing transformation

    Args:
        coords (cnoid.IRSLCoords.coordinates) : Transformation
        scale (float, default = 1.0) : Scaling for making dictionary

    Returns:
        dict[str, list[float]] : Dictonary can be used by make_coordinates

    Examples:
        >>> make_coords_map( make_coordinates( {'position' : [1, 2, 3]} ) )
        {'pos': [1.0, 2.0, 3.0], 'aa': [1.0, 0.0, 0.0, 0.0]}
        >>> make_coords_map(make_coordinates( {'pos' : [0.01, 0.02, 0.03]} ), method = 'translation', scale=1000.0)
        {'translation': [10.0, 20.0, 30.0], 'rotation': [1.0, 0.0, 0.0, 0.0]}
    """
    pos = coords.pos * scale
    if method is None:
        return {'pos': pos.tolist(), 'aa': coords.getRotationAngle().tolist()}
    elif method in ('RPY', 'rpy'):
        return {'pos': pos.tolist(), method: coords.getRPY().tolist()}
    elif method in ('q', 'quaternion'):
        return {'pos': pos.tolist(), method: coords.quaternion.tolist()}
    elif method in ('rotation'):
        return {'translation': pos.tolist(), method: coords.getRotationAngle().tolist()}
    elif method in ('aa', 'angle_axis', 'angle-axis'):
        return {'pos': pos.tolist(), method: coords.getRotationAngle().tolist()}
    elif method in ('rotation-matrix', 'matrix', 'mat', 'rot'):
        return {'pos': pos.tolist(), method: coords.rot.tolist()}
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

def axisAlignedCoords(axis, target_axis=ic.coordinates.Y, rotate=None, up_axis=None):
    """Generating axis aligned coordinates.

    target_axis on the generated coordinates will be axis on the world-coordinates

    Args:
        axis ( list[float] ) : Target direction(axis)
        target_axis ( list[float] ) : Target axis to be alined

    Returns:
        cnoid.IRSLCoords.coordinates : generated coordinates

    Examples:
        >>> ay = axisAlignedCoords([1., 1., 1.,], coordinates.Y)
        >>> ay.y_axis
        array([0.57735027, 0.57735027, 0.57735027])

        >>> az = axisAlignedCoords([1., 1., 1.,], coordinates.Z)
        >>> az.z_axis
        array([0.57735027, 0.57735027, 0.57735027])

        >>> target = fv(1., 1., 1.)
        >>> aa = axisAlignedCoords([1., 0., 1.,], target)
        >>> aa.transform_vector(target)
        array([1.22474487, 0.        , 1.22474487])
    """
    ax = np.array(axis)
    ic.coordinates.normalizeVector(ax)
    rota = np.cross(target_axis, ax)
    angle2 = math.atan2(np.linalg.norm(rota), np.dot(ax, target_axis))
    res = ic.coordinates()
    ic.coordinates.normalizeVector(rota)
    res.rotate(angle2, rota)
    if rotate is not None:
        res.rotate(rotate, target_axis)
    if up_axis is not None:
        ## up: ic.coordinates.Z
        ## res.rotate_vector(up_axis)
        pass ## not implemented yet
    return res

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
                              base_type = None, base_weight = 1.0, max_iteration = 32, threshold = 5e-5, position_precision = None,
                              use_joint_limit=True, joint_limit_max_error=1e-2, joint_limit_precision=0.1, **kwargs):
        ## default position precision // 1e-4, 1e-4, 1e-4, 0.001745, 0.001745, 0.001745;
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
        if position_precision is not None:
            a_constraint.precision = np.array(position_precision)
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
            if position_precision is not None:
                b_constraint.precision = np.array(position_precision)
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
                const.precision = joint_limit_precision
                const.maxError  = joint_limit_max_error
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

### functions for Body
def linkDirectChildren(lk):
    """
    """
    ch = lk.child
    if ch is None:
        return []
    else:
        child_list = [ch]
        while ch.sibling is not None:
            ch = ch.sibling
            child_list.append(ch)
        return child_list

def linkDescendants(lk):
    """
    """
    ch = lk.child
    if ch is None:
        return []
    else:
        child_list = [ch]
        while ch.sibling is not None:
            ch = ch.sibling
            child_list.append(ch)
        for ch in tuple(child_list):
            res = linkDescendants(ch)
            child_list.extend(res)
        return child_list

def mergedMassPropertyOfAllDescendants(plink):
    """Merge mass-property of all descendants of parent link

    Args:
        plink ( cnoid.Link ) : parent link
    Returns:
        (float, numpy.array, numpy.array) : (mass, center of mass[at parent link coordinates], inertia tensor[around center of mass])
    """
    plink_mass = plink.mass
    plink_c = plink.c
    plink_I = plink.I
    plink_coords = coordinates(plink.T)
    for clink in linkDescendants(plink):
        plink_mass, plink_c, plink_I = _mergeMassProperty(plink_coords, plink_mass, plink_c, plink_I, clink)
    return plink_mass, plink_c, plink_I

def mergedMassPropertyOfList(linkList):
    """Merge mass-property of list of link

    Args:
        linkList ( list[cnoid.Link] ) : List of link, the first element is treated as the parent link
    Returns:
        (coordinates, float, numpy.array, numpy.array) : (base_coordinate, mass, center of mass[at base_coordinates], inertia tensor[around center of mass])
    """
    plink = linkList[0]
    plink_mass = plink.mass
    plink_c = plink.c
    plink_I = plink.I
    plink_coords = coordinates(plink.T)
    for clink in linkList[1:]:
        plink_mass, plink_c, plink_I = _mergeMassProperty(plink_coords, plink_mass, plink_c, plink_I, clink)
    return plink_coords, plink_mass, plink_c, plink_I

def _mergeMassProperty(plink_coords, plink_mass, plink_c, plink_I, clink):
    new_mass = plink_mass + clink.mass
    ##
    cds_Tb = plink_coords.transformation( coordinates(clink.T) )
    #inv_p = plink_coords.inverse_transformation()
    #cds_Tb = inv_p.transform( coordinates(clink.T) )
    p_c_c = np.copy(clink.c)
    cds_Tb.transformVector(p_c_c)
    if new_mass > 0:
        new_c = ((clink.mass * p_c_c) + (plink_mass * plink_c))/new_mass
    else:
        new_c = np.zeros(3)
    ##
    pIc = cds_Tb.rot @ clink.I @ np.transpose(cds_Tb.rot)
    h_c = hat(new_c - p_c_c)
    h_p = hat(new_c - plink_c)
    ##
    new_I = (pIc - clink.mass * (h_c @ h_c)) + (plink_I - plink_mass * (h_p @ h_p))
    ##
    return (new_mass, new_c, new_I)

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
cnoid.Body.Link.directChildren = lambda self: linkDirectChildren(self)
cnoid.Body.Link.descendants = lambda self: linkDescendants(self)

def merge_mask(tp1, tp2):
    return tuple([x or y for (x, y) in zip(tp1, tp2)])
def invert_mask(tp1):
    return tuple([0 if x else 1 for x in tp1])

from .irsl_draw_object import coordsWrapper
class RobotModelWrapped(coordsWrapper): ## with wrapper
    """RobotModel for programming Interactively

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
            self.__rename_map_inv = {}
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
            for k, v in _rmap.items():
                self.__rename_map_inv[v] = k

        def nicknameOf(self, joint_name):
            """
            """
            if self.__rename_map_inv is None:
                return joint_name
            if joint_name in self.__rename_map_inv:
                return self.__rename_map_inv[joint_name]
            else:
                return joint_name

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

        def joint(self, jname):
            """Getting joint in this limb by name

            Args:
                jname (str) : name or nick-name of a joint

            Returns:
                cnoid.Body.Link : Instance of the joint

            """
            nm = self.rename(jname)
            if nm in self.__joint_map:
                return self.__joint_map[nm]

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
        def jointNicknames(self):
            return [ self.nicknameOf(j.jointName) for j in self.__joint_list ]

        @property
        def renameMap(self):
            return self.__rename_map

        @property
        def tipLink(self):
            return self.__tip_link

        @property
        def tipLinkToEEF(self):
            return self.__tip_link_to_eef

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

        def move(self, vec, wrt=ic.coordinates.wrt.local, **kwargs):
            """Relative moving end-effector linearly by translation vector

            Args:
                vec (numpy.array) : Translation vector
                wrt (default = cnoid.IRSLCoords.coordinates.wrt.local) : Just passing to cnoid.IRSLCoords.coordinates.translate
                kwargs (dict) : Just passing to IKWrapper

            Returns:
                 (boolean, int) : IK was success or not, and total count of calculation

            """
            tgt=self.endEffector
            tgt.translate(vec, wrt)
            return self.inverseKinematics(tgt, **kwargs)

        def rotate(self, angle, axis, wrt=ic.coordinates.wrt.local, **kwargs):
            """Relative moving end-effector angularly by angle-axis

            Args:
                angle (float) : Rotation angle
                axis (numpy.array) : Rotation axis
                wrt (default = cnoid.IRSLCoords.coordinates.wrt.local) : Just passing to cnoid.IRSLCoords.coordinates.rotate
                kwargs (dict) : Just passing to IKWrapper

            Returns:
                 (boolean, int) : IK was success or not, and total count of calculation

            """
            tgt=self.endEffector
            tgt.rotate(angle, axis, wrt)
            return self.inverseKinematics(tgt, **kwargs)

        def moveCoords(self, coords, wrt=ic.coordinates.wrt.local, **kwargs):
            """Relative moving end-effector by transformation of coordinates

            Args:
                coords (cnoid.IRSLCoords.coordinates) : Relative coordinates
                wrt (default = cnoid.IRSLCoords.coordinates.wrt.local) : Just passing to cnoid.IRSLCoords.coordinates.transform
                kwargs (dict) : Just passing to IKWrapper

            Returns:
                 (boolean, int) : IK was success or not, and total count of calculation

            """
            tgt=self.endEffector
            tgt.transform(coords, wrt)
            return self.inverseKinematics(tgt, **kwargs)

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

        def getAngleMap(self, *args):
            """Getting angles of the joint (limb version)

            Args:
                name_list ( list[str] ) : List of joint names. 'nickname' is acceptable.

            Returns:
                dict[str, float] : Keyword is a joint name and value is a joint angle.

            """
            if len(args) == 0 or self.__joint_list is None:
                ### warning
                return {}
            if type(args[0]) is str:
                name_list = args
            elif hasattr(args[0], '__iter__'):
                name_list = args[0]
            else:
                ### warning
                return {}
            res = {}
            for name in name_list:
                nm = self.rename(name)
                if nm in self.__joint_map:
                    res[nm] = self.__joint_map[nm].q
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
    def item(self):
        """Robot-model using wrapped by this instance

        Returns:
            cnoid.BodyPlugin.BodyItem : Instance of robot-item currently using

        """
        return self.__item

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
        p_cds.transform(ic.coordinates(dev.T_local))
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

    def getAngleMap(self, *args):
        """Getting angles of the joint

        Args:
            name_list ( list[str] ) : List of joint names. 'nickname' is acceptable.

        Returns:
            dict[str, float] : Keyword is a joint name and value is a joint angle.

        """
        if len(args) == 0 or self.__joint_list is None:
            ### warning
            return {}
        if type(args[0]) is str:
            name_list = args
        elif hasattr(args[0], '__iter__'):
            name_list = args[0]
        else:
            ### warning
            return {}
        res = {}
        for name in name_list:
            if name in self.__joint_map:
                res[name] = self.__joint_map[name].q
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
        else:
            self.revert()
        return self.copy()

    def hook(self):
        if self.__keep_limit:
            self.trimJointAngles()
        if self.__mode < 0:
            ### do nothing(no-hook)
            return
        if self.__mode == 0: ## kinematics only
            self.__robot.calcForwardKinematics()
            self.revert()## sync Body -> coordsWrapper
        elif self.__mode == 1: ## render
            self.flush()
        elif self.__mode == 2: ## render Immediately
            self.flush(True)

    def flush(self, updateGui=False):
        if self.__robot is not None:
            self.__robot.calcForwardKinematics()
            self.revert()## sync Body -> coordsWrapper
        if self.__item is not None:
            self.updateTarget(updateGui) ##
            self.__item.notifyKinematicStateChange()
            if updateGui:
                cnoid.Base.App.updateGui()

    def setDefaultPose(self):
        """Setting default pose if registered
        """
        self.setNamedPose('default')

    def setInitialPose(self):
        """Setting initial pose if registered
        """
        self.setNamedPose('initial')

    def setZeroPose(self):
        """Setting all joint angles to zero
        """
        self.angleVector(np.zeros(self.numJoints))

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
            Exception : If wrong limb name is passed

        """
        if limb_name in self.eef_map:
            return self.eef_map[limb_name]
        elif limb_name == 'default':
            ret = None
            for v in self.eef_map.values():
                ret = v
                break
            return ret
        elif limb_name == 'base':
            return self.robot.rootLink
        else:
            raise Exception('unknown limb name: {}'.format(limb_name))

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
        """Getting coordinates representing mid-point of leg's end-effector

        Args:
            p (float, default=0.5) : Parameter of mid-point. (0.5, 0.0, 1.0 represents the center, rleg, lleg, respectively)

        Returns:
            cnoid.IRSLCoords.coordinates : Coordinates indicating the mid-point of legs

        Note:
            This method requires settings of limbs( 'rleg' and 'lleg' )


        """
        cds = self.rlegEndEffector
        return cds.mid_coords(p, self.llegEndEffector)

    def fixLegToCoords(self, coords, p = 0.5):
        """Locating the robot by fixing the legs to designated coordinates

        Args:
            coords (cnoid.IRSLCoords.coordinates) : Target coordinates ( footMidCoords of this robot will be the same as this coordinates )
            p (float, default=0.5) : Parameter of mid-point. (0.5, 0.0, 1.0 represents the center, rleg, lleg, respectively)

        Note:
            This method requires settings of limbs( 'rleg' and 'lleg' )


        """
        mc = self.footMidCoords(p)
        self.fixSelfCoordsToTarget(mc, coords)

    def fixSelfCoordsToTarget(self, selfcoords, target_coords):
        """
        """
        cds = selfcoords.inverse_transformation()
        cds.transform(self.__robot.rootLink.getCoords())
        cds.transform(target_coords, wrt=ic.coordinates.wrt.world)
        self.newcoords(cds)
        #self.__robot.rootLink.setCoords(cds)
        #self.hook()

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
##>    @staticmethod
##>    def parseConstraint(const):
##>        if const is None or const == '6D':
##>            constraint = [1, 1, 1, 1, 1, 1]
##>        elif const == 'position':
##>            constraint = [1, 1, 1, 0, 0, 0]
##>        elif const == 'rotation':
##>            constraint = [0, 0, 0, 1, 1, 1]
##>        elif type(const) is str:
##>            ## 'xyzRPY'
##>            constraint = [0, 0, 0, 0, 0, 0]
##>            for ss in const:
##>                if ss == 'x':
##>                    constraint[0] = 1
##>                elif ss == 'y':
##>                    constraint[1] = 1
##>                elif ss == 'z':
##>                    constraint[2] = 1
##>                elif ss == 'R':
##>                    constraint[3] = 1
##>                elif ss == 'P':
##>                    constraint[4] = 1
##>                elif ss == 'Y':
##>                    constraint[5] = 1
##>        else:
##>            return const
##>        return constraint
    ##
    def fullBodyIK(self, targets, limbs, com_target=None, base_target=None, revert_if_failed=True, retry=10, **kwargs):
        """Top method to solve fullbody inverse-kinematics

        Args:
            targets (cnoid.IRSLCoords.coordinates) : List of coordinates as target for inverse-kinematics
            limbs (irsl_choreonoid.robot_util.RobotModelWrapped.EndEffector) : List of limb(EndEffector), size of limbs should be equal to size of targets
            com_target (numpy.array) : Target of COM ( Center of Mass )
            base_target (cnoid.IRSLCoords.coordinates) : Target of base (root_link)
            revert_if_failed (boolean, default = True) : if True, no modification (moving joint and root-link) to robot-model when calculation of IK is failed
            retry (int, defualt = 100) : number of retrying solving IK
            \*\*kwargs (dict) : Other keywords just passing to fullBodyIKRaw (See irsl_choreonoid.robot_util.RobotModelWrapped.fullBodyIKRaw)

        Returns:
             (boolean, int) : IK was success or not, and total count of calculation

        Note:
              constraint : [1,1,1, 1,1,1]
              base_type : [1,1,1, 1,1,1]

        """
        init_angle = self.angleVector()
        init_coords = self.rootCoords()
        if base_target is None:
            base_target = init_coords
        succ = False
        _total = 0
        while retry >= 0:
            succ, _iter = self.fullBodyIKRaw(targets, limbs, com_target=com_target, base_target=base_target, **kwargs)
            _total += _iter
            if succ:
                break
            retry -= 1
        if (not succ) and revert_if_failed:
            self.angleVector(init_angle)
            self.rootCoords(init_coords)
        self.hook()
        return (succ, _total)
    ##
    def fullBodyIKRaw(self, targets, limbs, com_target=None, base_target=None,
                   constraint='6D', base_type='xy', com_constraint = [1, 1, 0], weight = 1.0, base_weight = 1.0,
                   debug = False, max_iteration = 32, threshold = 5e-5, use_joint_limit=True, joint_limit_max_error=1e-2,
                   position_precision = None, com_precision = None, joint_limit_precision=0.1, **kwargs):
        """
        """
        if len(targets) != len(limbs):
            raise Exception('{len(targts) is not euqal to len(limbs), {} != {}}'.format(len(targets), len(limbs)))
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
        ## base_type (invert constraint)
        if base_type == '2D' or base_type == 'planer':
            base_const = np.array([0, 0, 1, 1, 1, 0]) ## move x, y, Yaw
        elif base_type == '3D-yaw':
            base_const = np.array([0, 0, 0, 1, 1, 0]) ## move x, y, z, Yaw
        elif base_type == 'position':
            base_const = np.array([0, 0, 0, 1, 1, 1]) ## move x, y, z
        elif base_type == 'rotation':
            base_const = np.array([1, 1, 1, 0, 0, 0]) ## move Roll, Pitch, Yaw
        elif base_type == 'free':
            base_const = np.array([0, 0, 0, 0, 0, 0]) ## free
        elif type(base_type) is str:
            ## 'xyzRPY'
            wstr = base_type
            _bweight = [1, 1, 1, 1, 1, 1]
            for ss in wstr:
                if ss == 'x':
                    _bweight[0] = 0
                elif ss == 'y':
                    _bweight[1] = 0
                elif ss == 'z':
                    _bweight[2] = 0
                elif ss == 'R':
                    _bweight[3] = 0
                elif ss == 'P':
                    _bweight[4] = 0
                elif ss == 'Y':
                    _bweight[5] = 0
            base_const = np.array(_bweight)
        elif base_type is not None:
            base_const = np.array(base_type)

        ### constraints for IK(limbs)
        constraints0 = IK.Constraints()
        for target, limb in zip(targets, limbs):
            tmp_constraint = IK.PositionConstraint()
            tmp_constraint.A_link =     limb.tipLink
            tmp_constraint.A_localpos = limb.tipLinkToEEF.toPosition()
            #constraint.B_link() = nullptr;
            tmp_constraint.B_localpos = target.toPosition()
            tmp_constraint.weight     = weight * np.array(constraint)
            if position_precision is not None:
                tmp_constraint.precision = np.array(position_precision)
            constraints0.push_back(tmp_constraint)
        ### constraints for base
        if base_type is not None:
            base_const = base_weight * base_const
            if debug:
                print('use base : {}'.format(base_const))
            b_constraint = IK.PositionConstraint()
            b_constraint.A_link =     self.robot.rootLink
            b_constraint.A_localpos = ic.coordinates().cnoidPosition
            #constraint.B_link() = nullptr;
            if base_target is None:
                b_constraint.B_localpos = self.robot.rootLink.T
            else:
                b_constraint.B_localpos = base_target.cnoidPosition
            b_constraint.weight     = np.array(base_const)
            if position_precision is not None:
                b_constraint.precision = np.array(position_precision)
            constraints0.push_back(b_constraint)
        ### COM constraint
        if com_target is not None:
            _com_constraint = IK.COMConstraint()
            _com_constraint.A_robot = self.robot
            _com_constraint.B_localp = com_target
            _com_constraint.weight   = np.array(com_constraint)
            if com_precision is not None:
                _com_constraint.precision = np.array(com_precision)
            constraints0.push_back(_com_constraint)
        #
        tasks = IK.Tasks()
        dummy_const = IK.Constraints()
        constraints = [ dummy_const, constraints0 ]
        jlist = []
        for limb in limbs:
            jlist += limb.jointList
        ### constraint joint-limit
        if use_joint_limit:
            constraints1 = IK.Constraints()
            for j in jlist:
                const = IK.JointLimitConstraint()
                const.joint = j
                const.precision = joint_limit_precision
                const.maxError  = joint_limit_max_error
                constraints1.push_back(const)
            constraints.append(constraints1)
        #
        variables = []
        if base_type is not None:
            variables.append(self.robot.rootLink)
        variables += jlist
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
    ##
    def legsCOM_IK(self, r_target, l_target, com_target=None, **kwargs):
        """
        """
        rleg = self.getLimb('rleg')
        lleg = self.getLimb('lleg')
        return self.fullBodyIK( (r_target, l_target), (rleg, lleg),
                                com_target=com_target, **kwargs)
    def armsCOM_IK(self, r_target, l_target, com_target=None, **kwargs):
        """
        """
        rarm = self.getLimb('rarm')
        larm = self.getLimb('larm')
        return self.fullBodyIK( (r_target, l_target), (rarm, larm),
                                com_target=com_target, **kwargs)
    def moveCentroidOnFoot(self, p = 0.5, com_height=None, **kwargs):
        """Move projected centroid to the center of feet

        Args:
            p (float, default = 0.5) : Parameter of center position (p == 0.5 means center, p == 0.0 means right-foot, p == 1.0 means left-foot)
            com_height(float, optional) : Set target of height of CenterOfMass
        """
        rleg = self.getLimb('rleg')
        lleg = self.getLimb('lleg')
        r_cds = rleg.endEffector
        l_cds = lleg.endEffector
        foot_mid = r_cds.mid_coords(p, l_cds)
        ##foot_mid = self.footMidCoords(p)
        com = foot_mid.pos
        if com_height is not None:
            com[2] = com_height
            if not 'com_constraint' in kwargs:
                kwargs['com_constraint'] = [1, 1, 1]
            if not 'base_type' in kwargs:
                kwargs['base_type'] = 'xyz'
        return self.fullBodyIK( (r_cds, l_cds), (rleg, lleg),
                                com_target=com, **kwargs)
    ##
    def addLink(self, name, parentLink, offset, jointType=Link.JointType.FixedJoint, visualShape=None, collisionShape=None, **kwargs):
        """Adding new link

        Args:
            name (str) :
            parentLink ( cnoid.Body.Link ) :
            offset ( cnoid.IRSLCoords.coordinates) :
            jointType () :
            visualShape () :
            collisionShape () :
            kwargs () :

        Note:
            keywords for kwargs are listed below.
            JointName, Mass, CenterOfMass, Inertia, EquivalentRotorInertia,
            JointId, JointAxis, JointVelocityRange, JointRange, JointEffortRange,
            InitialJointDisplacement, ActuationMode

        """
        plk = self.link(parentLink) if type(parentLink) is str else parentLink

        lk = self.robot.createLink()
        lk.setName(name)
        lk.setJointType(jointType)
        for k, v in kwargs.items():
            exec(f'lk.set{k}( v )')
        if visualShape is not None:
            lk.addVisualShape(visualShape)
        if collisionShape is not None:
            lk.addCollisionShape(collisionShape)
        #pcds = coordinates(plk.T)
        #pcds.transform(offset)
        lk.setOffsetPosition(offset.cnoidPosition)
        plk.appendChild(lk)
        self.robot.updateLinkTree()
        self.robot.calcForwardKinematics()
        self._updateBodyStructure()
    def _updateBodyStructure(self):
        if self.__item is not None:
            ### for updating item while changing structure of body
            self.__item.notifyModelUpdate(sum([int(i) for i in (BodyPlugin.BodyItem.ModelUpdateFlag.LinkSetUpdate, BodyPlugin.BodyItem.ModelUpdateFlag.LinkSpecUpdate,
                                                                BodyPlugin.BodyItem.ModelUpdateFlag.DeviceSetUpdate, BodyPlugin.BodyItem.ModelUpdateFlag.DeviceSpecUpdate,
                                                                BodyPlugin.BodyItem.ModelUpdateFlag.ShapeUpdate)]))
            self.__item.notifyKinematicStateUpdate()

    def setFrame(self, name, parentLink, offset):
        """Setting temporary frame (link), which is connected by fixed-joint

        Args:
            name (str) : Name of the adding link
            parentLink ( str or cnoid.Body.Link ) : Link name or instance of link which the adding link is connected to
            offset ( cnoid.IRSLCoords.coordinates ) : Offset from parentLink to the adding link ( on parentLink coordinates )

        """
        self.addLink(name, parentLink, offset,
                     Mass=0.0, Inertia=np.array([[0, 0., 0.], [0., 0, 0.], [0., 0., 0]]) )

    ##
    def generateCurrentVisual(self, scalable=False, init_coords=None):
        """Generate visual (scene graph) of robot with current state

        Args:
            scalable (boolean, default=False) : Enable methods, scale and switch
            init_coords (cnoid.IRSLCoords.coordinates, optional) : Coordinates of root-coords of generated visual

        Returns:
            ( irsl_draw_object.coordsWrapper ) : Wrapped SceneGrap (can visualize with DrawInterface)
        """
        root_coords = coordinates(self.robot.rootLink.T)
        allgrp = cnoid.Util.SgPosTransform()
        for l in self.linkList:
            cds = root_coords.transformation(coordinates(l.T))
            trs = cnoid.Util.SgPosTransform()
            trs.T = cds.cnoidPosition
            trs.addChild(cnoid.Util.SgGroup(l.getVisualShape()))
            allgrp.addChild(trs)
        if init_coords is None:
            init_coords = root_coords
        return coordsWrapper(allgrp, init_coords=init_coords, scalable=scalable)

    ## wrappedMethod to cnoid.Body
    def joint(self, name):
        return self.__robot.joint(name)
    def link(self, name):
        return self.__robot.link(name)
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
    @classmethod
    def loadModel(cls, fname, **kwargs):
        rb=loadRobot(fname, **kwargs)
        return cls(rb, **kwargs)
    @classmethod
    def loadModelItem(cls, fname, **kwargs):
        rb=loadRobotItem(fname, **kwargs)
        return cls(rb, **kwargs)

class ImportedRobotModel(RobotModelWrapped):
    """Class for making specific robot model

    1. Set model_cls.model_file in the module
    2. model_module.robot_class should be set

    Examples:
        >>> ### SpecificRobotClass.py
        >>> class SpecificRobotClass(ImportedRobotModel):
        >>>     def __init__(self, robot=None, item=True, world=False, **kwargs):
        >>>         super().__init__(robot=robot, item=item, world=world, **kwargs)
        >>> 
        >>>     def _setting_before_init(self, robot, **kwargs):  ## override
        >>>         pass
        >>> 
        >>>     def _init_ending(self, **kwargs): ## override
        >>>         pass
        >>> SpecificRobotClass.model_file = 'path_to_model_file' ## 1. ##
        >>> robot_class = SpecificRobotClass ## 2. ##

    """
    model_file = None
    ##
    def __init__(self, robot=None, item=True, world=False, **kwargs):
        """
        Args:
            robot ( cnoid.Body.Body or cnoid.BodyPlugin.BodyItem ) :
            item (boolean, default=True) : If true, loading as BodyItem
            world (boolean, default=True) : If true, BodyItem will be loaded under WorldItem
        """
        if robot is None:
            robot = self._loadRobotModel(item=item, world=world)
        ##
        self._setting_before_init(robot, **kwargs)
        super().__init__(robot, **kwargs)
        self._init_ending(**kwargs)

    def _setting_before_init(self, robot, **kwargs):
        ### override at inherited class
        pass

    def _init_ending(self, **kwargs):
        ### override at inherited class
        pass

    def _loadRobotModel(self, item=True, world=False):
        if isInChoreonoid() and item:
            res = loadRobotItem(self.model_file, world=world)
        else:
            res = loadRobot(self.model_file)
        return res

#    @classmethod
#    def makeRobot(cls, robot=None, item=True, world=True, **kwargs):
#        print(cls)
#        return cls(robot=robot, item=item, world=world, **kwargs)

### flush in Base, etc.
if isInChoreonoid():
    from .cnoid_base import *
    import cnoid.Base
