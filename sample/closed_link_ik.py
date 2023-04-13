# exec(open('closed_link_ik.py').read()) in choreonoid Python Console
# OR
# PYTHONPATH=$PYTHONPATH:$(dirname $(which choreonoid))/../lib/choreonoid-1.8/python python3 closed_link_ik.py

import cnoid.IRSLUtil as iu
import irsl_choreonoid.robot_util as ru
import numpy as np
import math
import cnoid.IKSolvers as IK

import cnoid.Util

fname = cnoid.Util.getShareDirectory() + '/../irsl_choreonoid/sample/closed_link_ik.body'

if ru.isInChoreonoid():
    rb = ru.loadRobotItem(fname)
else:
    rb = ru.loadRobot(fname)

rr = ru.RobotModel(rb)
rr.flush()

doClosedIK = True

def solveClosedIK(targetq = None, joint_id = 0, flush = True):
    global rr
    global doClosedIK
    if not doClosedIK:
        doClosedIK = True
        return
    constraints = IK.Constraints()

    l0 = rr.robot.getLink('LINK0')
    l1 = rr.robot.getLink('LINK1')

    ### input angle for the first joint
    if targetq is not None:
        if joint_id == 0:
            l0.q = targetq
        else:
            l1.q = targetq
    if flush:
        rr.flush()
    else:
        rr.robot.calcForwardKinematics()

    ex_constraint = IK.PositionConstraint()
    ex_constraint.A_link     = rr.robot.getLink('LINKD')
    cds_a = iu.coordinates(np.array([1.3, 0, 0]))
    ex_constraint.A_localpos = cds_a.toPosition()
    ex_constraint.B_link     = rr.robot.getLink('LINK1')
    cds_b = iu.coordinates(np.array([0,   0, 0.07]))
    ex_constraint.B_localpos = cds_b.toPosition()
    ex_constraint.weight = np.array([1, 1, 1, 0, 0, 0]) ## xyz : constrained

    j_constraint = IK.JointAngleConstraint()
    if joint_id == 0:
        j_constraint.joint = l0
        j_constraint.targetq = l0.q
    else:
        j_constraint.joint = l1
        j_constraint.targetq = l1.q
    # j_constraint.maxError =
    # j_constraint.precision =
    j_constraint.weight = 10.0

    constraints.push_back(ex_constraint)
    constraints.push_back(j_constraint)

    jlim_avoid_weight_old = np.zeros(6 + rr.robot.getNumJoints())
    dq_weight_all = np.append(np.zeros(6), np.ones(rr.robot.getNumJoints())) ## root is fixed

    loop = IK.solveFullbodyIKLoopFast(rr.robot,
                                      constraints,
                                      jlim_avoid_weight_old,
                                      dq_weight_all,
                                      30,
                                      1e-5,
                                      0)
    print("loop: {}".format(loop))
    if flush:
        rr.flush()
    else:
        doClosedIK = False
        rr.flush()

solveClosedIK(0.6)

def voidIK():
    solveClosedIK(flush=False)

if ru.isInChoreonoid():
    print("solveClosedIK() ## after you move joints")
    rb.sigKinematicStateChanged.connect(voidIK)
