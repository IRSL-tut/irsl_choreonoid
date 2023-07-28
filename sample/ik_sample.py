### sample for IKWrapper
import irsl_choreonoid.robot_util as ru
import irsl_choreonoid.cnoid_util as cu
import numpy as np

##
joints_a = [ 'LARM_SHOULDER_P', 'LARM_SHOULDER_R', 'LARM_SHOULDER_Y', 'LARM_ELBOW', 'LARM_WRIST_Y', 'LARM_WRIST_P', 'LARM_WRIST_R' ]
joints_b = joints_a + ['WAIST_P', 'WAIST_R', 'CHEST']
##
if cu.isInChoreonoid():
    ## on choreonoid
    bi = cu.loadRobotItem(cu.parseURL('choreonoid://share/model/SR1/SR1.body'))
    bd = bi.body
else:
    ## on console
    bd = cu.loadRobot(cu.parseURL('choreonoid://share/model/SR1/SR1.body'))

## ru.IKWrapper(instance-of-body, target-link(name, id, link))
## use_joints : optional, list of using joint(name, id, joint)
ik = ru.IKWrapper(bd, 'LARM_WRIST_R', use_joints = joints_a)
#ik = ru.IKWrapper(bd, 'LARM_WRIST_R', use_joints = joints_a, solver = 'LM') ## here is original IK

tgtorg = ik.getEndEffector() ## target coordinates
tgt = ik.getEndEffector().translate(np.array([0, 0, 0.1])) ## moved target

## solve inverse-kinematics
## inverseKinematics(target-coords)
## constraint : 'xyzRPY' or [1, 1, 1, 1, 1, 1], 'xyz' is just position(3-axis), 'xyzRPY' is for 6-axis
## add_noise : noise [rad], noise is added to joints before solving IK
## debug: True or False, print debug message
succ, _iter = ik.inverseKinematics(tgt, debug = True, constraint = 'xyz', add_noise=0.3)
while succ is False: ## sometimes ik fails
    succ, _iter = ik.inverseKinematics(tgt, debug = True, constraint = 'xyz', add_noise=0.45)
ik.flush()
print('original: {}\n  target: {}\n  solved: {}'.format(tgtorg, tgt, ik.endEffector))

## re-solving inverse-kinematics with different constraint
succ, _iter = ik.inverseKinematics(tgt, debug = True, constraint = 'xyzRPY', add_noise=0.4)
ik.flush()
print('original: {}\n  target: {}\n  solved: {}'.format(tgtorg, tgt, ik.endEffector))

ik.resetPose() ## reset to initial-pose
ik.flush()

## Changing use_joint
ik2 = ru.IKWrapper(bd, 'LARM_WRIST_R', use_joints = joints_b)
tgtorg2 = ik2.getEndEffector()
tgt2 = ik2.getEndEffector().translate(np.array([0, 0, 0.1]))
succ, _iter = ik2.inverseKinematics(tgt, debug = True, constraint = [1,1,1, 1,1,1], add_noise=0.45)
ik2.flush()
print('original: {}\n  target: {}\n  solved: {}'.format(tgtorg2, tgt2, ik2.endEffector))
