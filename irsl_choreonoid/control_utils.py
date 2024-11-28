#####
#
# Controllers
#
#####
class ControlPD(object):
    def __init__(self, dt=0.001, P=10, D=0.01, VP=None):
        """
        """
        self.dt = dt
        self.prev_angle = None
        self.prev_error = None
        self.target_angle    = 0.0
        self.target_velocity = 0.0
        self.P = P
        self.D = D
        self.VP = VP

    def setTarget(self, target_q, target_dq = 0.0):
        """
        """
        self.target_angle    = target_q
        self.target_velocity = target_dq

    def calc(self, q, vel=None):
        """
        """
        if self.prev_angle is None:
            self.prev_angle = q
        if vel is None:
            current_velocity = (q - self.prev_angle)/self.dt
        else:
            current_velocity = vel
        ##
        err = self.target_angle - q
        if self.prev_error is None:
            self.prev_error = err
        ##
        res =  self.P * err + self.D * ( err - self.prev_error )/self.dt
        if self.VP is not None:
            res += self.VP * (self.target_velocity - current_velocity)
        ##
        self.prev_error = err
        self.prev_angle = q
        return res

class JointPD(ControlPD):
    def __init__(self, joint, **kwargs):
        """
        """
        super().__init__(**kwargs)
        self.joint = joint
        self.target_angle    = joint.q
        self.target_velocity = joint.dq

    def setJointTarget(self):
        """
        """
        self.target_angle    = self.joint.q_target
        self.target_velocity = self.joint.dq_target

    def set(self):
        """
        """
        u = self.calc(self.joint.q)
        self.joint.u = u

    def setWithVel(self):
        """
        """
        u = self.calc(self.joint.q, self.joint.dq)
        self.joint.u = u

class PDController(object):
    def __init__(self, body, dt=0.001, P=100, D=0.01, VP=None, settings=None):
        """
        """
        self.body = body
        self.len  = body.getNumJoints()
        self.dt = dt
        self.default_P = P
        self.default_D = D
        self.default_VP = VP
        self.settings = settings
        self.controllers = [ self.setJointPD(body.joint(idx)) for idx in range(self.len) ]

    def setJointPD(self, joint):
        lnm = joint.name
        jnm = joint.jointName
        if self.settings is not None and lnm in self.settings:
            s_ = self.settings[lnm]
        elif self.settings is not None and jnm in self.settings:
            s_ = self.settings[lnm]
        else:
            return JointPD(joint, dt=self.dt, P=self.default_P, D=self.default_D, VP=self.default_VP)
        return JointPD(joint, dt=self.dt,
                       P  = s_['P']  if 'P'  in s_ else self.default_P,
                       D  = s_['D']  if 'D'  in s_ else self.default_D,
                       VP = s_['VP'] if 'VP' in s_ else self.default_VP)

    def control(self, setJointTarget=True):
        """
        """
        for ctrl in self.controllers:
            if setJointTarget:
                ctrl.setJointTarget()
            ctrl.set()

    def controlWithVel(self, setJointTarget=True):
        """
        """
        for ctrl in self.controllers:
            if setJointTarget:
                ctrl.setJointTarget()
            ctrl.setWithVel()

#####
#
# Sequencers
#
#####
## linearInterpolator
## 
class Sequencer(object):
    def __init__(self, dt=0.001):
        self.sequence = []
        self.dt = dt
    def pushTargetAngle(self, angle, time, startVel=None, stopVel=None):
        pass
    def pushTargetAngles(self, angles, times, **kwargs):
        pass
    ## def insert(self),###
    def pop(self):
        pass
    def lastAngle(self):
        pass
    def lastVelocity(self):
        pass
