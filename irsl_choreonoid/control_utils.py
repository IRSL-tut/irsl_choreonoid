import math
import scipy.interpolate
import numpy as np
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
        self.joint.q_target = joint.q
        self.joint.dq_target = joint.dq
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
            s_ = self.settings[jnm]
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
##
## TODO:linearInterpolator
##
def interpolate1D(tt, yy):
    vv = [0.0]
    for idx in range(1, len(tt)-1):
        v_prev = (yy[idx] - yy[idx-1])/(tt[idx] - tt[idx-1])
        v_next = (yy[idx+1] - yy[idx])/(tt[idx+1] - tt[idx])
        if v_prev * v_next <= 0.0:
            v = 0.0
        else:
            v = (v_prev + v_next) * 0.5
        vv.append(v)
    vv.append(0.0)
    #
    ff = []
    for st, ed, y_st, y_ed, v_st, v_ed in zip(tt[0:-1], tt[1:], yy[0:-1], yy[1:], vv[0:-1], vv[1:]):
        t_ = np.array([st, ed])
        y_ = np.array([y_st, y_ed])
        bc_type=[ [(1, v_st), (2, 0.0)], [(1, v_ed), (2, 0.0)] ]
        ff.append( scipy.interpolate.make_interp_spline(t_, y_, k = 5, bc_type=bc_type) )
        #bc_type=[ [(1, v_st) ], [(1, v_ed) ] ]
        #ff.append( scipy.interpolate.make_interp_spline(t_, y_, k = 3, bc_type=bc_type) )
    return ff

def interpolateVector(tt, vec_list, rate):
    vsize = len(vec_list[0])
    allmat = np.array(vec_list)
    func_lst = []
    for idx in range(vsize):
        func_lst.append( interpolate1D(tt, allmat[:, idx].tolist()) )
    #
    npt = [ np.linspace(st, ed, num=(math.floor( (ed - st)*rate ) + 1))[1:] for st, ed in zip(tt[0:-1], tt[1:]) ]
    #
    ttnew = []
    vecnew = [ list() for idx in range(vsize) ]
    for segment in range(len(npt)):
        nn = npt[segment].tolist()
        ttnew += nn
        for idx in range(vsize):
            res = func_lst[idx][segment](nn).tolist()
            vecnew[idx] += res
    #res = np.array(vecnew)
    return ttnew, vecnew

def interpolateCoords(tt, coords_list):
    """
    not implemented yet
    """
    pass

class Sequencer(object):
    """ Sequencer to set target-angles every cycle with interpolatin
    """
    def __init__(self, dt=0.001):
        """
        Args:
            dt (float, default=0.001) :
        """
        self.sequence = [[]]
        self.dt = dt
        self.prev_angle_vec = None
        self.prev_velocity_vec = None
    def _appendAngles(self, vec):
        if len(self.sequence[0]) == 0:
            self.sequence = vec
        else:
            if len(self.sequence[0]) != len(vec):
                raise Exception('invalid length of angles, org: {}, new: {}', len(self.sequence[0]), len(vec))
            for seq, v in zip(self.sequence, vec):
                seq += v
    def pushTargetAngle(self, start_angle_vector, angle_vector, time, startVel=None, stopVel=None):
        tt, vec = interpolateVector([0, time], [start_angle_vector, angle_vector], math.ceil(1/self.dt))
        self._appendAngles(vec)
    def pushTargetAngles(self, angle_vectors, times_from_start):
        tt, vec = interpolateVector(times_from_start, angle_vectors, math.ceil(1/self.dt))
        self._appendAngles(vec)
    def setNoInterpolation(self, angle_vectors, step=1):
        """Set angle-vectors without interpolation

        Args:
            angle_vectors ( list[ numpy.array ] ) : List of angle-vector
            step (int, default=1) : Target angle is updated every step-times of cycle
        """
        if step > 1:
            avs = []
            for av, nav in zip(angle_vectors[:-1], angle_vectors[1:]):
                for i in range(step):
                    avs.append( ( (step - i)*av + i*nav ) / step )
            avs.append( angle_vectors[-1] )
        else:
            avs = angle_vectors
        mat = np.array(avs)
        tmat = mat.transpose()
        self.sequence = tmat.tolist()
    def setWithSteps(self, angle_vector, step=2):
        """Set single angle-vector with linear interpolation

        Args:
            angle_vector ( numpy.array ) : Target angle-vector
            step (int, default=2) : After step cycle, target-angles equals to angle_vector
        """
        if self.prev_angle_vec is None:
            self.setNoInterpolation( [angle_vector] )
        else:
            mat = np.array( [(self.prev_angle_vec + angle_vector)*0.5, angle_vector] )
            self.sequence = mat.transpose().tolist()
    ## def insert(self),###
    def pop(self):
        if len(self.sequence[0]) == 0:
            return
        angles = [ lst.pop(0) for lst in self.sequence ]
        res = np.array(angles)
        self.prev_angle_vec = res
        return res
    ##
    @property
    def remainCount(self):
        return len(self.sequence[0])
    ##
    def clear(self, start=0, end=None):
        for lst in self.sequence:
            del lst[start:end]
    #def lastAngleVector(self):
    #    pass
    #def lastVelocityVector(self):
    #    pass

class BodySequencer(Sequencer):
    def __init__(self, body, dt=0.001):
        """
        """
        super().__init__(dt=dt)
        self.body = body
    def pushNextAngle(self, target_angle_vector, time):
        self.pushNextAngles( [target_angle_vector], [time] )
    def pushNextAngles(self, angle_vectors, durations):
        if type(angle_vectors) is tuple:
            angle_vectors = list(angle_vectors)
        angle_vectors.insert(0, self.body.getAngles())
        #
        cur_time = 0.0
        times = [ cur_time ]
        for du in durations:
            cur_time += du
            times.append(cur_time)
        #
        self.pushTargetAngles(angle_vectors, times)
    def setNextTarget(self):
        res = self.pop()
        if res is None:
            return
        self.body.setTargetAngles(res)
