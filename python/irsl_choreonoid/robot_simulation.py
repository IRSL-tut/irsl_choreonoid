import cnoid.Util
import cnoid.Base
import cnoid.BodyPlugin
import cnoid.IRSLSimPlugin
#import robot_util.py as ru
from . import robot_util as ru
import numpy as np

def clear_all_items():
    rt_ = ru.getRootItem()
    if rt_ is None:
        return
    ch_ = rt_.childItem
    if ch_ is None:
        return
    ch_.detachFromParentItem()

def create_robot_environment(robot_fname, create_simulator = True, add_controller = True, robot_name = None, world = 'World', extra_objects = None):
    rt_ = ru.getRootItem()
    if type(world) == str:
        wd_ = rt_.findItem(world)
        if wd_ is None:
            wd_ = cnoid.BodyPlugin.WorldItem()
            wd_.setName(world)
            rt_.addChildItem(wd_)
    elif type(world) == cnoid.BodyPlugin.WorldItem:
        wd_ = world
        #rt_.addChildItem(wd_) ## ??
    else:
        return None
    ru.getItemTreeView().checkItem(wd_)

    if create_simulator:
        sim_ = ru.addSimulator(world = wd_)
        if sim_ is None:
            return None
        ### add floor
        ru.loadRobot(cnoid.Util.getShareDirectory() + '/model/misc/floor.body', world = wd_)

    bdi_ = ru.loadRobot(robot_fname, name = robot_name, world = wd_, return_item = True)
    if bdi_ is None:
        return None

    if add_controller:
        cont_ = cnoid.IRSLSimPlugin.SimStepControllerItem()
        if cont_ is None:
            return None
        bdi_.addChildItem(cont_)
        ru.getItemTreeView().checkItem(cont_)

    if not extra_objects is None:
        for objs in extra_objects:
            ru.loadRobot(objs, world = wd_)

    if create_simulator and add_controller:
        sim_rb_ = SimRobot(controller = cont_, simulator = sim_)
        return sim_rb_

    return True

# TODO
## create environment and ...
## simulation_env / robot model with ...
### fname and model-settings :=> model-settings.yamlにfnameを含む
## start simulation with current settings
class SimRobot(object):
    def __init__(self, controller, simulator):
        self.controller = controller
        self.simulator  = simulator
        self.simulator.setRealtimeSyncMode(False)
        self.initialized = False

        self.robot_item = self.controller.parentItem
        self.robot = self.robot_item.body
        ##
        #for idx in range(self.robot.numJoints):
        #    j = self.robot.joint(idx)
        #    j.setEquivalentRotorInertia(0.01)

    def startSimulation(self, doReset = False):
        #self.simulator.startSimulation(doReset)
        cnoid.BodyPlugin.SimulationBar.instance.startSimulation(doReset)
        self.controller.wait_next_step()
        self.initialize()
        self.initialized = True

    def stopSimulation(self):
        self.initialized = False
        self.controller.disable_stepping()
        self.simulator.stopSimulation()

    def flush(self):
        cnoid.Base.MessageView.instance.flush() ## ??

    def tick(self, count = 1):
        if not self.initialized:
            return
        cnt_prev = -1
        while(count > 0):
            cnt = self.controller.wait_next_step()
            while cnt <= cnt_prev:
                cnt = self.controller.wait_next_step()
                self.controller.usleep(50)
            cnt_prev = cnt
            self.PDcontrol()
            self.controller.notify_sim_step()
            self.flush()
            count = count -1

    def initialize(self):
        r = self.controller.getSimBody()
        self.simrobot = r
        self.dt = self.controller.timeStep()
        #self.initial_angles = r.angleVector()
        self.initial_angles = np.array([ r.joint(idx).q for idx in range(r.numJoints) ])
        self.reference_angles = np.copy(self.initial_angles)
        # len(initial_angles)
        self.kp_coefficients = np.full(r.numJoints, 5000.0)
        self.kd_coefficients = np.full(r.numJoints, 100.0)
        self.target_torque = np.full(r.numJoints, 0.0)

        for idx in range(r.numJoints):
            j = r.joint(idx)
            # amode = j.actuationMode
            j.setActuationMode(cnoid.Body.Link.JointEffort)

            #jmode = j.sensingMode
            j.mergeSensingMode(cnoid.Body.Link.JointAngle)
            j.mergeSensingMode(cnoid.Body.Link.JointVelocity)
            j.mergeSensingMode(cnoid.Body.Link.JointAcceleration)
            j.mergeSensingMode(cnoid.Body.Link.LinkPosition)
            j.mergeSensingMode(cnoid.Body.Link.LinkTwist)
            j.mergeSensingMode(cnoid.Body.Link.LinkExtWrench)
            j.mergeSensingMode(cnoid.Body.Link.LinkContactState)

    def PDcontrol(self):
        for idx in range(self.simrobot.numJoints):
            j = self.simrobot.joint(idx)
            qref = self.reference_angles[idx]
            Kp = self.kp_coefficients[idx]
            Kd = self.kd_coefficients[idx]
            j.u = self.target_torque[idx] ## output previous
            self.target_torque[idx] = Kp * (qref - j.q) + Kd * (0 - j.dq)
