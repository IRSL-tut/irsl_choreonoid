import cnoid.BodyPlugin as BodyPlugin
import cnoid.Body as cbody
import cnoid.Util as cutil
import cnoid.IRSLUtil as IU
import irsl_choreonoid.cnoid_base as ib
from .control_utils import PDController
from .control_utils import BodySequencer
from .robot_util import mergedMassPropertyOfAllDescendants
import numpy as np
import math

## generatesettings of body
def _generatePDParameters(body, dt=0.001, weightP=0.05, weightD=1.0, weightI=1.0, updateRotorInertia=False, **kwargs):
    res = {}
    kk=0.05 * weightP
#    if P == 'hard':
#        kk=0.5
#    elif P == 'soft':
#        kk=0.0005
    kd=0.5 * weightD
#    if D == 'hard':
#        kd=2.0
#    elif D == 'soft':
#        kd=0.1
    kI=1.0 * weightI
#    if I == 'hard':
#        kI=0.1
#    elif I == 'soft':
#        kI=10
    ##
    for idx in range(body.numJoints):
        j = body.joint(idx)
        ax = j.jointAxis
        lk_mass, lk_c, lk_I = mergedMassPropertyOfAllDescendants(j)
        joint_m = lk_mass * ( np.linalg.norm(np.cross(ax, lk_c)) ** 2) + np.dot(ax, lk_I @ ax)
        P = 2*kk*joint_m/dt/dt
        D = P * kd * ( 10 ** (math.log10(joint_m/5)/4 - 1.5) )
        if updateRotorInertia:
            I = kI * joint_m
            j.setEquivalentRotorInertia(I)
        res[j.jointName] = {'P': P, 'D': D}
    ##
    return res
##
## TODO
## record item
## record images
##
class SimulationEnvironment(object):
    def __init__(self, robotName, simulator=None, addSimulator=True, fixed=False, world=None):
        """
        Args:
            robotName ( str ) : Name of robot in this simulation
            simulator ( cnoid.BodyPlugin.SimulatorItem, optional ) : SimulatorItem will be used
            addSimulator ( boolean, default=True) : If True, simulatorItem will be added
            fixed ( boolean, default=False) : If True, robot is fixed
            world ( str or cnoid.BodyPlugin.WorldItem) : World Item
        """
        self.robot_name = robotName
        if simulator is None:
            res = ib.findItemsByClass( BodyPlugin.SimulatorItem )
            if len(res) == 0:
                if addSimulator:
                    simulator = ib.addSimulator(world=world)
                else:
                    raise Exception('No simulator found')
            else:
                simulator = res[0]
        ## checkRobotName
        self.sim = simulator
        self.sim.setRealtimeSyncMode(3) ## manual-mode
        #self.dt = sim.worldTimeStep
        self.controller = None
        self.sequencer = None
        self._world  = None
        self._sim_body = None
        self._sbody = None
        self._robotItem = None
        self._findRobot(fixed)


    @property
    def robot(self):
        """
        RobotItem that this instance is treated as main-robot
        """
        return self._robotItem

    @property
    def simBody(self):
        """
        SimulationBody that this instance is treated as main-robot
        """
        return self._sim_body

    @property
    def sbody(self):
        """
        Body that this instance is treated as main-robot (equal simBody.body)
        """
        return self._sbody

    @property
    def world(self):
        """
        WorldItem to which this simulator refers
        """
        if self._world is None:
            itm = self.sim.parentItem
            if type(itm) is BodyPlugin.WorldItem:
                ## simulation should be under worldItem
                self._world = itm
        return self._world

    @property
    def bodies(self):
        """
        List of BodyItem which is in this world

        Returns:
            list [ cnoid.BodyPlugin.BodyItem ] :

        """
        return ib.findItemsByClass(BodyPlugin.BodyItem, root=self.world)

    @property
    def bodyNames(self):
        """
        List of a name of BodyItem which is in this world

        Returns:
            list [ str ] :
        """
        return [ bd.name for bd in self.bodies ]

    @property
    def simulationBodies(self):
        """
        List of simulationbody which is in this world

        """
        return [ self.sim.findSimulationBody(bd.name) for bd in self.bodies ]

    @property
    def worldTimeStep(self):
        """
        Time step of this simulation

        Returns:
            float : Time-Step [second]

        """
        return self.sim.worldTimeStep

    @worldTimeStep.setter
    def worldTimeStep(self, sec):
        """
        Set time step of this simulation

        Args:
            src (float) : Time-Step [second]

        """
        self.sim.setTimeStep(sec)

    def simulationBody(self, name=None):
        """Search simulation body by name

        Args:
            name (str, optional) : name of simulation body

        Returns:
            simulationbody : Simulation body which has designated name
        """
        if name is None:
            return self.sim.findSimulationBody(self.robot_name)
        else:
            return self.sim.findSimulationBody(name)

    def _findRobot(self, fixed=False):
        """Robot is fixed to the environment
        """
        res = ib.findItemsByName(self.robot_name, root=self.world)
        if len(res) > 1:
            pass
        if len(res) == 0:
            raise Exception('No robot({}) found under {}'.format(self.robot_name, self.world))
        self._robotItem = res[0]
        if fixed:
            self._robotItem.body.rootLink.setJointType(cbody.Link.FixedJoint)

    def start(self, dt=None, addCountroller=True, addSequencer=True, controllerSettings=None,
              P=10000, D=200, generatePDSettings=False, rotorInertia=None, **kwargs):
        """Start simulation

        Args:
            dt (float) : Time step of this simulation [second]
            addCountroller (boolean, default=True) : Adding PDcontroller to servo joints
            addSequencer (boolean, default=True) : Adding sequencer to set target-angles with interpolatin
            controllerSettings (optional) : {'joint_name0': {'P': pgain, 'D': dgain, 'rotorInertia': IM2}, ... }
            P (float, default=10000) : default P-gain
            D (float, default=200) : default D-gain
            generatePDSettings (boolean, default=False) :
        """
        if dt is not None:
            self.worldTimeStep = dt

        if generatePDSettings:
            controllerSettings=_generatePDParameters(self._robotItem.body, **kwargs)
            self.PDSettings=controllerSettings
        if rotorInertia is not None:
            for j in self._robotItem.body.joints:
                j.setEquivalentRotorInertia(rotorInertia)
        if controllerSettings:
            for k, v in controllerSettings.items():
                if 'rotorInertia' in v:
                    j = self._robotItem.body.joint(k)
                    if j is not None:
                        j.setEquivalentRotorInertia(v['rotorInertia'])
        self.sim.startSimulation()
        sim_body = self.sim.findSimulationBody(self.robot_name)
        if sim_body is None:
            self.sim.stopSimulation()
            raise Exception('No body found : {}'.format(self.robot_name))
        self._sim_body = sim_body
        self._sbody = sim_body.body()
        self.controller = None
        self.sequencer  = None
        if addCountroller:
            self.controller = PDController(self._sbody, dt=self.worldTimeStep, P=P, D=D, settings=controllerSettings)
        if addSequencer:
            self.sequencer = BodySequencer(self._sbody, dt=self.worldTimeStep)

    def storeInitialState(self):
        """Storing initial state of all objects in this world
        """
        for bd in self.bodies:
            bd.storeInitialState()

    def isRunning(self):
        """Simulation is running or not
        """
        return self.sim.isRunning()

    def stop(self):
        """Stop simulation
        """
        self.sim.stopSimulation()

    def restart(self):
        pass

    def sendAngleVector(self, angle_vector, tm=1.0, **kwargs):
        """Set target angle-vector to the robot (similar to RobotInterface)

        Args:
            angle_vector (numpy.array) : Target angle-vector
            tm (float, default=1.0) : Target duration of motion
        """
        if self.sequencer is not None:
            self.sequencer.pushNextAngle(angle_vector, tm)

    def sendAngleVectorSequence(self, angle_vector_list, tm_list, **kwargs):
        """Set target angle-vectors to the robot (similar to RobotInterface)

        Args:
            angle_vector_list ( list[ numpy.array ] ) : Target list of angle-vector
            tm_list ( list[ float ] ) : Target list of motion duration
        """
        if self.sequencer is not None:
            self.sequencer.pushNextAngles(angle_vector_list, tm_list)

    def runCount(self, count, update=33, stop=False, callback=None, update_callback=None, **kwargs):
        """Run simulation

        Args:
            count (int) : Number of simulation time steps to run
            update (int, default=33) : Update visual each count of this argument
            stop (boolean, default=False) : If True, stop simulation at the end of this method
            callback ( callable, optional ) : Callback function called every control cycle
            update_callback ( callable, optional ) : Callback function called every update
        """
        if not self.sim.isRunning():
            self.start(**kwargs)
        for i in range(count):
            if not self.sim.isRunning():
                break
            ##
            if self.sequencer is not None:
                self.sequencer.setNextTarget()
            if self.controller is not None:
                self.controller.control()
            ##
            if update is not None:
                if i % update == 0:
                    self.sim.tickRequest()
                    if update_callback is not None:
                        update_callback(self)
                    IU.processEvent()
                    while self.sim.tickRequested():
                        IU.usleep(1)
                else:
                    self.sim.tickRequest(True)
            else:
                self.sim.tickRequest(True)
            if callback is not None:
                callback(self)
        if stop:
            self.sim.stopSimulation()

    def run(self, sec, update=33, stop=False, callback=None, update_callback=None, **kwargs):
        """Run simulation

        Args:
            sec (float) : Duration for running the simulation
            update (int, default=33) : Update visual each count of this argument
            stop (boolean, default=False) : If True, stop simulation at the end of this method
            callback ( callable, optional ) : Callback function called every control cycle
            update_callback ( callable, optional ) : Callback function called every update
        """
        return self.runCount(math.floor(sec/self.worldTimeStep),
                             update = update, stop = stop,
                             callback = callback, update_callback = update_callback,
                             **kwargs)

def setupSimEnv(robot_class, robotName=None, addFloor=True, initialCoords=None, initialPose=None):
    mrobot = robot_class(world=True, name=robotName)
    if addFloor:
        ib.loadRobotItem(cutil.getShareDirectory() + '/model/misc/floor.body')
    if type(initialPose) is str:
        if hasattr(mrobot, initialPose):
            setpose = getattr(mrobot, initialPose)
            setpose()
        if initialCoords is not None:
            mrobot.fixLegToCoords(initialCoords)
    elif type(initialPose) is np.array:
        mrobot.angleVector(initialPose)
        mrobot.fixLegToCoords(initialCoords)
    elif initialPose:
        if initialCoords is not None:
            mrobot.fixLegToCoords(initialCoords)
    else:
        if hasattr(mrobot, 'setSimDefaultPose'):
            mrobot.setSimDefaultPose()
        elif hasattr(mrobot, 'setDefaultPose'):
            mrobot.setDefaultPose()
        elif hasattr(mrobot, 'setInitialPose'):
            mrobot.setInitialPose()

        if initialCoords is not None:
            mrobot.fixLegToCoords(initialCoords)

    mrobot.item.storeInitialState()

    simenv = SimulationEnvironment(mrobot.item.name)

    return simenv, mrobot

def startSimEnv(simenv, mrobot, pre_wait=2.0, post_wait=2.0, **kwargs):
    simenv.stop()
    if hasattr(mrobot, 'getJointSettings'):
        simenv.start(controllerSettings=mrobot.getJointSettings(**kwargs))
    else:
        simenv.start(generatePDSettings=True, **kwargs)
    simenv.run(pre_wait)
