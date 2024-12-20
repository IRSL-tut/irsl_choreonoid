import cnoid.BodyPlugin as BodyPlugin
import cnoid.Body as cbody
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
        self.findRobot(fixed)

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

    def findRobot(self, fixed=False):
        """Robot is fixed to the environment
        """
        res = ib.findItemsByName(self.robot_name, root=self.world)
        if len(res) > 1:
            pass
        if len(res) == 0:
            raise Exception('No robot({}) found under {}'.format(self.robot_name, self.world))
        self.robotItem = res[0]
        if fixed:
            self.robotItem.body.rootLink.setJointType(cbody.Link.FixedJoint)

    def start(self, addCountroller=True, addSequencer=True, controllerSettings=None, P=10000, D=200, generatePDSettings=False, **kwargs):
        """Start simulation

        Args:
            addCountroller (boolean, default=True) :
            addSequencer (boolean, default=True) :
            controllerSettings (optional) :
            P (float, default=10000) :
            D (float, default=200) :
            generatePDSettings (boolean, default=False) :
        """
        if generatePDSettings:
            controllerSettings=_generatePDParameters(self.robotItem.body, **kwargs)
            self.PDSettings=controllerSettings
        self.sim.startSimulation()
        sim_body = self.sim.findSimulationBody(self.robot_name)
        if sim_body is None:
            self.sim.stopSimulation()
            raise Exception('No body found : {}'.format(self.robot_name))
        self.sim_body = sim_body
        self.body = sim_body.body()
        self.controller = None
        self.sequencer = None
        if addCountroller:
            self.controller = PDController(self.body, dt=self.sim.worldTimeStep, P=P, D=D, settings=controllerSettings)
        if addSequencer:
            self.sequencer = BodySequencer(self.body, dt=self.sim.worldTimeStep)

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
            angle_vector () :
            tm (float, default=1.0) :
        """
        if self.sequencer is not None:
            self.sequencer.pushNextAngle(angle_vector, tm)

    def sendAngleVectorSequence(self, angle_vector_list, tm_list, **kwargs):
        """Set target angle-vectors to the robot (similar to RobotInterface)

        Args:
            angle_vector_list () :
            tm_list ( list[ float ] ) :
        """
        if self.sequencer is not None:
            self.sequencer.pushNextAngles(angle_vector_list, tm_list)

    def run(self, sec, update=33, stop=False, callback=None, update_callback=None, **kwargs):
        """Run simulation

        Args:
            sec (float) : Duration for running the simulation
            update (int, default=33) : Update visual each count of this argument
            stop (boolean, default=False) : If True, stop simulation at the end of this method
            callback ( callable, optional ) : Callback function called every control cycle
            update_callback ( callable, optional ) : Callback function called every update
        """
        if not self.sim.isRunning():
            self.start(**kwargs)
        for i in range(math.floor(sec*1000)):
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
