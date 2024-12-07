import irsl_choreonoid.cnoid_base as ib
import cnoid.BodyPlugin as BodyPlugin
from .control_utils import PDController
from .control_utils import BodySequencer
import cnoid.IRSLUtil as IU
import math

class SimulationEnvironment(object):
    def __init__(self, robotName, simulator=None, addSimulator=True, world=None):
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
        self.controller = None
        self.sequencer = None
    def start(self, addCountroller=True, addSequencer=True, controllerSettings=None, P=10000, D=200):
        self.sim.startSimulation()
        sim_body = self.sim.findSimulationBody(self.robot_name)
        if sim_body is None:
            self.sim.stopSimulation()
            raise Exception('No body found : {}'.format(self.robot_name))
        self.body = sim_body.body()
        self.controller = None
        self.sequencer = None
        if addCountroller:
            self.controller = PDController(self.body, P=P, D=D, settings=controllerSettings)
        if addSequencer:
            self.sequencer = BodySequencer(self.body)
    def storeInitial(self):
        pass
    def isRunning(self):
        return self.sim.isRunning()
    def stop(self):
        self.sim.stopSimulation()
    def restart(self):
        pass
    def sendAngleVector(self, angle_vector, tm=1.0, **kwargs):
        if self.sequencer is not None:
            self.sequencer.pushNextAngle(angle_vector, tm)
    def sendAngleVectorSequence(self, angle_vector_list, tm_list, **kwargs):
        if self.sequencer is not None:
            self.sequencer.pushNextAngles(angle_vector_list, tm_list)
    ## stop
    ## restart
    ## isRunning
    def run(self, sec, update=33, stop=True, callback=None, **kwargs):
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
                    IU.processEvent()
                    while self.sim.tickRequested():
                        IU.usleep(1)
                else:
                    self.sim.tickRequest(True)
            else:
                self.sim.tickRequest(True)
            if callback is not None:
                callback(self.body)
        if stop:
            self.sim.stopSimulation()
