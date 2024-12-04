# jupyter console --kernel=choreonoid
exec(open('/choreonoid_ws/install/share/irsl_choreonoid/sample/irsl_import.py').read())

## Robot
from irsl_choreonoid.sample_robot import SampleRobot
robot = SampleRobot.makeRobot()
robot.setDefaultPose()
end = robot.angleVector()

## add floor (object for simulation)
ib.loadRobotItem(cutil.getShareDirectory() + '/model/misc/floor.body')

from irsl_choreonoid.simulation_utils import SimulationEnvironment
sim = SimulationEnvironment('SR1')
sim.start()
sim.sequencer.pushNextAngle(end, 5.0)
sim.run(10)
