## cnoid.Util
import cnoid.Util as cutil
## cnoid.Body
import cnoid.Body as cbody
from cnoid.Body import Body
from cnoid.Body import Link
from cnoid.Body import Device
## assimp
from cnoid.AssimpPlugin import *
## IRSL (not base)
from cnoid.IRSLCoords import coordinates
import cnoid.IRSLCoords as IC
import cnoid.IRSLUtil as IU
from irsl_choreonoid.draw_coords import GeneralDrawInterfaceWrapped as DrawInterface
from irsl_choreonoid.draw_coords import DrawCoordsListWrapped as DrawCoords
import irsl_choreonoid.make_shapes as mkshapes
import irsl_choreonoid.cnoid_util as iu
from irsl_choreonoid.cnoid_util import parseURL
import irsl_choreonoid.robot_util as ru
from irsl_choreonoid.robot_util import RobotModelWrapped as RobotModel
from irsl_choreonoid.RobotBuilder import RobotBuilder
from irsl_choreonoid.RobotBuilder import SimpleRobotBuilder
## etc
import numpy as np
from numpy import array as npa
from numpy.linalg import norm
from irsl_choreonoid.cnoid_util import fv ## for npa
import math
from math import pi as PI
##
if iu.isInChoreonoid():
    ## in base
    import irsl_choreonoid.cnoid_base as ib
    import cnoid.Base as cbase
    import cnoid.BodyPlugin as BodyPlugin
## ROS
try:
    from irsl_choreonoid_ros.setup_cnoid import SetupCnoid
    import irsl_choreonoid_ros.cnoid_ros_util as cru
    from irsl_choreonoid_ros.RobotInterface import RobotInterface
    from irsl_choreonoid_ros.cnoid_ros_util import parseURLROS as parseURL
except ImportError:
    pass
## utility
def exec_script(fname, *args):
    """Execute python code written to textfile
    Usage: exec_script('filename.py', locals(), globals())
    """
    exec(open(fname).read(), *args)
### reload module
#import X
# >>> reaload( X )
#from X import Y
# >>> import X
# >>> reload(X)
# >>> from X import Y
from importlib import reload
