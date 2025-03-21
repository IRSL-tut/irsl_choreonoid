## cnoid.Util
import cnoid.Util as cutil
## cnoid.Body
import cnoid.Body as cbody
from cnoid.Body import Body
from cnoid.Body import Link
from cnoid.Body import Device

## IRSL (not base)
from cnoid.IRSLCoords import coordinates
from .draw_coords import GeneralDrawInterfaceWrapped as DrawInterface

from . import make_shapes as mkshapes
from . import cnoid_util as iu
from . import robot_util as ru
## from .robot_util import RobotModelWrapped as RobotModel
import numpy as np
from numpy import array as npa
from numpy.linalg import norm
import math
from math import pi as PI

import cnoid.IRSLUtil
##
if iu.isInChoreonoid():
    ## in base
    from . import cnoid_base as ib
    #-import irsl_choreonoid.cnoid_base as ib
    import cnoid.Base as cbase
    import cnoid.BodyPlugin as BodyPlugin
try:
    from cnoid.URDFPlugin import URDFBodyWriter
except Exception as e:
    pass
##
import copy
from enum import IntEnum

class DummyInterface(object):
    def __init__(self):
        self.SgPosTransform = cutil.SgPosTransform()
    def clear(self):
        self.SgPosTransform = cutil.SgPosTransform()
    def objects(self):
        res = []
        for idx in range(self.SgPosTransform.numChildren):
            res.append(self.SgPosTransform.getChild(idx))
        return res
    def addObject(self, obj):
        if hasattr(obj, 'target'): ## coordsWrapper
            self.SgPosTransform.addChild(obj.target)
        else:
            self.SgPosTransform.addChild(obj)
    def removeObject(self, obj):
        if hasattr(obj, 'target'): ## coordsWrapper
            self.SgPosTransform.removeChild(obj.target)
        else:
            self.SgPosTransform.removeChild(obj)
    def addObjects(self, objlst):
        for obj in objlst:
            self.addObject(obj)
    def removeObjects(self, objlst):
        for obj in objlst:
            self.removeObject(obj)

## add inertia on shape
class RobotBuilder(object):
    """Building robot interactively
    """
    class JointType(IntEnum):
        Fixed = Link.JointType.FixedJoint.value
        Free  = Link.JointType.FreeJoint.value
        Revolute  = Link.JointType.RevoluteJoint.value
        Linear    = Link.JointType.PrismaticJoint.value
        Prismatic = Link.JointType.PrismaticJoint.value
        ## 2DOF
        LinearXZ = 92
        LinearYZ = 93
        LinearXY = 94
        RevoluteXZ = 96
        RevoluteYZ = 97
        RevoluteXY = 98
        ## 3DOF
        Planar     = 95 ##X,Y,theta
        Ball       = 99

    def __init__(self, robot=None, gui=True, name=None): ## item
        """
        Args:
            robot (,optional) :
            gui (boolean, default=True) :
            name (str, optional) :

        """
        self.__bodyItem = None
        self.__di = None
        self.__body = None
        self.__cnoid = False
        if gui and iu.isInChoreonoid():
            ## set self.__di
            self.__cnoid = True
            self.__di=DrawInterface()
            self.__setBodyItem(robot=robot, name=name)
        else:
            self.__di=DummyInterface()
        ## set self.__body
        self.__setBody(robot=robot)
        self.created_links = []

        self.__store_mode = 0 # 0: storing objects as shape,
                              # 1: storing objects converted to mesh as visual, and storing objects (as is) as collision if possible,
                              # 2: storing objects converted to mesh as shape (visual and collision)
                              # 3: collision is bounding-box
    def __setBodyItem(self, robot, name):
        ## set self.__bodyItem
        if isinstance(robot, BodyPlugin.BodyItem):
            self.__bodyItem = robot
        elif type(robot) is str:
            self.__bodyItem = ib.loadRobotItem(robot, world=False, addItem=False)
        else:
            self.__bodyItem = BodyPlugin.BodyItem()
        if name is None:
            name = 'BodyBuilder'
        self.bodyItem.setName(name)
        cbase.RootItem.instance.addChildItem(self.bodyItem)
        cbase.ItemTreeView.instance.checkItem(self.bodyItem)

    def __setBody(self, robot):
        if self.bodyItem is not None:
            self.__body = self.bodyItem.body
        else:
            if isinstance(robot, Body):
                self.__body = robot
            elif type(robot) is str:
                self.__robot = iu.loadRobot(robot)
            else:
                self.__body = Body()

    def __del__(self):
        ## print('destruct builder')
        if self.__di is not None:
            self.__di.clear()
            self.__di = None
        if self.bodyItem is not None:
            cbase.ItemTreeView.instance.checkItem(self.bodyItem, False)
            self.bodyItem.removeFromParentItem()
            # cbase.RootItem.instance.addChildItem(self.bodyItem)
            # del self.BodyItem

    def storeMode(self, mode=None):
        if mode is not None:
            self.__store_mode = mode
        return self.__store_mode

    @property
    def body(self):
        """Currently builded robot model

        Returns:
            cnoid.Body.Body : Currently builded robot model

        """
        return self.__body
    @property
    def bodyItem(self):
        """Currently displayed BodyItem

        Returns:
            cnoid.BodyPlugin.BodyItem : Currently displayed BodyItem

        """
        return self.__bodyItem
### start: GUI wrapper
    @property
    def draw(self):
        """DrawInterface for accessing raw-level display methods

        Returns:
            irsl_choreonoid.draw_coords.DrawInterfaceWrapped : DrawInterface

        """
        return self.__di

    def hideRobot(self):
        """Hiding robot model (uncheck RobotItem)
        """
        if self.bodyItem is not None:
            cbase.ItemTreeView.instance.checkItem(self.bodyItem, False)

    def showRobot(self):
        """Showing robot model (check RobotItem)
        """
        if self.bodyItem is not None:
            cbase.ItemTreeView.instance.checkItem(self.bodyItem)
            self.notifyUpdate()

    def resetRobot(self):
        """Resetting joint-angles of the robot
        """
        for lk in self.body.links:
            lk.q = 0
        self.notifyUpdate()

    def newRobot(self, robot=None, name=None):
        """Restarting with new robot(item)

        Args:
            robot (str, optional) :
            name (str, optional) : name of RobotItem

        """
        self.clear()
        if self.bodyItem is not None:
            cbase.ItemTreeView.instance.checkItem(self.bodyItem, False)
            self.bodyItem.removeFromParentItem()
            self.__setBodyItem(robot=robot, name=name)
        self.__setBody(robot=robot)

    def clear(self):
        """Clearing showing geometries
        """
        if self.__di is not None:
            self.__di.clear()

    def objects(self):
        """Getting list of shown objects
        """
        if self.__di is not None:
            return self.__di.objects()

    def addShape(self, shape):
        """Adding a shape

        Args:
            shape (cnoid.Util.SgNode) : Shape to be added

        """
        if self.__di is not None:
            self.__di.addObject(shape)
    def addShapes(self, shapes):
        """Adding a shapes

        Args:
            shapes ( list[cnoid.Util.SgNode] ) : Shapes to be added

        """
        if self.__di is not None:
            self.__di.addObjects(shapes)

    def removeShape(self, shape):
        """Removing a shape

        Args:
            shape (cnoid.Util.SgNode) : Shape to be removed

        """
        if self.__di is not None:
            self.__di.removeObject(shape)
    def removeShapes(self, shapes):
        """Removing a shapes

        Args:
            shapes ( list[cnoid.Util.SgNode] ) : Shapes to be removed

        """
        if self.__di is not None:
            self.__di.removeObjects(shapes)

    def createLinkFromShape(self, name=None, mass=None, density=1000.0, parentLink=None, root=False,
                            clear=True, collision=None, useCollisionForMassparam=False, overwriteMassparam=None, **kwargs):
        """Creating link from drawn shapes and appending to the other link

        Args:
            name (str, optional) :
            mass (float, optional) :
            density (float, default=1000.0) :
            parentLink (cnoid.Body.Link, optional) :
            root (boolean, default=False) :
            clear (boolean, default=True) :
            collision (cnoid.Util.SgNode, optional) :
            useCollisionForMassparam (boolean, default=False) : 
            overwriteMassparam (dict, optional): 'mass', 'COM', 'inertia'
            \*\*kwargs :

        Returns:
            cnoid.Body.Link : Created link

        """
        if self.__di is None:
            return
        if name is None:
            name='LINK_{}'.format(len(self.created_links))
        #groot=self.__di.target
        #hasattr(rb.draw, 'SgPosTransform' )
        groot=self.__di.SgPosTransform.clone()
        res = RobotBuilder.searchSceneGraph(groot, 'joint_root')
        if len(res) == 0:
            if root:
                jtype='free'
            else:
                jtype='fixed'
            jaxis=coordinates.Z
            cds_w_j = coordinates(groot.T)
        else:
            if len(res) > 1:
                ##warn
                pass
            res=res[0]
            ###
            jroot=res[0]
            jax=jroot.getChild(0)
            jtype=jax.name ## name of joint_axis
            jtype=jtype.split(':')[1]
            jaxis=coordinates(jax.T).y_axis
            ###
            cds = coordinates(jroot.T)
            if res[1] is None:
                cds_w_j = cds
            else:
                cds_w_j = res[1].copy().transform(cds)
            RobotBuilder.removeNode(groot, jroot)

        if jtype=='fixed':
            jtype=Link.JointType.FixedJoint
        elif jtype=='free':
            jtype=Link.JointType.FreeJoint
        elif jtype=='revolute':
            jtype=Link.JointType.RevoluteJoint
        elif jtype=='prismatic':
            jtype=Link.JointType.PrismaticJoint
        elif jtype=='ball':
            pass
        else:
            jtype=Link.JointType.FreeJoint
        cds_offset = cds_w_j.inverse_transformation()
        ##link.visual <= shapes(org:joint_root)
        groot.setPosition(cds_offset.cnoidPosition)
        if collision is not None:
            l_collision = cutil.SgPosTransform()
            l_collision.addChild(collision)
            l_collision.setPosition(cds_offset.cnoidPosition)
        ##
        if collision is not None and useCollisionForMassparam:
            res = RobotBuilder.traverseSceneGraph(l_collision, excludes=['joint_root', 'COM_root', 'inertia_root', 'joint_axis'])
        else:
            res = RobotBuilder.traverseSceneGraph(groot, excludes=['joint_root', 'COM_root', 'inertia_root', 'joint_axis'])
        ##
        if len(res) < 1:
            print('There is no shape in the scene, rootNode: {}'.format(groot))
        if overwriteMassparam is not None:
            info = overwriteMassparam
        elif mass is not None:
            info= RobotBuilder.mergeResults(res, mass=mass)
        else:
            info= RobotBuilder.mergeResults(res, density=density)
        ##link.axis (joint_axis)
        #print('info: {}'.format(info))
        if jtype=='ball':
            new_kwargs = copy.deepcopy(kwargs)
            namebase=name
            if 'JointName' in new_kwargs:
                namebase = new_kwargs['JointName']
                del new_kwargs['JointName']
            jid = 0
            if 'JointId' in new_kwargs:
                jid = new_kwargs['JointId']
                del new_kwargs['JointId']
            aac=ru.axisAlignedCoords(jaxis, coordinates.Y)
            lname0 = '{}_ballx'.format(namebase)
            lname1 = '{}_bally'.format(namebase)
            lk_0 = self.createLink(name=lname0, JointType=Link.JointType.RevoluteJoint,
                                   mass=0.0, inertia=np.zeros((3,3)), COM=np.zeros(3),
                                   JointAxis=aac.x_axis, JointId=jid, **new_kwargs)
            lk_1 = self.createLink(name=lname1, JointType=Link.JointType.RevoluteJoint,
                                   mass=0.0, inertia=np.zeros((3,3)), COM=np.zeros(3),
                                   JointAxis=aac.y_axis, JointId=jid+1, **new_kwargs)
            ##
            lk=self.createLink(name=name, mass=info['mass'], COM=info['COM'], inertia=info['inertia'],
                               shape=groot, JointType=Link.JointType.RevoluteJoint,
                               JointAxis=aac.z_axis, JointId=jid+2, JointName=namebase, **new_kwargs)
            lk_0.setPosition(cds_w_j.cnoidPosition)
            lk_1.setPosition(cds_w_j.cnoidPosition)
            lk.setPosition(cds_w_j.cnoidPosition)
            if parentLink is not None:
                self.appendLink(parentLink, lk_0)
                self.appendLink(lk_0, lk_1)
                self.appendLink(lk_1, lk)
            elif root:
                ### warning
                self.setRootLink(lk)
            else:
                self.appendLink(self.body.rootLink, lk_0)
                self.appendLink(lk_0, lk_1)
                self.appendLink(lk_1, lk)
            if clear:
                self.clear()
            return lk
        else:
            if collision is None:
                lk=self.createLink(name=name, mass=info['mass'], COM=info['COM'], inertia=info['inertia'],
                                   shape=groot, JointType=jtype, JointAxis=jaxis, **kwargs)
            else:
                lk=self.createLink(name=name, mass=info['mass'], COM=info['COM'], inertia=info['inertia'],
                                   visual=groot, collision=l_collision, JointType=jtype, JointAxis=jaxis, **kwargs)
            ##link.T <= joint_root
            lk.setPosition(cds_w_j.cnoidPosition)
            if parentLink is not None:
                self.appendLink(parentLink, lk)
                if clear:
                    self.clear()
            elif root:
                self.setRootLink(lk)
                if clear:
                    self.clear()
            else:
                self.appendLink(self.body.rootLink, lk)
            return lk

    def viewInfo(self, autoScale=False, **kwargs):
        """Showing information of links. (joint-type, joint-axis, mass, center-of-mass, inertia)
        Args:
            autoScale (boolean, default=False) :
            \*\*kwargs : The other keywords will be passed to irsl_choreonoid.RobotBuilder.RobotBuilder.createVisualizedLinkShape

        Returns:
            list [ SceneGraph ] : Created shapes

        """
        if autoScale:
            ## not implemented
            kwargs['scale']=0.1
        res = []
        for lk in self.body.links:
            vis=self.createVisualizedLinkShape(lk, **kwargs)
            res.append(vis)
            self.addShape(vis)
        return res
### end: GUI wrapper
    def notifyUpdate(self):
        """Notifying update for redrawing robot-model and shapes
        """
        self.body.updateLinkTree()
        self.body.calcForwardKinematics()
        if self.bodyItem is not None:
            self.bodyItem.notifyModelUpdate(sum([int(i) for i in (BodyPlugin.BodyItem.ModelUpdateFlag.LinkSetUpdate, BodyPlugin.BodyItem.ModelUpdateFlag.LinkSpecUpdate,
                                                                  BodyPlugin.BodyItem.ModelUpdateFlag.DeviceSetUpdate, BodyPlugin.BodyItem.ModelUpdateFlag.DeviceSpecUpdate,
                                                                  BodyPlugin.BodyItem.ModelUpdateFlag.ShapeUpdate)]))
            self.bodyItem.notifyKinematicStateUpdate()

    def createJointShape(self, jointType=Link.JointType.FreeJoint, wrapped=True, coords=None, add=True, scale=0.3, **kwargs):
        """Creating and showing jointShape

        Args:
            jointType (cnoid.Body.Link.JointType, default=FixedJoint) :
            wrapped (boolean, default=True) :
            coords (cnoid.IRSLCoords.coordinates, optional) :
            add (boolean, default=True) :
            scale (float, default=0.3) :

        Returns:
            Shape : Created shape

        """
        kwargs['scale']=scale
        tp='fixed'
        if jointType == Link.JointType.FreeJoint:
            sh=self.__freeJointShape(**kwargs)
            tp='free'
        elif jointType == Link.JointType.FixedJoint:
            sh=self.__fixedJointShape(**kwargs)
        elif jointType == Link.JointType.RevoluteJoint:
            sh=self.__revoluteJointShape(**kwargs)
            tp='revolute'
        elif jointType == Link.JointType.PrismaticJoint:
            sh=self.__prismaticJointShape(**kwargs)
            tp='prismatic'
        elif jointType == RobotBuilder.JointType.Ball:
            sh=self.__ballJointShape(**kwargs)
            tp='ball'
        elif jointType == RobotBuilder.JointType.Planar:
            ##sh= not implemented
            tp='planar'
        else:
            pass
        sh.setName('joint_axis:{}'.format(tp))
        trs=cutil.SgPosTransform()
        trs.setName('joint_root')
        trs.addChild(sh)
        if coords is not None:
            trs.setPosition(coords.cnoidPosition)
        if wrapped:
            trs = mkshapes.coordsWrapper(trs)
        if add:
            self.addShape(trs)
        return trs
#OffsetPosition
#JointType
#JointAxis
#JointId
#JointName
#InitialJointAngle
#JointRange
#JointVelocityRange
#JointEffortRange
#EquivalentRotorInertia
    def createLink(self, name='', mass=0.1, COM=None, density=None, inertia=None, shape=None, visual=None, collision=None, **kwargs):
        """Creating a link

        Args:
            name (str, default='') :
            mass (float, default=0.1) :
            COM (numpy.array, optional) :
            density (float, optional) :
            inertia (numpy.array, optional) :
            shape (cnoid.Util.SgNode, optional) :
            visual (cnoid.Util.SgNode, optional) :
            collision (cnoid.Util.SgNode, optional) :
            \*\*kwargs :

        Returns:
            cnoid.Body.Link : Created Link

        """
        lk = self.createLinkBase(name=name, mass=mass, COM=COM, density=density, inertia=inertia, shape=shape, visual=visual, collision=collision)
        for k, v in kwargs.items():
            if hasattr(lk, 'set{}'.format(k)):
                if type(v) is tuple or type(v) is list:
                    exec('lk.set{}(*v)'.format(k))
                else:
                    exec('lk.set{}(v)'.format(k))
            elif hasattr(lk, '{}'.format(k)):
                exec('lk.{} = v'.format(k))
        return lk

    def createRootLink(self, **kwargs):
        """Creating a link and adding it as root-link

        Args:
            \*\*kwargs :

        Returns:
            cnoid.Body.Link : Created Link

        """
        lk=self.createLink(**kwargs)
        lk.setJointType(Link.JointType.FreeJoint)
        self.body.setRootLink(lk)
        self.notifyUpdate()
        return lk

    def createLinkBase(self, name='', mass=0.1, density=None, COM=None, inertia=None, shape=None, visual=None, collision=None):
        baselk = self.body.createLink()
        baselk.setName(name)
        baselk.setMass(mass)
        if COM is None:
            baselk.setCenterOfMass(npa([0., 0., 0.]))
        else:
            baselk.setCenterOfMass(npa(COM))
        if inertia is None:
            baselk.setInertia(npa([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]))
        else:
            baselk.setInertia(npa(inertia))
        if shape is not None:
            ## clone
            if type(shape) is cutil.SgGroup:
                ##shape = cutil.SgGroup(shape)
                shape = shape.clone()
            elif type(shape) is cutil.SgPosTransform:
                ##shape = cutil.SgPosTransform(shape)
                shape = shape.clone()
            else:
                print('Invalid type?? shape: {}, type: {}'.format(shape, type(shape)))
                tmp = cutil.SgGroup()
                tmp.addChild(shape)
                #shape = cutil.SgGroup(tmp)
                shape = tmp.clone()
            baselk.addShapeNode(shape)
        if visual is not None:
            ## clone
            if type(visual) is cutil.SgGroup:
                #visual = cutil.SgGroup(visual)
                visual = visual.clone()
            elif type(visual) is cutil.SgPosTransform:
                #visual = cutil.SgPosTransform(visual)
                visual = visual.clone()
            else:
                print('Invalid type?? visual: {}, type: {}'.format(visual, type(visual)))
                tmp = cutil.SgGroup()
                tmp.addChild(visual)
                #visual = cutil.SgGroup(tmp)
                visual = tmp.clone()
            baselk.addVisualShapeNode(visual)
        if collision is not None:
            ## clone
            if type(collision) is cutil.SgGroup:
                #collision = cutil.SgGroup(collision)
                collision = collision.clone()
            elif type(collision) is cutil.SgPosTransform:
                #collision = cutil.SgPosTransform(collision)
                collision = collision.clone()
            else:
                print('Invalid type?? collision: {}, type: {}'.format(collision, type(collision)))
                tmp = cutil.SgGroup()
                tmp.addChild(collision)
                #collision = cutil.SgGroup(tmp)
                collision = tmp.clone()
            baselk.addCollisionShapeNode(collision)
        self.created_links.append(baselk)
        return baselk

    def updateMassParameterAsUniformDensity(self, alink, density=None, shape='visual'):
        if shape == 'visual':
            sh=alink.visual
        else:
            sh=alink.collision
        res = RobotBuilder.traverseSceneGraph(sh)
        if density is not None:
            info=RobotBuilder.mergeResults(res, density=density)
            alink.setMass(info['mass'])
        else:
            info=RobotBuilder.mergeResults(res, mass=alink.mass)
        alink.setCenterOfMass(info['COM'])
        alink.setInertia(info['inertia'])

    def setRootLink(self, link):
        self.body.setRootLink(link)
        self.notifyUpdate()

    def appendLink(self, link0, link1=None, offset=None):
        if link1 is None:
            link1 = link0
            link0 = self.body.rootLink
        if offset is None:
            p_cds = link0.getCoords()
            c_cds = link1.getCoords()
            offset=p_cds.transformation(c_cds)
        link1.setOffsetPosition(offset.cnoidPosition)
        link0.appendChild(link1)
        self.notifyUpdate()

    def removeLink(self, link):
        ## not implemented yet
        self.notifyUpdate()

    ### start: link visualization
    def __addShape(self, alink, shape):
        sh=alink.shape
        if type(sh) is cutil.SgGroup:
            sh.addChild(shape)
        else:
            cur=cutil.SgGroup()
            cur.setName('visual_root')
            if sh is not None:
                cur.addChild(sh)
            cur.addChild(shape)
            alink.addShapeNode(cur)

    def addCOMShape(self, alink, color=[1,1,0], transparent=0.7):
        mass=alink.mass
        COM=alink.centerOfMass
        ## equivalant radius of sphere of iron with the same mass
        rr = math.pow((mass/8000 * 3)/(4*PI), 1.0/3)
        sh=mkshapes.makeSphere(rr, color=color, transparent=transparent)
        r2=rr*2
        ls=cutil.SgLineSet()
        ls.lineWidth=4
        ls.setVertices(npa([[-r2,0,0],[r2,0,0],[0,-r2,0],[0,r2,0],[0,0,-r2],[0,0,r2]], dtype='float32'))
        ls.addLine(0,1)
        ls.addLine(2,3)
        ls.addLine(4,5)
        ls.setColors(npa([color], dtype='float32'))
        ls.resizeColorIndicesForNumLines(ls.numLines)
        ls.setLineColor(0, 0)
        ls.setLineColor(1, 0)
        ls.setLineColor(2, 0)
        sh.target.addChild(ls)
        sh.translate(COM)
        sh.target.setName('COM_root')
        self.__addShape(alink, sh.target)

    def addInertiaShape(self, alink, color=[1,0,1], transparent=0.5, useBox=False):
        inertia, mat = np.linalg.eig(alink.I)
        mass=alink.mass
        if mass == 0.0:
            mass = 1.0
        rx=mat[:,0]
        ry=mat[:,1]
        rz=mat[:,2]
        if math.fabs(np.dot(rx, ry)) >  0.001:
            ry = np.cross(rz, rx)
        elif math.fabs(np.dot(rx, rz)) >  0.001:
            rz = np.cross(rx, ry)
        rot=np.column_stack([rx, ry, rz])
        if useBox:
            sh=mkshapes.makeBox(1.0, 1.0, 1.0, color=color, transparent=transparent, wrapped=False)
            vscale=npa([  math.pow(6.0/mass * ( -inertia[0] + inertia[1] + inertia[2] ), 0.5),
                          math.pow(6.0/mass * (  inertia[0] - inertia[1] + inertia[2] ), 0.5),
                          math.pow(6.0/mass * (  inertia[0] + inertia[1] - inertia[2] ), 0.5) ])
        else:
            vscale=npa([  math.pow(2.5/mass * ( -inertia[0] + inertia[1] + inertia[2] ), 0.5),
                          math.pow(2.5/mass * (  inertia[0] - inertia[1] + inertia[2] ), 0.5),
                          math.pow(2.5/mass * (  inertia[0] + inertia[1] - inertia[2] ), 0.5) ])
            sh=mkshapes.makeSphere(1.0, color=color, transparent=transparent, wrapped=False)
        scl = cutil.SgScaleTransform(vscale)
        trs = cutil.SgPosTransform()
        scl.addChild(sh)
        trs.addChild(scl)
        cds=coordinates(alink.centerOfMass, rot)
        trs.setPosition(cds.cnoidPosition)
        trs.setName('inertia_root')
        self.__addShape(alink, trs)

    def addJointShape(self, alink, **kwargs):
        jt=alink.jointType
        if jt == Link.JointType.FreeJoint:
            sh=self.__freeJointShape(**kwargs)
        elif jt == Link.JointType.FixedJoint:
            sh=self.__fixedJointShape(**kwargs)
        elif jt == Link.JointType.RevoluteJoint:
            sh=self.__revoluteJointShape(**kwargs)
            ### align y-axis to alink.axis
            v1 = alink.jointAxis / norm(alink.jointAxis)
            v0 = np.cross(v1, coordinates.Y)
            if norm(v0) < 0.5:
                v0 = np.cross(v1, coordinates.Z)
            v0 /= norm(v0)
            v2 = np.cross(v0, v1)
            rot=np.column_stack([v0, v1, v2])
            sh.setPosition(coordinates(rot).cnoidPosition)
        elif jt == Link.JointType.PrismaticJoint:
            sh=self.__prismaticJointShape(**kwargs)
            ### align y-axis to alink.axis
            v1 = alink.jointAxis / norm(alink.jointAxis)
            v0 = np.cross(v1, coordinates.Y)
            if norm(v0) < 0.5:
                v0 = np.cross(v1, coordinates.Z)
            v0 /= norm(v0)
            v2 = np.cross(v0, v1)
            rot=np.column_stack([v0, v1, v2])
            sh.setPosition(coordinates(rot).cnoidPosition)
        else:
            print('JointType: {} is not implemtented yet'.format(jt))
            return
        sh.setName('joint_axis') ## just rotate
        self.__addShape(alink, sh)

    def __revoluteJointShape(self, scale=None, color=[0,1,1], transparent=0.6):
        ## +y-axis ## TODO : rotate direction
        RR = 0.5
        rr = 0.05
        r0 = 0.2
        r1 = 0.3
        l0 = 0.5
        l1 = 0.3
        bd0 = mkshapes.makeTorus(RR-rr, rr, color=color, transparent=transparent)
        bd1 = mkshapes.makeCylinder(r0, l0, color=color, transparent=transparent)
        bd1.translate(npa([0,l0/2,0]))
        bd2 = mkshapes.makeCone(r1, l1, color=color, transparent=transparent)
        bd2.translate(npa([0,l0+l1/2,0]))
        ##
        res=cutil.SgPosTransform()
        if scale is not None:
            current = cutil.SgScaleTransform(scale)
            res.addChild(current)
        else:
            current = res
        current.addChild(bd0.target)
        current.addChild(bd1.target)
        current.addChild(bd2.target)
        return res

    def __prismaticJointShape(self, scale=None, color=[0,1,1], transparent=0.6):
        w0= 0.4
        l0= 0.7
        w1= 0.3
        l1= 0.4
        bd0 = mkshapes.makeBox(w0, l0, w0, color=color, transparent=transparent)
        bd1 = mkshapes.makeBox(w1, l1, w1, color=color, transparent=transparent)
        bd0.translate(npa([0, -l0/2, 0]))
        bd1.translate(npa([0,  l1/2, 0]))
        bd2 = mkshapes.makeCone(w1, w1, color=color, transparent=transparent, DivisionNumber=4)
        bd2.rotate(PI/2, coordinates.Y)
        bd2.translate(npa([0, l1+w1/2, 0]))
        res=cutil.SgPosTransform()
        if scale is not None:
            current = cutil.SgScaleTransform(scale)
            res.addChild(current)
        else:
            current = res
        current.addChild(bd0.target)
        current.addChild(bd1.target)
        current.addChild(bd2.target)
        return res

    def __freeJointShape(self, scale=None, color=None, transparent=0.6):
        R0=0.15
        L0=0.5
        rr=0.25
        ll=0.3
        if color is None:
            col = [0, 1, 0]
        else:
            col = color
        bd0 = mkshapes.makeCylinder(R0, L0, color=col, transparent=transparent)
        bd0.translate(npa([0,L0/2,0]))
        a0 = mkshapes.makeCone(rr, ll, color=col, transparent=transparent)
        a0.translate(npa([0,L0+ll/2,0]))
        ##
        if color is None:
            col = [0, 0, 1]
        else:
            col = color
        bd1 = mkshapes.makeCylinder(R0, L0, color=col, transparent=transparent)
        bd1.rotate(PI/2, coordinates.X)
        bd1.translate(npa([0,L0/2,0]))
        a1 = mkshapes.makeCone(rr, ll, color=col, transparent=transparent)
        a1.rotate(PI/2, coordinates.X)
        a1.translate(npa([0,L0+ll/2,0]))
        ##
        if color is None:
            col = [1, 0, 0]
        else:
            col = color
        bd2 = mkshapes.makeCylinder(R0, L0, color=col, transparent=transparent)
        bd2.rotate(-PI/2, coordinates.Z)
        bd2.translate(npa([0,L0/2,0]))
        a2 = mkshapes.makeCone(rr, ll, color=col, transparent=transparent)
        a2.rotate(-PI/2, coordinates.Z)
        a2.translate(npa([0,L0+ll/2,0]))
        ##
        res=cutil.SgPosTransform()
        if scale is not None:
            current = cutil.SgScaleTransform(scale)
            res.addChild(current)
        else:
            current = res
        current.addChild(bd0.target)
        current.addChild(bd1.target)
        current.addChild(bd2.target)
        current.addChild(a0.target)
        current.addChild(a1.target)
        current.addChild(a2.target)
        return res

    def __fixedJointShape(self, scale=None, color=None, transparent=0.6):
        RR=0.2
        L0=0.7
        if color is None:
            col = [0, 1, 0]
        else:
            col = color
        bd0 = mkshapes.makeBox(RR, L0, RR, color=col, transparent=transparent)
        bd0.translate(npa([0, L0/2, 0]))
        if color is None:
            col = [0, 0, 1]
        else:
            col = color
        bd1 = mkshapes.makeBox(RR, RR, L0, color=col, transparent=transparent)
        bd1.translate(npa([0, 0, L0/2]))
        if color is None:
            col = [1, 0, 0]
        else:
            col = color
        bd2 = mkshapes.makeBox(L0, RR, RR, color=col, transparent=transparent)
        bd2.translate(npa([L0/2, 0, 0]))
        ##
        res=cutil.SgPosTransform()
        if scale is not None:
            current = cutil.SgScaleTransform(scale)
            res.addChild(current)
        else:
            current = res
        current.addChild(bd0.target)
        current.addChild(bd1.target)
        current.addChild(bd2.target)
        return res

    def __ballJointShape(self, scale=None, color=None, transparent=0.6):
        bd0 = mkshapes.makeSphere(0.5, color=color, transparent=transparent)
        res=cutil.SgPosTransform()
        if scale is not None:
            current = cutil.SgScaleTransform(scale)
            res.addChild(current)
        else:
            current = res
        current.addChild(bd0.target)
        return res

    def addDeviceShape(self, alink, scale=1.0):
        ## TODO / not implemented yet ##
        pass

    def createVisualizedLinkShape(self, alink, scale=0.1, wrapped=True, addCOM=True, addInertia=True, addJoint=True, addDevice=True, addToLink=False, noLinkShape=False, useCollision=False, useInertiaBox=False):
        """
        Args:
            alink (cnoid.Body.Link) : Target link to be visualized
            scale (float, default=0.1) : Scale factor of shapes
            wrapped (boolean, default=True) : Returns wrapped object
            addCOM (boolean, default=True) : Add COM(Center Of Mass) shapes
            addInertia (boolean, default=True) : Add inertia-tensor shapes
            addJoint (boolean, default=True) : Add joint-type shapes
            addDevice (boolean, default=True) : Add device-type shapes
            addToLink (boolean, default=False) : Add shapes directly to alink
            noLinkShape : (boolean, default=False) :
            useCollision (boolean, default=False) : Use collision shape as visualized shape
            useInertiaBox (boolean, default=False) : Use box shape for visualizing inertia-tensor (default: ellipsoid)

        Returns:
            Shape : Created shape

        """
        ## SgTransform(linkPos) / SgTransform(shapeBase)
        if addToLink:
            cloned_lk = alink
        else:
            cloned_lk = self.body.createLink(alink)
            if noLinkShape:
                cloned_lk.clearShapeNodes()
            elif useCollision:
                col = cloned_lk.collisionShape
                cloned_lk.clearShapeNodes()
                cloned_lk.addVisualShape(col)
        if addCOM:
            self.addCOMShape(cloned_lk)
        if addInertia:
            self.addInertiaShape(cloned_lk, useBox=useInertiaBox)
        if addJoint:
            self.addJointShape(cloned_lk, scale=scale)
        if addDevice:
            self.addDeviceShape(cloned_lk, scale=scale)
        if addToLink:
            return
        linkorg = cutil.SgPosTransform()
        linkorg.setName('linkPos')
        sh = cloned_lk.shape
        if sh is not None:
            linkorg.addChild(sh)
        if wrapped:
            res = mkshapes.coordsWrapper(linkorg)
            res.newcoords(cloned_lk.getCoords())
        else:
            res = linkorg
            linkorg.setPosition(cloned_lk.T)
        return res
    ### end: link visualization
    def exportBody(self, fname, modelName=None, mode=0, **kwargs):
        """Exporting created robot-model to file as .body file

        Args:
            fname (str) : Name of file
            modelName (str, optional) :
            mode (int, default=0) :  0:EmbedModels, 1:LinkToOriginalModelFiles, 2:ReplaceWithStdSceneFiles, 3:ReplaceWithObjModelFiles
            kwargs (dict) :

        """
        if modelName is not None:
            self.body.setName(modelName)
        return iu.exportBody(fname, self.body, extModelFileMode=mode, **kwargs)

    def exportURDF(self, fname, **kwargs):
        """Exporting created robot-model to file as .urdf file

        Args:
            fname (str) : Name of file
            kwargs (dict) :

        """
        return iu.exportURDF(fname, self.body, **kwargs)

    ### start: mkshapes wrapper
    def loadScene(self, fname, wrapped=True, add=True, **kwargs):
        """Loading scene as a shape, see `irsl_choreonoid.make_shapes.loadScene <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.loadScene>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.loadScene(fname, wrapped=wrapped, **kwargs)
        if add:
            self.addShape(res)
        return res
    def loadMesh(self, fname, wrapped=True, add=True, **kwargs):
        """Loading mesh as a shape, see `irsl_choreonoid.make_shapes.loadMesh <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.loadMesh>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.loadMesh(fname, wrapped=wrapped, **kwargs)
        if add:
            self.addShape(res)
        return res
    def makeBox(self, x, y = None, z = None, wrapped=True, add=True, **kwargs):
        """Making box shape, see `irsl_choreonoid.make_shapes.makeBox <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.makeBox>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.makeBox(x,y,z,wrapped,**kwargs)
        if add:
            self.addShape(res)
        return res
    def makeCylinder(self, radius, height, wrapped=True, add=True, **kwargs):
        """Making cylinder shape, see `irsl_choreonoid.make_shapes.makeCylinder <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.makeCylinder>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.makeCylinder(radius, height, wrapped, **kwargs)
        if add:
            self.addShape(res)
        return res
    def makeSphere(self, radius, wrapped=True, add=True, **kwargs):
        """Making sphere shape, see `irsl_choreonoid.make_shapes.makeSphere <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.makeSphere>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.makeSphere(radius, wrapped, **kwargs)
        if add:
            self.addShape(res)
        return res
    def makeCone(self, radius, height, wrapped=True, add=True, **kwargs):
        """Making cone shape, see `irsl_choreonoid.make_shapes.makeCone <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.makeCone>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.makeCone(radius, height, wrapped, **kwargs)
        if add:
            self.addShape(res)
        return res
    def makeCapsule(self, radius, height, wrapped=True, add=True, **kwargs):
        """Making capsule shape, see `irsl_choreonoid.make_shapes.makeCapsule <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.makeCapsule>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.makeCapsule(radius, height, wrapped, **kwargs)
        if add:
            self.addShape(res)
        return res
    def makeTorus(self, radius, corssSectionRadius, beginAngle = None, endAngle = None, wrapped=True, add=True, **kwargs):
        """Making torus shape, see `irsl_choreonoid.make_shapes.makeTorus <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.makeTorus>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.makeTorus(radius, corssSectionRadius, beginAngle, endAngle, wrapped, **kwargs)
        if add:
            self.addShape(res)
        return res
    def makeExtrusion(self, crossSection, spine, wrapped=True, add=True, **kwargs):
        """Making extrusion shape, see `irsl_choreonoid.make_shapes.makeExtrusion <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.makeExtrusion>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.makeExtrusion(crossSection, spine, wrapped, **kwargs)
        if add:
            self.addShape(res)
        return res
    def makeElevationGrid(self, xDimension, zDimension, xSpacing, zSpacing, height, wrapped=True, add=True, **kwargs):
        """Making elevation-grid shape, see `irsl_choreonoid.make_shapes.makeElevationGrid <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.makeElevationGrid>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.makeElevationGrid(xDimension, zDimension, xSpacing, zSpacing, height, wrapped, **kwargs)
        if add:
            self.addShape(res)
        return res
    def make3DAxis(self, coords=None, add=True, **kwargs):
        """Making 3D-axis shape, see `irsl_choreonoid.make_shapes.make3DAxis <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.make3DAxis>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.make3DAxis(coords, **kwargs)
        if add:
            self.addShape(res)
        return res
    def make3DAxisBox(self, coords=None, add=True, **kwargs):
        """Making 3D-axis shape (type: box), see `irsl_choreonoid.make_shapes.make3DAxisBox <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.make3DAxisBox>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.make3DAxisBox(coords, **kwargs)
        if add:
            self.addShape(res)
        return res
    def makeCoords(self, coords=None, add=True, **kwargs):
        """Making 3D-axis shape (type: line), see `irsl_choreonoid.make_shapes.makeCoords <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.makeCoords>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.makeCoords(coords, **kwargs)
        if add:
            self.addShape(res)
        return res
    def makeCross(self, coords=None, add=True, **kwargs):
        """Making 3D-axis shape (type: crossing-line), see `irsl_choreonoid.make_shapes.makeCross <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.makeCross>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.makeCross(coords, **kwargs)
        if add:
            self.addShape(res)
        return res

    def makeLineAlignedShape(self, start, end, add=True, **kwargs):
        """Making shape which is aligned to designated line, see `irsl_choreonoid.make_shapes.makeLineAlignedShape <./module_irsl_choreonoid.html#irsl_choreonoid.make_shapes.makeLineAlignedShape>`_

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.makeLineAlignedShape(start, end, module=self, **kwargs)
        if add:
            self.addShape(res)
        return res

    ### end: mkshapes wrapper
    class MassParam(object):
        def __init__(self, shape, coords): ## make volume, com, inertia from boundingbox
            ## shape is primitive or mesh(wo primitive)
            self.volume = 0.0
            self.mass  = 0.0
            self.COM   = npa([0, 0, 0])
            self.unit_inertia = npa([[0, 0, 0],[0, 0, 0],[0, 0, 0]])
            self.density = None
            self.coords = coords
            self.shape  = shape
            if hasattr(shape, 'volume'):
                self.volume = shape.volume
            if hasattr(shape, 'COM'):
                self.COM = shape.COM
            if hasattr(shape, 'inertia'):
                self.unit_inertia = shape.inertia
            else:
                res_ = cnoid.IRSLUtil.calcMassProperties(shape)
                self.volume = res_[0]
                self.COM = npa([res_[1], res_[2], res_[3]])
                self.unit_inertia = npa([[res_[4], res_[7], res_[9]],
                                         [res_[7], res_[5], res_[8]],
                                         [res_[9], res_[8], res_[6]]]) / self.volume
        def setDensity(self, density=1000.0):
            if self.volume != 0.0:
                self.mass = density * self.volume
                self.density = density
        def setMass(self, mass):
            self.mass = mass
            if self.volume != 0.0:
                self.density = mass/self.volume
        @property
        def Inertia(self):
            return self.mass * self.unit_inertia

    @staticmethod
    def removeNode(gnode, target):
        if gnode.isGroupNode():
            for idx in range(gnode.numChildren):
                ch = gnode.getChild(idx)
                if ch is target:
                    gnode.removeChildAt(idx, False)
                    return True
                if RobotBuilder.removeNode(ch, target):
                    return True
        return False

    @staticmethod
    def searchSceneGraph(gnode, name, currentCoords=None):
        res = []
        if gnode.name==name:
            res += [(gnode, currentCoords)]
        if gnode.isGroupNode():
            if isinstance(gnode, cutil.SgPosTransform):
                if currentCoords is None:
                    currentCoords = coordinates(gnode.T)
                else:
                    currentCoords = currentCoords.copy().transform(coordinates(gnode.T))
            for idx in range(gnode.numChildren):
                ch = gnode.getChild(idx)
                res += RobotBuilder.searchSceneGraph(ch, name, currentCoords)
        return res

    @staticmethod
    def traverseSceneGraph(gnode, currentCoords=None, use_primitive=True, excludes=[]):
        ## extract shape and coords
        ## print('call {} / {}'.format(gnode, currentCoords))
        res = []
        if gnode.name in excludes:
            return res
        if gnode.isGroupNode():
            if isinstance(gnode, cutil.SgPosTransform):
                if currentCoords is None:
                    currentCoords = coordinates(gnode.T)
                else:
                    currentCoords = currentCoords.copy().transform(coordinates(gnode.T))
            for idx in range(gnode.numChildren):
                ch = gnode.getChild(idx)
                res += RobotBuilder.traverseSceneGraph(ch, currentCoords, use_primitive=use_primitive)
        elif isinstance(gnode, cutil.SgShape):
            if use_primitive:
                m = gnode.mesh
                if m is not None:
                    pri = m.primitive
                    if pri is None:
                        pri = m
                res = [ RobotBuilder.MassParam(pri, currentCoords) ]
            else:
                ## not enter here
                res = [(gnode, currentCoords)]
        return res

    @staticmethod
    def mergeResults(result, newcoords=coordinates(), mass=None, density=None, debug=False):
        if mass is None and density is None:
            raise Exception('set weight or density')
        ##
        total_volume = 0.0
        for masspara in result:
            total_volume += masspara.volume
        ##
        total_mass = 0.0
        for masspara in result:
            if mass is not None:
                if total_volume == 0.0:
                    masspara.setMass=mass
                else:
                    masspara.setMass(mass * masspara.volume/total_volume)
            else:
                masspara.setDensity(density)
            total_mass += masspara.mass
        if debug:
            for masspara in result:
                print('{}, {} at {}, {}'.format(masspara.coords,
                                                masspara.mass,
                                                masspara.COM,
                                                masspara.Inertia))
        ##
        newCOM_w = npa([0., 0., 0.])
        for masspara in result:
            newCOM_w += masspara.mass * masspara.coords.transform_vector(masspara.COM)
        if total_mass != 0.0:
            newCOM_w /= total_mass
        ##
        newIner_w = npa([[0., 0., 0.],[0., 0., 0.],[0., 0., 0.]])
        for masspara in result:
            #print("new: {}, org: {}".format(newCOM_w, masspara.coords.transform_vector(masspara.COM)))
            #print("       , org: {} on {}".format(masspara.coords, masspara.COM))
            trans = newCOM_w - masspara.coords.transform_vector(masspara.COM)
            #print("trans: {}".format(trans))
            x = trans[0]
            y = trans[1]
            z = trans[2]
            i_trans = masspara.mass * npa([[y*y + z*z, -x*y, -x*z],
                                           [-x*y, x*x + z*z, -y*z],
                                           [-x*z, -y*z, x*x + y*y]])
            rot = masspara.coords.rot
            i_rot = rot.dot(masspara.Inertia).dot(np.transpose(rot))
            #print("i_trans: {}, i_rot: {}".format(i_trans, i_rot))
            newIner_w += (i_trans + i_rot)
            #print("newIner: {}".format(newIner_w))
        res = {}
        res['mass'] = total_mass
        res['volume'] = total_volume
        res['COM'] = newCOM_w
        res['inertia'] = newIner_w
        return res

class SimpleRobotBuilder(RobotBuilder):
    '''
    Examples:
        >>> link_params = [
            {'point': [0, 0, 0.2], 'type': 'yaw_revolute', 'axis': 'Z'},
            {'point': [0, 0, 0.4], 'type': 'revolute', 'axis': 'X'},
            {'point': [0, 0, 0.6], 'type': 'revolute', 'axis': 'Y'},
            {'point': [0, 0, 0.6 + 1.4], 'type': 'revolute', 'axis': 'Y'},
            {'point': [0, 0, 0.6 + 1.4 + 1.4], 'type': 'revolute', 'axis': 'Y'},
            {'point': [0, 0, 0.6 + 1.4 + 1.4 + 0.2], 'type': 'revolute', 'axis': 'X'},
            {'point': [0, 0, 0.6 + 1.4 + 1.4 + 0.4], 'type': 'yaw_revolute', 'axis': 'Z'},
            ]

        >>> g_root = {'primitive': 'cylinder',
           'parameter': {'radius': 1.5, 'height': 0.4, 'color': [1.0, 1.0, 0], 'DivisionNumber': 6},
           'rotation': [1, 0, 0, PI/2],
           'translation': [0, 0, 0.0]}

        >>> g_tip = [
            {'primitive': 'box',
             'parameter': {'x': 0.1, 'y': 0.1, 'z': 0.3, 'color': [0, 1.0, 0]},
             'translation': [0, 0, 0.15 ]},
            {'primitive': 'box',
             'parameter': {'x': 0.15, 'y': 0.15, 'z': 0.4, 'color': [0, 1.0, 1.0]},
             'translation': [0,  0.15, 0.3 + 0.15],
             'rotation': [1, 0, 0, -0.6]},
            {'primitive': 'box',
             'parameter': {'x': 0.15, 'y': 0.15, 'z': 0.4, 'color': [0, 1.0, 1.0]},
             'translation': [0, -0.15, 0.3 + 0.15],
             'rotation': [1, 0, 0, 0.6]},
            {'primitive': 'box',
             'parameter': {'x': 0.15, 'y': 0.185, 'z': 0.3, 'color': [0, 1.0, 1.0]},
             'translation': [0,  0.22,  0.3 + 0.40],
             },
            {'primitive': 'box',
             'parameter': {'x': 0.15, 'y': 0.185, 'z': 0.3, 'color': [0, 1.0, 1.0]},
             'translation': [0,  -0.22, 0.3 + 0.40],
             },
            ]

        >>> sp = SimpleRobotBuilder()

        >>> sp.buildSimpleRobot(link_params, rootGeometry=g_root, tipGeometry=g_tip)

    Note:
        definition of link_params
        [ { 'point': [], 'type': str, 'axis': str }, ... ]

        point: absolute world position, list of 3 float values,  [X, Y, Z]
        type: type of the joint, 'revolute', 'yaw_revolute'
        axis: direction of joint-axis, 'X', 'Y', or 'Z'
    '''
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.default_density = 1000
        self.default_joint_range = [-PI, PI]
        self.default_velocity_range = [-PI*10, PI*10]
        self.default_effort_range = [-100, 100]
        self.cur_lk = None
        self.cur_jid = None
        self.links = []
        self.scale = 1.0
        self.link_color  = [0, 1.0, 0]
        self.joint_color = [1.0, 0, 0]
        ### TODO : adding link/joint name
        ## link_prefix
        ## joint_prefix
    def _makePreJoint(self, cur_coords, cur_type, cur_axis):
        if   cur_type == 'revolute':
            thick_  = 0.25 * self.scale
            radius_ = 0.10 * self.scale
        elif cur_type == 'yaw_revolute':
            thick_  = 0.05 * self.scale
            radius_ = 0.25 * self.scale
        ##
        cyl=self.makeCylinder(radius_, thick_, color=self.joint_color)
        cyl.newcoords(cur_coords.copy())
        mv_ = npa([0, -thick_/2, 0])
        if   cur_axis == 'X':
            cyl.rotate(-PI/2, coordinates.Z).translate(mv_)
        elif cur_axis == 'Y':
            cyl                             .translate(mv_)
        elif cur_axis == 'Z':
            cyl.rotate( PI/2, coordinates.X).translate(mv_)
    #
    def _makeJoint(self, cur_coords, cur_type, cur_axis):
        j  = self.createJointShape(jointType=Link.JointType.RevoluteJoint)
        j.newcoords(cur_coords.copy())
        if cur_axis == 'X':
            j.rotate(-PI/2, coordinates.Z)
        elif cur_axis == 'Z':
            j.rotate( PI/2, coordinates.X)
        lk = self.createLinkFromShape(name='LINK{}'.format(self.cur_jid),
                                    JointName='JOINT{}'.format(self.cur_jid),
                                    parentLink=self.cur_lk,
                                    JointId=self.cur_jid,
                                    density=self.default_density,
                                    JointRange=self.default_joint_range,
                                    JointVelocityRange=self.default_velocity_range,
                                    JointEffortRange=self.default_effort_range,
                                    EquivalentRotorInertia=0.1)
        self.cur_lk = lk
        self.cur_jid += 1
    #
    def _makePostJoint(self, cur_coords, cur_type, cur_axis):
        if   cur_type == 'revolute':
            thick_  = 0.25 * self.scale
            radius_ = 0.10 * self.scale
        elif cur_type == 'yaw_revolute':
            thick_  = 0.05 * self.scale
            radius_ = 0.25 * self.scale
        cyl=self.makeCylinder(radius_, thick_, color=self.joint_color)
        cyl.newcoords(cur_coords.copy())
        mv_ = npa([0, thick_/2, 0])
        if   cur_axis == 'X':
            cyl.rotate(-PI/2, coordinates.Z).translate(mv_)
        elif cur_axis == 'Y':
            cyl                             .translate(mv_)
        elif cur_axis == 'Z':
            cyl.rotate( PI/2, coordinates.X).translate(mv_)
    #
    def makeInitialLink(self, lst, name='Root', rootGeometry=None):
        ## root-geometry
        if rootGeometry is not None:
            if type(rootGeometry) is list:
                for g in rootGeometry:
                    self.makeShapeFromParam(**g)
            else:
                self.makeShapeFromParam(**rootGeometry)
        #
        c_cds_  = coordinates(lst[0]['point'])
        c_type_ = lst[0]['type']
        c_axis_ = lst[0]['axis']
        self._makePreJoint(c_cds_, c_type_, c_axis_) ## cur-joint
        lk_ = self.createLinkFromShape(name=name, root=True, density=self.default_density)
        self.cur_lk  = lk_
        self.cur_jid = 0
    #
    def makeLinks(self, lst, tipGeometry=None):
        c_cds_  = coordinates(lst[0]['point'])
        c_type_ = lst[0]['type']
        c_axis_ = lst[0]['axis']
        ##
        for l in lst[1:]:
            n_cds_  = coordinates(l['point'])
            n_type_ = l['type']
            n_axis_ = l['axis']
            #
            self._makePreJoint(n_cds_, n_type_, n_axis_) ## next-joint
            ## to-next-geometry
            self.makeLineAlignedShape(c_cds_.pos, n_cds_.pos, size=0.1*self.scale, color=self.link_color)
            self._makePostJoint(c_cds_, c_type_, c_axis_) ## cur-joint
            self._makeJoint(c_cds_, c_type_, c_axis_) ## next-joint
            #
            c_cds_  = n_cds_
            c_type_ = n_type_
            c_axis_ = n_axis_
        ## tip-geometry
        if tipGeometry is not None:
            glst = []
            if type(tipGeometry) is list:
                for g in tipGeometry:
                    glst.append(self.makeShapeFromParam(**g))
            else:
                glst.append(self.makeShapeFromParam(**tipGeometry))
            for g in glst:
                g.transform(c_cds_, coordinates.wrt.world)
        else:
            ### default tip
            tip = self.makeLineAlignedShape(npa([0,0,0.]), npa([0, 0, 1.0]), size=0.1*self.scale, color=self.link_color)
            tip.transform(c_cds_, coordinates.wrt.world)
        #
        self._makePostJoint(c_cds_, c_type_, c_axis_) ## cur-joint
        self._makeJoint(c_cds_, c_type_, c_axis_) ## next-joint
    #
    def makeShapeFromParam(self, primitive=None, parameter=None, translation=None, rotation=None):
        mk = {}
        if translation is not None:
            mk['translation'] = translation
        if rotation is not None:
            mk['rotation'] = rotation
        #
        if len(mk) > 0:
            coords = ru.make_coordinates(mk)
        else:
            coords = coordinates()
        #
        func = None
        if primitive == 'box':
            func = self.makeBox
        elif primitive == 'cylinder':
            func = self.makeCylinder
        elif primitive == 'sphere':
            func = self.makeCylinder
        elif primitive == 'cone':
            func = self.makeCone
        elif primitive == 'capsule':
            func = self.makeCapsule
        elif primitive == 'torus':
            func = self.makeTorus
        elif primitive == 'tetrahedron':
            func = self.makeTetrahedron
        elif primitive == 'extrusion':
            func = self.makeExtrusion
        #
        if func is not None:
            ret = func(**parameter)
            ret.newcoords(coords)
            return ret
    #
    def buildSimpleRobot(self, link_param_list, name='Root', rootGeometry=None, tipGeometry=None):
        self.makeInitialLink(link_param_list, name=name, rootGeometry=rootGeometry)
        self.makeLinks(link_param_list, tipGeometry=tipGeometry)
