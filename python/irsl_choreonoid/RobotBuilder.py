## cnoid.Util
import cnoid.Util as cutil
## cnoid.Body
import cnoid.Body as cbody
from cnoid.Body import Body
from cnoid.Body import Link
from cnoid.Body import Device

## IRSL (not base)
from cnoid.IRSLCoords import coordinates
#import cnoid.IRSLCoords as IC
from .draw_coords import GeneralDrawInterfaceWrapped as DrawInterface
#-from irsl_choreonoid.draw_coords import GeneralDrawInterfaceWrapped as DrawInterface
## from .draw_coords import DrawCoordsListWrapped as DrawCoords
from . import make_shapes as mkshapes
from . import cnoid_util as iu
#-import irsl_choreonoid.make_shapes as mkshapes
#-import irsl_choreonoid.cnoid_util as iu
## from . import robot_util as ru
## from .robot_util import RobotModelWrapped as RobotModel
## etc
import numpy as np
from numpy import array as npa
from numpy.linalg import norm
import math
from math import pi as PI
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

## add inertia on shape
class RobotBuilder(object):
    """Building robot interactively
    """
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
        if gui and iu.isInChoreonoid():
            ## set self.__di
            self.__di=DrawInterface()
            self.__setBodyItem(robot=robot, name=name)
        ## set self.__body
        self.__setBody(robot=robot)
        self.created_links = []

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
        if self.__di is not None:
            cbase.ItemTreeView.instance.checkItem(self.bodyItem, False)

    def showRobot(self):
        """Showing robot model (check RobotItem)
        """
        if self.__di is not None:
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

    def removeShape(self, shape):
        """Removing a shape

        Args:
            shape (cnoid.Util.SgNode) : Shape to be removed

        """
        if self.__di is not None:
            self.__di.removeObject(shape)

    def createLinkFromShape(self, name=None, mass=None, density=1000.0, parentLink=None, root=False, clear=True, **kwargs):
        """Creating link from drawn shapes and appending to the other link

        Args:
            name (str, optional) :
            mass (float, optional) :
            density (float, default=1000.0) :
            parentLink (cnoid.Body.Link, optional) :
            root (boolean, default=False) :
            clear (boolean, default=True) :
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
        groot=cutil.SgPosTransform(self.__di.SgPosTransform)
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
            jtype=jax.name
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
        else:
            jtype=Link.JointType.FreeJoint
        cds_offset = cds_w_j.inverse_transformation()
        ##link.visual <= shapes(org:joint_root)
        groot.setPosition(cds_offset.cnoidPosition)
        res = RobotBuilder.traverseSceneGraph(groot, excludes=['joint_root', 'COM_root', 'inertia_root', 'joint_axis'])
        #print('res: {}'.format(res))
        if len(res) < 1:
            print('There is no shape in the scene, rootNode: {}'.format(groot))
        if mass is not None:
            info= RobotBuilder.mergeResults(res, mass=mass)
        else:
            info= RobotBuilder.mergeResults(res, density=density)
        ##link.axis (joint_axis)
        #print('info: {}'.format(info))
        lk=self.createLink(name=name, mass=info['mass'], COM=info['COM'], inertia=info['inertia'],
                           shape=groot, JointType=jtype, JointAxis=jaxis, **kwargs)
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
            \*\*kwargs :

        Returns:
            [] : Created shapes

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
            jointType (cnoid.Body.Link.JointType, default=FreeJoint) :
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
        else:
            pass
        sh.setName('joint_axis:{}'.format(tp)) ## just rotate
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
            shape (,optional) :
            visual (,optional) :
            collision (,optional) :
            \*\*kwargs :

        Returns:
            cnoid.Body.Link : Created Link

        """
        lk = self.createLinkBase(name=name, mass=mass, COM=COM, density=density, inertia=inertia, shape=shape, visual=visual, collision=collision)
        for k, v in kwargs.items():
            if type(v) is tuple or type(v) is list:
                exec('lk.set{}(*v)'.format(k))
            else:
                exec('lk.set{}(v)'.format(k))
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
                shape = cutil.SgGroup(shape)
            elif type(shape) is cutil.SgPosTransform:
                shape = cutil.SgPosTransform(shape)
            else:
                print('Invalid type?? shape: {}, type: {}'.format(shape, type(shape)))
                tmp = cutil.SgGroup()
                tmp.addChild(shape)
                shape = cutil.SgGroup(tmp)
            baselk.addShapeNode(shape)
        if visual is not None:
            ## clone
            if type(visual) is cutil.SgGroup:
                visual = cutil.SgGroup(visual)
            elif type(visual) is cutil.SgPosTransform:
                visual = cutil.SgPosTransform(visual)
            else:
                print('Invalid type?? visual: {}, type: {}'.format(visual, type(visual)))
                tmp = cutil.SgGroup()
                tmp.addChild(visual)
                visual = cutil.SgGroup(tmp)
            baselk.addVisualShapeNode(visual)
        if collision is not None:
            ## clone
            if type(collision) is cutil.SgGroup:
                collision = cutil.SgGroup(collision)
            elif type(collision) is cutil.SgPosTransform:
                collision = cutil.SgPosTransform(collision)
            else:
                print('Invalid type?? collision: {}, type: {}'.format(collision, type(collision)))
                tmp = cutil.SgGroup()
                tmp.addChild(collision)
                collision = cutil.SgGroup(tmp)
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

    def addDeviceShape(self, alink, scale=1.0):
        ## TODO / not implemented yet ##
        pass

    def createVisualizedLinkShape(self, alink, scale=0.1, wrapped=True, addCOM=True, addInertia=True, addJoint=True, addDevice=True, addToLink=False, useCollision=False, useInertiaBox=False):
        """
        Args:
            alink (cnoid.Body.Link) :
            scale (float, default=0.1) :
            wrapped (boolean, default=True) :
            addCOM (boolean, default=True) :
            addInertia (boolean, default=True) :
            addJoint (boolean, default=True) :
            addDevice (boolean, default=True) :
            addToLink (boolean, default=False) :
            useCollision (boolean, default=False) :
            useInertiaBox (boolean, default=False) :

        Returns:
            Shape : Created shape

        """
        ## SgTransform(linkPos) / SgTransform(shapeBase)
        if addToLink:
            cloneed_lk = alink
        else:
            cloned_lk = self.body.createLink(alink)
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
    def writeBodyFile(self, fname, modelName=None, mode=0, **kwargs):
        """Writing created robot-model to file as .body file

        Args:
            fname (str) : Name of file
            modelName (str, optional) :
            mode (int, default=0):

        """
        #mode: 0:EmbedModels, 1:LinkToOriginalModelFiles, 2:ReplaceWithStdSceneFiles, 3:ReplaceWithObjModelFiles
        bw = cbody.StdBodyWriter()
        bw.setMessageSinkStdErr()
        bw.setExtModelFileMode(mode)
        if modelName is not None:
            self.body.setName(modelName)
        return bw.writeBody(self.body, fname)
    def writeURDF(self, fname, **kwargs):
        """Writing created robot-model to file as .urdf file

        Args:
            fname (str) : Name of file
            modelName (str, optional) :
            mode (int, default=0):

        """
        ubw = URDFBodyWriter()
        ubw.setMessageSinkStdErr()
        for k, v in kwargs.items():
            exec('ubw.set{}(v)'.format(k))
        ubw.writeBody(self.body, fname)
    ### start: mkshapes wrapper
    def loadScene(self, fname, wrapped=True, add=True, **kwargs):
        """Loading scene as a shape, see irsl_choreonoid.make_shapes.loadScene

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.loadScene(fname, wrapped, **kwargs)
        if add:
            self.addShape(res)
        return res
    def loadMesh(self, fname, wrapped=True, add=True, **kwargs):
        """Loading mesh as a shape, see irsl_choreonoid.make_shapes.loadMesh

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.loadMesh(fname, wrapped, **kwargs)
        if add:
            self.addShape(res)
        return res
    def makeBox(self, x, y = None, z = None, wrapped=True, add=True, **kwargs):
        """Making box shape, see irsl_choreonoid.make_shapes.makeBox

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
        """Making cylinder shape, see irsl_choreonoid.make_shapes.makeCylinder

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
        """Making sphere shape, see irsl_choreonoid.make_shapes.makeSphere

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
        """Making cone shape, see irsl_choreonoid.make_shapes.makeCone

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
        """Making capsule shape, see irsl_choreonoid.make_shapes.makeCapsule

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
        """Making torus shape, see irsl_choreonoid.make_shapes.makeTorus

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
        """Making extrusion shape, see irsl_choreonoid.make_shapes.makeExtrusion

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
        """Making elevation-grid shape, see irsl_choreonoid.make_shapes.makeElevationGrid

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
        """Making 3D-axis shape, see irsl_choreonoid.make_shapes.make3DAxis

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
        """Making 3D-axis shape (type: box), see irsl_choreonoid.make_shapes.make3DAxisBox

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
        """Making 3D-axis shape (type: line), see irsl_choreonoid.make_shapes.makeCoords

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
        """Making 3D-axis shape (type: crossing-line), see irsl_choreonoid.make_shapes.makeCross

        Args:
            add (boolean, default=True) :

        Returns:
            Shape : Created shape

        """
        res = mkshapes.makeCross(coords, **kwargs)
        if add:
            self.addShape(res)
        return res
    ### end: mkshapes wrapper
    class MassParam(object):
        def __init__(self, shape, coords):
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
        def setDensity(self, density=1000.0):
            if self.volume != 0.0:
                self.mass = density * self.volume
                self.density = density
        def setMass(self, mass):
            self.mass = mass
            if self.volume != 0.0:
                self.density = self.volume/mass
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
                if RobotBuilder.revemoNode(ch, target):
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
                res = [(gnode, currentCoords)]
        return res

    @staticmethod
    def mergeResults(result, newcoords=coordinates(), mass=None, density=None):
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
                masspara.setMass(mass * masspara.volume/total_volume)
            else:
                masspara.setDensity(density)
            total_mass += masspara.mass
        ##
        newCOM_w = npa([0., 0., 0.])
        for masspara in result:
            newCOM_w += masspara.mass * masspara.coords.transform_vector(masspara.COM)
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
            i_rot = np.transpose(rot).dot(masspara.Inertia).dot(rot)
            #print("i_trans: {}, i_rot: {}".format(i_trans, i_rot))
            newIner_w += (i_trans + i_rot)
            #print("newIner: {}".format(newIner_w))
        res = {}
        res['mass'] = total_mass
        res['volume'] = total_volume
        res['COM'] = newCOM_w
        res['inertia'] = newIner_w
        return res
