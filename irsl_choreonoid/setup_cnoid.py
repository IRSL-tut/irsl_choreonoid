### for development version
from cnoid.Base import RootItem
from cnoid.Base import ItemTreeView

from cnoid.BodyPlugin import AISTSimulatorItem
from cnoid.GLVisionSimulatorPlugin import GLVisionSimulatorItem
from cnoid.BodyPlugin import BodyItem
from cnoid.BodyPlugin import WorldItem
from cnoid.BodyPlugin import SimulationBar

# from cnoid.PythonSimScriptPlugin import PythonSimScriptItem

##
import cnoid.Body as cbody

from cnoid.IRSLCoords import coordinates

from .cnoid_util import parseURL
from .robot_util import make_coordinates

import irsl_choreonoid.cnoid_base as ib
import irsl_choreonoid.make_shapes as mkshapes

import yaml
# import math
# import sys

#### specification of yaml
## robot:
##   model: @robot_file_name@
##   name: MyRobot
##   initial_joint_angles: []
##   initial_coords: {} ## using make_coordinates from irsl_choreonoid.robot_util
##   BodyROSItem: ## should be launch from choreonoid_ros
##       joint_state_publication: false
##       joint_state_update_rate: 100
##       name_space: arm_robot3
##   ROSControlItem: ## should be launch from choreonoid_ros
##       name_space: arm_robot3
## robots:
##   - { robot: }
## object:
##   model: @object_file_name@
##   name: MyObject
##   initial_joint_angles: []
##   initial_coords: {} ## using make_coordinates from irsl_choreonoid.robot_util
##   fixed: True ##
## objects:
##   - { object: }
## world:
##   World:
##     name:
##     draw_grid: False
##   Simulator:
##     type: 'AISTSimulator'
##   GLVision:
##   Camera:
##     lookEye:
##     lookForDirection:
##     lookAtCenter:
##     lookAtUp:
##     position: { pos: [], aa: [] }
##     fov:
##   WorldROS: ## should be launch from choreonoid_ros
##   ROS:
##     urdf_settings:
##          file: @file_name@
##          robotName:
##          transmission:
##          name:
##     set_parameter: 
##          - type: 'yaml'
##            file: @file_name@
##            name: 
##          - type: 'param'
##            name: 
##            parameter: {}
##          - parameter: {}
##     generate_settings:
##        robot: @robot_file_name@
##        controllers: [ {name: '', type: '', joints: [] } ]
## pythonScript:
####

param_method_dict = {
#    'param_name':'method_name'
#    'param_name':'variable_name='
    'name_space': 'nameSpace=',
    'max_clock_publishing_rate': 'maxClockPublishingRate=',
    'joint_state_update_rate': 'jointStateUpdateRate=',
    'joint_state_publication': 'jointStatePublication=',
    }
def _applyParameter(item, param):
    if type(param) is not dict:
        return
    for key,val in param.items():
        #print("{}/{}".format(key, val))
        eval_str = ''
        if key in param_method_dict:
            method = param_method_dict[key]
            #print("method: {}".format(method))
            if method[-1] == '=':
                if hasattr(item, method[:-1]):
                    eval_str = 'item.' + method + 'val'
            else:
                if hasattr(item, method):
                    eval_str = 'item.' + method + '(val)'
        else:
            method = 'set' + ''.join([ s.capitalize() for s in key.split('_') ])
            if hasattr(item, method):
                eval_str = 'item.' + method + '(val)'
            elif hasattr(item, 'set' + key):
                eval_str = 'item.set' + key + '(val)'
            elif hasattr(item, key):
                eval_str = 'item.' + key + '(val)'

        if len(eval_str) > 0:
            print('eval: {} / val={}'.format(eval_str, val)) ## debug
            exec(eval_str, locals(), globals())

def _getDictValue(in_dict, keys, default=None):
    if in_dict is None:
        return default
    for k in keys:
        if k in in_dict:
            return in_dict[k]
    return default

def _getDictValueExist(in_dict, keys):
    if in_dict is None:
        return (False, None)
    for k in keys:
        if k in in_dict:
            return (True, in_dict[k])
    return (False, None)

def _splitFiles(fname):
    res = fname.split(';')
    if len(res) > 1:
        return res
    return None

class _BodyItemWrapper(object):
    def __init__(self, world, parseURL, offset=None):
        self.body_item = None
        self.world_item = world
        self.offset = offset
        self._parseURL = parseURL

    def addObject(self, info, fix=False):
        model_ = _getDictValue(info, ('model', 'Model', 'file', 'File', 'body', 'Body', 'model_file', 'modelFile', 'uri', 'URI'))
        if model_ is None:
            return self.addPrimitive(info)## always fixed
        return self.addRobot(info, fix=fix)

    def addRobot(self, info, fix=False):
        self.body_item = BodyItem()
        model_ = _getDictValue(info, ('model', 'Model', 'file', 'File', 'body', 'Body', 'model_file', 'modelFile', 'uri', 'URI'))
        if model_ is None:
            return
        fname = self._parseURL(model_)
        self.body_item.load(fname)
        ##
        name_ = _getDictValue(info, ('name', 'Name', 'modelName'))
        if name_ is not None:
            self.body_item.setName(name_)
        ##
        self.body_item.body.updateLinkTree()
        self.body_item.body.initializePosition()
        angles_ = _getDictValue(info, ('initial_joint_angles', 'joint_angles', 'initial_angles', 'jointAngles', 'initialAngles',
                                       'initial_joint_angle', 'joint_angle', 'initial_angle', 'jointAngle', 'initialAngle',
                                       'initialJointAngles', 'initialJointAngle'))
        if angles_ is not None:
            for j, q in zip(self.body_item.body.joints, angles_):
                j.q = q
        cds_dict_ =  _getDictValue(info, ('initial_coords', 'initial_position', 'Coords', 'Position', 'initialCoords', 'initialPosition'))
        if cds_dict_ is not None:
            cds = make_coordinates(cds_dict_)
            if self.offset is not None:
                cds.transform(self.offset, coordinates.wrt.world)
            self.body_item.body.rootLink.setPosition(cds.cnoidPosition)
        elif self.offset is not None:
            self.body_item.body.rootLink.setPosition(self.offset.cnoidPosition)
        self.body_item.body.calcForwardKinematics()
        self.body_item.storeInitialState()
        ##
        fix_ = _getDictValue(info, ('fix', 'Fix', 'fixedObject', 'fixed', 'Fixed'))
        if fix_ is not None:
            fix = fix_
        if fix: ## fix is overwrittern by info
            self.body_item.body.setRootLinkFixed(True);
        ##
        self.world_item.insertChildItem(self.body_item, self.world_item.childItem)
        ##
        check = True
        if 'no_check' in info and info['no_check']:
            check = False
        else:
            ItemTreeView.instance.checkItem(self.body_item)

    def _addPrimitiveWithShape(self, shape, info):
        bitem = BodyItem()
        nm_ = _getDictValue(info, ('name', 'Name', 'NAME'))
        if nm_ is not None:
            bitem.setName(nm_)
        baselk = bitem.body.createLink()
        baselk.addShapeNode(shape)
        cds = make_coordinates(info)
        baselk.setPosition(cds.cnoidPosition)
        baselk.setJointType(cbody.Link.JointType.FixedJoint)
        bitem.body.setRootLink(baselk)
        ##
        bitem.body.updateLinkTree()
        bitem.body.calcForwardKinematics()
        ##
        self.world_item.insertChildItem(bitem, self.world_item.childItem)
        ItemTreeView.instance.checkItem(bitem)
        return bitem

    def addPrimitive(self, info, fix=True):
        model_ = _getDictValue(info, ('box', 'Box', 'BOX'))
        if model_ is not None:
            shape_ = mkshapes.makeBox(*model_, rawShape=True, **info)
            self._addPrimitiveWithShape(shape_, info)
            return
        model_ = _getDictValue(info, ('cylinder', 'Cylinder', 'CYLINDER'))
        if model_ is not None:
            shape_ = mkshapes.makeCylinder(*model_, rawShape=True, **info)
            self._addPrimitiveWithShape(shape_, info)
            return
        model_ = _getDictValue(info, ('sphere', 'Sphere', 'SPHERE'))
        if model_ is not None:
            shape_ = mkshapes.makeSphere(*model_, rawShape=True, **info)
            self._addPrimitiveWithShape(shape_, info)
            return
        model_ = _getDictValue(info, ('cone', 'Cone', 'CONE'))
        if model_ is not None:
            shape_ = mkshapes.makeCone(*model_, rawShape=True, **info)
            self._addPrimitiveWithShape(shape_, info)
            return
        model_ = _getDictValue(info, ('capsule', 'Capsule', 'CAPSULE'))
        if model_ is not None:
            shape_ = mkshapes.makeCapsule(*model_, rawShape=True, **info)
            self._addPrimitiveWithShape(shape_, info)
            return
        model_ = _getDictValue(info, ('torus', 'Torus', 'TORUS'))
        if model_ is not None:
            shape_ = mkshapes.makeTorus(*model_, rawShape=True, **info)
            self._addPrimitiveWithShape(shape_, info)
            return

class SetupCnoid(object):
    """
    Utility class for setting .cnoid file from python script
    """
    def __init__(self, rootItem=None, worldItem=None):
        """
        Args:
            rootItem (cnoid.Base.Item, optional) : If set, it is used as root for creating environment
            worldItem (cnoid.Base.WorldItem, optional) : If set, it is used as a worldItem
        """
        self.world_item = worldItem
        if rootItem:
            self.root_item = rootItem
        else:
            self.root_item = RootItem.instance
        #self.robots =  []
        #self.objects = []
        self.simulator = None
        self.simulatorRobot = None
        self._parseURL = parseURL

    def buildEnvironment(self, info_dict, world='World', createWorld=False, setCamera=False, offset=None):
        """
        Building environment (setting objects) under the WorldItem

        Args:
            info_dict ( dict['key': value] ) : Dictionary for representing objects on environment
            world (str, default='World') : Name of WorldItem, added objects under this item
            craeteWorld (boolean, default=False) : If True, creating new WorldItem
            setCamera (boolean, default=False) : If True, set camera position
            offset (cnoid.IRSLCoords.coordinates) : Offset of objects
        """
        if type(info_dict) is list:
            for obj_info in info_dict:
                self._addObject(obj_info, worldItem=worldItem, offset=offset)
            return

        if type(info_dict) is not dict:
            raise Exception('type of {} is not dict'.format(info_dict))

        worldItem = None
        if createWorld:
            if 'world' in info_dict:
                world_info = info_dict['world']
                ## World
                exist_, world_ = _getDictValueExist(world_info, ('World', 'world', 'WORLD'))
                if exist_:
                    self._addWorld(param=world_)
            else:
                self._addWorld(name=world)
            worldItem = self.world_item
        else:
            worldItem = self.root_item.findItem(world)
        ##
        if worldItem is None:
            if self.world_item:
                worldItem = self.world_item
            else:
                worldItem = self.root_item
        ##
        if setCamera:
            if 'world' in info_dict:
                world_info = info_dict['world']
                camera_ = _getDictValue(world_info, ('Camera', 'camera', 'View', 'view'))
                if camera_ is not None:
                    self._setCameraPosition(param=camera_)
        ##
        if 'object' in info_dict:
            self._addObject(info_dict['object'], worldItem=worldItem, offset=offset)
        if 'objects' in info_dict:
            for obj_info in info_dict['objects']:
                self._addObject(obj_info, worldItem=worldItem, offset=offset)
            # notify

    def addExtraWorld(self, world_info):
        """
        abstract method
        """
        pass

    def createCnoid(self, info_dict, addDefaultSimulator=True, addDefaultWorld=True, noEnvironment=False):
        """
        Creating project from parameters

        Args:
            info_dict ( dict['key': value] ) : Dictionary for representing the project
            addDefaultSimulator (boolean, default=True) : If True, adding new SimulatorItem if there is no instruction in info_dict
            addDefaultWorld (boolean, default=True) : If True, adding new WorldItem if there is no instruction in info_dict
            noEnvironment (boolean, default=False) : If True, not adding environment(objects). Use buildEnvironment method.

        """
        if type(info_dict) is not dict:
            raise Exception('type of {} is not dict'.format(info_dict))

        ### parse world first
        if 'world' in info_dict:
            world_info = info_dict['world']
            ## World
            exist_, world_ = _getDictValueExist(world_info, ('World', 'world', 'WORLD'))
            if exist_:
                self._addWorld(param=world_)
            else:
                self._addWorld()

            #### parse extra
            self.addExtraWorld(world_info)

            ## Simulator
            exist_, simulator_ = _getDictValueExist(world_info, ('Simulator', 'simulator', 'SimulatorItem'))
            if exist_:
                self._addSimulator(param=simulator_)
            ## GLVision
            exist_, gl_vision_ = _getDictValueExist(world_info, ('GLVision', 'gl_vision', 'Vision', 'vision'))
            if exist_:
                self._addGLVision(param=gl_vision_)
            ## Camera
            camera_ = _getDictValue(world_info, ('Camera', 'camera', 'View', 'view'))
            if camera_ is not None:
                self._setCameraPosition(param=camera_)
        else:
            ## add default_world
            if addDefaultWorld:
                self._addWorld()
            if addDefaultSimulator:
                self._addSimulator()

        if 'robot' in info_dict:
            self._addRobot(info_dict['robot'])
        if 'robots' in info_dict:
            for rb_info in info_dict['robots']:
                self._addRobot(rb_info)

        if not noEnvironment:
            if 'object' in info_dict:
                self._addObject(info_dict['object'])
            if 'objects' in info_dict:
                for obj_info in info_dict['objects']:
                    self._addObject(obj_info)
            # notify
    def buildEnvironmentFromYaml(self, yamlFile, **kwargs):
        """
        Building environment from yaml-file

        Args:
            yamlFile (str) : File name to load
            kwargs (dict) : Keyword to pass to setup_cnoid.buildEnvironment

        """
        files = _splitFiles(yamlFile)
        if files is None:
            fname = self._parseURL(yamlFile)
            info_ = yaml.safe_load(open(fname))
            self.buildEnvironment(info_, **kwargs)
        else:
            fname = self._parseURL(files[0])
            info_ = yaml.safe_load(open(fname))
            self.buildEnvironment(info_, **kwargs)
            kwargs['createWorld'] = False
            kwargs['setCamera'] = False
            for f in files[1:]:
                fname = self._parseURL(f)
                info_ = yaml.safe_load(open(fname))
                self.buildEnvironment(info_, **kwargs)

    def createCnoidFromYaml(self, yamlFile, **kwargs):
        """
        Creating project from yaml-file

        Args:
            yamlFile (str) : File name to load
            kwargs (dict) : Keyword to pass to setup_cnoid.createCnoid

        """
        files = _splitFiles(yamlFile)
        if files is None:
            fname = self._parseURL(yamlFile)
            info_ = yaml.safe_load(open(fname))
            self.createCnoid(info_, **kwargs)
        else:
            fname = self._parseURL(files[0])
            info_ = yaml.safe_load(open(fname))
            self.createCnoid(info_, **kwargs)
            kwargs['createWorld'] = False
            kwargs['setCamera'] = False
            for f in files[1:]:
                fname = self._parseURL(f)
                info_ = yaml.safe_load(open(fname))
                self.buildEnvironment(info_, **kwargs)

    @classmethod
    def setEnvironmentFromYaml(cls, yamlFile, **kwargs):
        """
        Setup environment from yaml-file (classmethod of buildEnvironmentFromYaml)

        Args:
            yamlFile (str) : File name to load
            kwargs (dict) : Keyword to pass to setup_cnoid.buildEnvironment

        Returns:
            irsl_choreonoid_ros.setup_cnoid : Instance of setup_cnoid

        """
        cnoid = cls()
        cnoid.buildEnvironmentFromYaml(yamlFile, **kwargs)
        return cnoid

    @classmethod
    def setCnoidFromYaml(cls, yamlFile, **kwargs):
        """
        Setup project from yaml-file (classmethod of createCnoidFromYaml)

        Args:
            yamlFile (str) : File name to load
            kwargs (dict) : Keyword to pass to setup_cnoid.createCnoid

        Returns:
            irsl_choreonoid_ros.setup_cnoid : Instance of setup_cnoid

        """
        cnoid = cls()
        cnoid.createCnoidFromYaml(yamlFile, **kwargs)
        return cnoid
    @classmethod
    def setupEnvironment(cls, info, **kwargs):
        """
        Building environment (setting objects) under the WorldItem (class method)

        Args:
            info_dict ( dict['key': value] ) : Dictionary for representing objects on environment
            world (str, default='World') : Name of WorldItem, added objects under this item
            craeteWorld (boolean, default=False) : If True, creating new WorldItem
            setCamera (boolean, default=False) : If True, set camera position
            offset (cnoid.IRSLCoords.coordinates) : Offset of objects
        """
        cnoid = cls()
        cnoid.buildEnvironment(info, **kwargs)
        return cnoid
    @classmethod
    def setupCnoid(cls, info, **kwargs):
        """
        Creating project from parameters (class method)

        Args:
            info_dict ( dict['key': value] ) : Dictionary for representing the project
            addDefaultSimulator (boolean, default=True) : If True, adding new SimulatorItem if there is no instruction in info_dict
            addDefaultWorld (boolean, default=True) : If True, adding new WorldItem if there is no instruction in info_dict
            noEnvironment (boolean, default=False) : If True, not adding environment(objects). Use buildEnvironment method.

        """
        cnoid = cls()
        cnoid.createCnoid(info, **kwargs)
        return cnoid
    def startSimulator(self, realTime=None):
        """
        Starting simulation

        Args:
            realTime (boolean, default=None) : If True, simulator will run with realtime sync mode

        """
        if self.simulator is not None:
            if realTime is not None:
                self.simulator.setRealtimeSyncMode(realTime)
            ItemTreeView.instance.checkItem(self.simulator)
            ItemTreeView.instance.selectItem(self.simulator)
            # self.simulator.startSimulation(True)
            SimulationBar.instance.startSimulation(True)## doRest=True

    def _addWorld(self, name='World', check=True, param=None):
        wd = self.root_item.findItem(name)
        if wd is None:
            if self.world_item is None:
                self.world_item = WorldItem()
                self.world_item.name = name
                _applyParameter(self.world_item, param)
                self.root_item.addChildItem(self.world_item)
        else:
            self.world_item = wd
        ##
        grid_ = _getDictValue(param, ('draw_grid', 'DrawGrid', 'grid', 'Grid'))
        if grid_ is not None:
            if grid_ is False:
                ib.disableGrid()
            elif grid_ == 'XY':
                ib.disableGrid()
                ib.enableGrid(0)
            elif grid_ == 'XZ':
                ib.disableGrid()
                ib.enableGrid(1)
            elif grid_ == 'YZ':
                ib.disableGrid()
                ib.enableGrid(2)
            elif type(grid_) is list or type(grid_) is tuple:
                ib.disableGrid()
                for g_ in grid_:
                    if g_ == 'XY':
                        ib.enableGrid(0)
                    elif g_ == 'XZ':
                        ib.enableGrid(1)
                    elif g_ == 'YZ':
                        ib.enableGrid(2)
        ##
        if check:
            ItemTreeView.instance.checkItem(self.world_item)

    def _getWrapper(self, worldItem=None, offset=None):
        if worldItem is not None:
            bi = _BodyItemWrapper(worldItem, parseURL=self._parseURL, offset=offset)
        else:
            bi = _BodyItemWrapper(self.world_item, parseURL=self._parseURL, offset=offset)
        return bi

    def _addRobot(self, info=None, worldItem=None, offset=None):
        bi = self._getWrapper(worldItem=worldItem, offset=offset)
        bi.addRobot(info)

    def _addObject(self, info=None, worldItem=None, offset=None):
        bi = self._getWrapper(worldItem=worldItem, offset=offset)
        bi.addObject(info)

    def _addSimulator(self, check=True, param=None): ## name is overwritten by param
        if 'type' in param:
            exec('self.simulator = {}Item()'.format(param['type']), locals(), globals())
        if self.simulator is not None:
            ## set integrationMode: runge-kutta
            self.world_item.addChildItem(self.simulator)## ? insert
            _applyParameter(self.simulator, param)
            if check:
                ItemTreeView.instance.checkItem(self.simulator)

    def _addGLVision(self, simulator=None, target_bodies=None, target_sensors=None, param=None):
        if self.simulator is None:
            return
        vsim = GLVisionSimulatorItem()
        if param is not None and 'target_bodies' in param:
            vsim.setTargetBodies(param['target_bodies'])
        elif target_bodies is not None:
            vsim.setTargetBodies(target_bodies)
        if param is not None and 'target_sensors' in param:
            vsim.setTargetSensors(param['target_sensors'])
        elif target_sensors is not None:
            vsim.setTargetSensors(target_sensors)
        ## default parameters
        vsim.setMaxFrameRate(1000)
        vsim.setMaxLatency(0)
        vsim.setVisionDataRecordingEnabled(False)
        ## vsim.setThreadEnabled(True)
        vsim.setDedicatedSensorThreadsEnabled(True)
        vsim.setBestEffortMode(True)
        vsim.setRangeSensorPrecisionRatio(2.0)
        #vsim.setAllSceneObjectsEnabled(False)
        vsim.setAllSceneObjectsEnabled(True)
        vsim.setHeadLightEnabled(True)
        vsim.setAdditionalLightsEnabled(True)
        ##
        _applyParameter(vsim, param)
        #add
        self.simulator.addChildItem(vsim)
        self.glvision = vsim

    def _setCameraPosition(self, param=None):
        if param is None:
            return
        fov_ = _getDictValue(param, ('fov', 'FOV'))
        cds_ = _getDictValue(param, ('position', 'Position', 'coords', 'coordinates'))
        if cds_ is not None:
            cds_ = make_coordinates(cds_)
            ib.setCameraCoords(cds_, fov_)
        else:
            eye_ = _getDictValue(param, ('lookEye', 'eye', 'Eye'))
            up_ = _getDictValue(param, ('lookUp', 'up', 'Up'))
            if eye_ is not None and up_ is not None:
                dir_ = _getDictValue(param, ('lookForDirection', 'lookDirection', 'Direction', 'direction'))
                if dir_ is not None:
                    cds = ib.cameraPositionLookingFor(eye_, dir_, up_, opencv=False)
                    ib.setCameraCoords(cds, fov_, opencv=False)
                else:
                    center_ = _getDictValue(param, ('lookAtCenter', 'lookAt', 'at', 'center'))
                    if center_ is not None:
                        cds = ib.cameraPositionLookingAt(eye_, center_, up_, opencv=False)
                        ib.setCameraCoords(cds, fov_, opencv=False)

    def _parseURDF(self, param):
        pass

    def _addScriptItem(self, param):
        script = PythonSimScriptItem()
        filename = _getDictValue(param, ('file', 'file_name', 'filename', 'File', 'FileName', 'script', 'Script'))
        if filename is not None:
            script.load(filename)
            # script.setExecutionTiming(SimulationScriptItem.ExecutionTiming.AFTER_INITIALIZATION)
            # script.setBackgroundMode(False)
            _applyParameter(script, param)
            self.world_item.addChildItem(script)## ? insert
        itemTreeView.checkItem(script)
