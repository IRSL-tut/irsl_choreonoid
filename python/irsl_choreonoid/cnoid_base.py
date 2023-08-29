from cnoid.Base import ItemTreeView
from cnoid.Base import MessageView
from cnoid.Base import SceneView
from cnoid.Base import SceneWidget
from cnoid.Base import ProjectManager
from cnoid.Base import RootItem

from cnoid.BodyPlugin import AISTSimulatorItem
from cnoid.BodyPlugin import BodyItem
from cnoid.BodyPlugin import WorldItem

from cnoid.IRSLCoords import coordinates
from numpy import array as npa

from .robot_util import make_coordinates
from .robot_util import make_coords_map

from cnoid.Body import Body

from cnoid.Util import SgCamera
## DEPRECATED: use cnoid.Base.ItemTreeView.instance
#def flushRobotView(name):
#    #findItem(name).notifyKinematicStateChange()
#    findItem(name).notifyKinematicStateUpdate()
#    #MessageView.getInstance().flush()
#    MessageView.instance.flush()

def loadProject(project_file):
    """Loading project file (currend project may be changed)

    Args:
        project_file (str): filename of project file (.cnoid)

    """
    ProjectManager.instance.loadProject(filename=project_file)

##
## cnoid Item (Base)
##
def getItemTreeView():
    """DEPRECATED: use cnoid.Base.ItemTreeView.instance
    """
    return ItemTreeView.instance
    if callable(ItemTreeView.instance):
        return ItemTreeView.instance()
    else:
        return ItemTreeView.instance

def getRootItem():
    """DEPRECATED: use cnoid.Base.RootItem.instance
    """
    return RootItem.instance
    if callable(RootItem.instance):
        return RootItem.instance()
    else:
        return RootItem.instance

def getOrAddWorld(name = 'World'):
    """Getting or creating WorldItem

    Args:
        name (str, default = 'World') : name of WorldItem

    Returns:
        cnoid.Base.WorldItem : added or found WorldItem

    """
    rI = RootItem.instance
    ret = rI.findItem(name)
    if ret == None:
        ret = WorldItem()
        ret.setName(name)
        rI.addChildItem(ret)
        ItemTreeView.instance.checkItem(ret)
    return ret

def addSimulator(world = None, simulator_name = 'AISTSimulator'):
    """Adding SimulatorItem

    Args:
        world (optional) : world item, simulator should be created under this world
        simulator_name (str, default = 'AISTSimulator') :  name of SimulatorItem to be added

    Returns:
        cnoid.BodyPlugin.AISTSimulatorItem : Added SimulatorItem

    """
    if world is None:
        world = getOrAddWorld()
    sim_ = world.findItem(simulator_name)
    if sim_ == None:
        sim_ = AISTSimulatorItem()
        world.addChildItem(sim_)
        ItemTreeView.instance.checkItem(sim_)
    return sim_

def loadRobotItem(fname, name = None, world = True, addItem = True):
    """Load robot model and add it as a BodyItem

    Args:
        fname (str): filename (or path)
        name (str, optional): name of loaded robot-model
        world (boolean or WorldItem, default = True): if True, WorldItem is added
        addItem (boolean): if True and world if False, loaded item is added to RootItem
    Returns:
        cnoid.BodyPlugin.BodyItem : Loaded robot-model

    """
    # print('loadRobot: %s'%(fname))
    bI = BodyItem()
    if not bI.load(str(fname)):
        return None
    if name:
        bI.setName(name)
    rr = bI.body
    rr.updateLinkTree()
    rr.initializePosition()
    rr.calcForwardKinematics()
    bI.storeInitialState()
    if world == True:
        wd = getOrAddWorld()
        wd.insertChildItem(bI, wd.childItem)
        ItemTreeView.instance.checkItem(bI)
    elif type(world) is WorldItem:
        world.insertChildItem(bI, world.childItem)
        ItemTreeView.instance.checkItem(bI)
    elif addItem:
        RootItem.instance.addChildItem(bI)
        ItemTreeView.instance.checkItem(bI)
    return bI

def findItem(name):
    """Finding item in ItemTreeView

    Args:
        name (str) : name of item to be searched

    Returns:
        cnoid.Base.Item : found item (first one)

    """
    return RootItem.instance.findItem(name)

def findItems(name):
    """Finding item in ItemTreeView

    Args:
        name (str) : name of item to be searched

    Returns:
        list [ cnoid.Base.Item ] : all found items which has the name

    """
    return [ itm for itm in RootItem.instance.getDescendantItems() if itm.name == name ]

def findItemsByQuery(query):
    """Finding item which is query returning True in ItemTreeView

    Args:
        query (callable, taking 1 argument, argtype cnoid.Base.Item) : Query function which rturns true if the item should be extracted

    Returns:
        list [ cnoid.Base.Item ] : all found items that query returns True

    """
    return [ itm for itm in RootItem.instance.getDescendantItems() if func(itm) ]

def findItemsByName(name):
    """Finding item with the same name in ItemTreeView

    Args:
        name (str) : name of item to be searched

    Returns:
        list [ cnoid.Base.Item ] : all found items which has the name

    """
    return findItemsByQuery( lambda itm : (itm.name == name) )

def findItemsByClass(cls):
    """Finding item with class given as the argument in ItemTreeView

    Args:
        cls (class) : class of item to be searched

    Returns:
        list [ cnoid.Base.Item ] : all found items which has the name

    """
    return findItemsByQuery( lambda itm : (type(itm) == cls) )

def removeItem(item_):
    """Removing item

    Args:
        item_ (cnoid.Base.Item) : item to be removed

    """
    item_.detachFromParentItem()

def findBodyItem(name_or_body):
    """Seaching BodyItem

    Args:
        name_or_body (str or cnoid.Body.Body) : If type is str, searching BodyItem with the same name. If type is body, searching BodyItem which has identical body

    Returns:
        cnoid.Base.Item : found BodyItem

    """
    ret = None
    if type(name_or_body) is str:
        for itm in findItems(name_or_body):
            if type(itm) is cnoid.BodyPlugin.BodyItem:
                ret = itm
                break
    elif type(name_or_body) is Body:
        for itm in RootItem.instance.getDescendantItems():
            if type(itm) is BodyItem and itm.body == name_or_body:
                ret = itm
                break
    return ret

def findRobot(name):
    """DEPRECATED: use findBodyItem
    """
    ret = findBodyItem(name)
    if ret is None:
        return None
    ## add class check...
    if callable(ret.body):
        return ret.body()
    else:
        return ret.body

#
# Utility for Scene
#
def cameraPositionLookingFor(eye, direction, up):
    """Generating camera coordinates where designated direction is the camera's optical axis

    Args:
        eye (numpy.array or list[float]) : 3D position of the camera
        direction (numpy.array or list[float]) : Direction of the camera's optical axis
        up (numpy.array or list[float]) : Up direction

    Returns:
        cnoid.IRSLCoords.coordinates : Camera coordinates

    Note:
        Refer cnoid.Util.SgCamera.positionLookingFor

    """
    return coordinates(SgCamera.positionLookingFor(npa(eye), npa(direction), npa(up)))

def cameraPositionLookingAt(eye, center, up):
    """Generating camera coordinates looking at the point

    Args:
        eye (numpy.array or list[float]) : 3D position of the camera
        center (numpy.array or list[float]) : 3D position wherer the camera's optical axis passes through
        up (numpy.array or list[float]) : Up direction

    Returns:
        cnoid.IRSLCoords.coordinates : Camera coordinates

    Note:
        Refer cnoid.Util.SgCamera.positionLookingAt

    """
    return coordinates(SgCamera.positionLookingAt(npa(eye), npa(center), npa(up)))

def saveImageOfScene(filename):
    """Saving scene as image file

    Args:
        filename (str) : Name of the file to be saved

    Note:
        Refer cnoid.Base.SceneWidget.saveImage

    """
    sw = currentSceneWidget()
    sw.saveImage(filename)

def currentSceneView():
    """Returning the instance of SceneView

    Args:
        None

    Returns:
        cnoid.Base.SceneView : Current Instance of SceneView ( return cnoid.Base.SceneView.instance )

    """
    return SceneView.instance

def currentSceneWidget():
    """Returning the instance of SceneWidget

    Args:
        None

    Returns:
        cnoid.Base.SceneWidget : Current Instance of SceneWidget ( return cnoid.Base.SceneView.instance.sceneWidget )

    """
    return SceneView.instance.sceneWidget

def getCameraCoords(withFOV=True):
    """Returning camera position of current scene

    Args:
        withFOV (boolean, default = True ) : If true, getting camera's field of view

    Returns:
        (cnoid.IRSLCoords.coordinates, float) : Tuple with camera coordinates and camera's filed of view

    """
    sw = currentSceneWidget()
    cds = coordinates(sw.builtinCameraTransform.T)
    if withFOV:
        fov = sw.builtinPerspectiveCamera.fieldOfView
        return (cds, fov)
    else:
        return (cds, None)

def setCameraCoords(cds, fov=None):
    """Setting camera coordinates of current scene

    Args:
        cds (cnoid.IRSLCoords.coordinates) : Camera's coordinates to be set
        fov (float, default = None) : Camera's filed of view to be set

    """
    sw = currentSceneWidget()
    sw.builtinCameraTransform.setPosition(cds.cnoidPosition)
    if fov is not None:
        sw.builtinPerspectiveCamera.setFieldOfView(fov)
    sw.builtinPerspectiveCamera.notifyUpdate()

def getCameraCoordsParam(withFOV=True):
    """Returning camera position of current scene (returning dictionary type)

    Args:
        withFOV (boolean, default = True ) : If true, getting camera's field of view

    Returns:
        dict[str, param] : Dictionary can be read with irsl_choreonoid.robot_util.make_coordinates, and added the keyword 'fov'

    """
    cds, fov = getCameraCoords(withFOV)
    _ret = make_coords_map(cds)
    _ret['fov'] = fov
    return _ret

def setCameraCoordsParam(param_dict):
    """Setting camera coordinates of current scene (receiving dictionary type)

    Args:
        param_dict (dict[str, param]) : Dictionary can be read with irsl_choreonoid.robot_util.make_coordinates, and added the keyword 'fov'

    """
    fov = None
    if 'fov' in param_dict:
        fov = param_dict['fov']
    cds = make_coordinates(param_dict)
    setCameraCoords(cds, fov)

def setBackgroundColor(backgound_color):
    """Setting color of background

    Args:
        background_color (numpy.array or list[float]) : Vector with 3 elements. Eech element represents 'Red', 'Green', 'Blue'.

    """
    currentSceneWidget().setBackgroundColor(npa(backgound_color))

def disableGrid(plane = None):
    """Disabling to show grids

    Args:
        plane (int, optional) : ID of plane to be disabled. 0\: XY, 1\: XZ, 2\: YZ

    """
    sw = currentSceneWidget()
    if type(plane) == int:
        sw.setGridEnabled(SceneWidget.GridPlane(plane), False)
    else:
        sw.setGridEnabled(SceneWidget.GridPlane.XY_Grid, False)
        sw.setGridEnabled(SceneWidget.GridPlane.YZ_Grid, False)
        sw.setGridEnabled(SceneWidget.GridPlane.XZ_Grid, False)
    sw.updateGrids()

def enableGrid(plane = 0):
    """SHowing grids

    Args:
        plane (int, default = 0) : ID of plane to show. 0\: XY, 1\: XZ, 2\: YZ

    """
    sw = currentSceneWidget()
    sw.setGridEnabled(SceneWidget.GridPlane(plane), True)
    sw.updateGrids()
