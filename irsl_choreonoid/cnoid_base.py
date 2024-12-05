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

import math

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
    return [ itm for itm in RootItem.instance.getDescendantItems() if query(itm) ]

def findItemsByName(name):
    """Finding item with the same name in ItemTreeView

    Args:
        name (str) : name of item to be searched

    Returns:
        list [ cnoid.Base.Item ] : all found items which has the name

    """
    return findItemsByQuery( lambda itm : (itm.name == name) )

def findItemsByClassExact(cls):
    """Finding item with class given as the argument in ItemTreeView

    Args:
        cls (class) : class of item to be searched

    Returns:
        list [ cnoid.Base.Item ] : all found items which has the name

    """
    return findItemsByQuery( lambda itm : (type(itm) == cls) )

def findItemsByClass(cls):
    """Finding item with class given as the argument in ItemTreeView

    Args:
        cls (class) : class of item to be searched

    Returns:
        list [ cnoid.Base.Item ] : all found items which has the name

    """
    return findItemsByQuery( lambda itm : isinstance(itm, cls) )

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
def cameraPositionLookingFor(eye, direction, up, opencv=True):
    """Generating camera coordinates where designated direction is the camera's optical axis

    Args:
        eye (numpy.array or list[float]) : 3D position of the camera
        direction (numpy.array or list[float]) : Direction of the camera's optical axis
        up (numpy.array or list[float]) : Up direction
        opencv (boolean, default = True) : OpenCV stype coordinates will be returned

    Returns:
        cnoid.IRSLCoords.coordinates : Camera coordinates

    Note:
        Refer cnoid.Util.SgCamera.positionLookingFor

    """
    res = coordinates(SgCamera.positionLookingFor(npa(eye), npa(direction), npa(up)))
    if opencv:
        res.rotate(math.pi, coordinates.X)
    return res

def cameraPositionLookingAt(eye, center, up, opencv=True):
    """Generating camera coordinates looking at the point

    Args:
        eye (numpy.array or list[float]) : 3D position of the camera
        center (numpy.array or list[float]) : 3D position wherer the camera's optical axis passes through
        up (numpy.array or list[float]) : Up direction
        opencv (boolean, default = True) : OpenCV stype coordinates will be returned

    Returns:
        cnoid.IRSLCoords.coordinates : Camera coordinates

    Note:
        Refer cnoid.Util.SgCamera.positionLookingAt

    """
    res = coordinates(SgCamera.positionLookingAt(npa(eye), npa(center), npa(up)))
    if opencv:
        res.rotate(math.pi, coordinates.X)
    return res

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

def getCameraCoords(withFOV=True, opencv=True):
    """Returning camera position of current scene

    Args:
        withFOV (boolean, default = True ) : If true, getting camera's field of view
        opencv (boolean, default = True) : OpenCV stype coordinates will be returned
    Returns:
        (cnoid.IRSLCoords.coordinates, float) : Tuple with camera coordinates and camera's filed of view

    """
    sw = currentSceneWidget()
    cds = coordinates(sw.builtinCameraTransform.T)
    if opencv:
        cds.rotate(math.pi, coordinates.X)
    if withFOV:
        fov = sw.builtinPerspectiveCamera.fieldOfView
        return (cds, fov)
    else:
        return (cds, None)

def setCameraCoords(cds, fov=None, update=True, opencv=True):
    """Setting camera coordinates of current scene

    Args:
        cds (cnoid.IRSLCoords.coordinates) : Camera's coordinates to be set
        fov (float, default = None) : Camera's filed of view to be set
        update (boolean, default=True) : If True, rendering scene with updated camera position
        opencv (boolean, default=True) : cds is OpenCV stype coordinates

    """
    sw = currentSceneWidget()
    if opencv:
        cds = cds.copy().rotate(math.pi, coordinates.X)
    sw.builtinCameraTransform.setPosition(cds.cnoidPosition)
    if fov is not None:
        sw.builtinPerspectiveCamera.setFieldOfView(fov)
    if update:
        sw.builtinPerspectiveCamera.notifyUpdate()

def getCameraCoordsParam(withFOV=True, opencv=True):
    """Returning camera position of current scene (returning dictionary type)

    Args:
        withFOV (boolean, default = True ) : If true, getting camera's field of view

    Returns:
        dict[str, param] : Dictionary can be read with irsl_choreonoid.robot_util.make_coordinates, and added the keyword 'fov'

    """
    cds, fov = getCameraCoords(withFOV, opencv=opencv)
    _ret = make_coords_map(cds)
    _ret['fov'] = fov
    return _ret

def setCameraCoordsParam(param_dict, opencv=True):
    """Setting camera coordinates of current scene (receiving dictionary type)

    Args:
        param_dict (dict[str, param]) : Dictionary can be read with irsl_choreonoid.robot_util.make_coordinates, and added the keyword 'fov'

    """
    fov = None
    if 'fov' in param_dict:
        fov = param_dict['fov']
    cds = make_coordinates(param_dict)
    setCameraCoords(cds, fov, opencv=opencv)

def getCameraMatrix():
    """Getting camera matrix of this scene

    Retuns:
        numpy.array (3x3 matrix) : Camera Matrix

    """
    sw = currentSceneWidget()
    ## cam_cds_ = coordinates(sw.builtinCameraTransform.T)
    fov_ = sw.builtinPerspectiveCamera.fieldOfView
    width_  = sw.width()
    height_ = sw.height()
    cx_ = 0.5*width_
    cy_ = 0.5*height_
    if height_ < width_:
        ff_  = 0.5*height_ / math.tan(fov_/2)
    else:
        ff_  = 0.5*width_ / math.tan(fov_/2)
    return npa([[ff_, 0, cx_], [0, ff_, cy_], [0, 0, 1]], dtype='float64')

def setCameraMatrix(camera_matrix, width=None, height=None):
    """Setting camera matrix of this scene

    Args:
        camera_matrix ( numpy.array[3x3] ) : Camera matrix to be set
        width ( int, optional ) : Set width of this scene
        height ( int, optional ) : Set height of this scene

    """
    ff_ = (camera_matrix[0][0] + camera_matrix[1][1])/2
    sw = currentSceneWidget()
    if width is not None and height is not None:
        sw.setScreenSize(width, height)
    width_  = sw.width()
    height_ = sw.height()
    if height_ < width_:
        fov_ = 2 * math.atan2(height_/2, ff_)
    else:
        fov_ = 2 * math.atan2(width_/2, ff_)
    sw.builtinPerspectiveCamera.setFieldOfView(fov_)

def projectPoints(point_list, world=True):
    """Projecting 3D points to 2D points on image plane

    Args:
        point_list ( list [ numpy.array[3x3] ] ) : List 3D points on world coordinates

    Returns:
        list [ numpy.array ] : List of projected points

    """
    cam_matrix = getCameraMatrix()
    if world:
        cam_cds, fov = getCameraCoords()
        point_list = [ cam_cds.inverse_transform_vector(pt) for pt in point_list ]
    ret = []
    for cam_pt in point_list:
        uvs = cam_matrix.dot(cam_pt)
        ret.append( npa([uvs[0]/uvs[2], uvs[1]/uvs[2]], dtype='float64') )
    return ret

def unprojectPoints(uv_list, depth=1.0, depth_list=None, centerRelative=False, world=True):
    """Projecting 3D points to 2D points on image plane

    Args:
        point_list ( list [ numpy.array[3x3] ] ) : List 3D points on world coordinates

    Returns:
        list [ numpy.array ] : List of projected points

    """
    sw = currentSceneWidget()
    fov_ = sw.builtinPerspectiveCamera.fieldOfView
    width_  = sw.width()
    height_ = sw.height()
    if centerRelative:
        cx_ = 0.0
        cy_ = 0.0
    else:
        cx_ = 0.5*width_
        cy_ = 0.5*height_
    if height_ < width_:
        ff_  = 0.5*height_ / math.tan(fov_/2)
    else:
        ff_  = 0.5*width_ / math.tan(fov_/2)
    zz = depth/ff_

    pts = [ npa([(uv[0] - cx_)*zz, (uv[1] - cy_)*zz, depth], dtype='float64') for uv in uv_list ]

    if world:
        cam_cds, fov = getCameraCoords()
        return [ cam_cds.transform_vector(pt) for pt in pts ]
    else:
        return pts

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
    """Showing grids

    Args:
        plane (int, default = 0) : ID of plane to show. 0\: XY, 1\: XZ, 2\: YZ

    """
    sw = currentSceneWidget()
    sw.setGridEnabled(SceneWidget.GridPlane(plane), True)
    sw.updateGrids()

def setCoordinateAxes(on=True):
    """On/Off of the coordinate axes on the screen

    Args:
        on (boolean, default=True) : On/Off of the coordinate axes

    """
    sw = currentSceneWidget()
    sw.setCoordinateAxes(on)

def setViewSize(width, height):
    """Setting size of view(widget)

    Args:
        width (int) : Width of the widget to be set
        height (int) : Height of the widget to be set

    """
    sw = currentSceneWidget()
    sw.resize(width, height)

def viewAll():
    """Setting camera position as viewing all objects
    """
    sw = currentSceneWidget()
    sw.viewAll()

def getAllBoundingBox():
    """Getting a boundingBox of all objects on the scene

    Returns:
        BoundingBox : BoundingBox of all objects on the scene

    """
    sw = currentSceneWidget()
    return sw.scene.boundingBox()
