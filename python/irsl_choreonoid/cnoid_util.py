from cnoid.Base import RootItem
from cnoid.Base import ItemTreeView
from cnoid.Base import MessageView

from cnoid.BodyPlugin import AISTSimulatorItem
from cnoid.BodyPlugin import BodyItem
from cnoid.BodyPlugin import WorldItem

from cnoid.Body import BodyLoader
from cnoid.Body import Body
from cnoid.Body import Link

import cnoid.Util

import numpy as np

from urllib.parse import urlparse

##
## python utility
##
def load_script(filename):
    ### another way
    #import runpy
    #runpy.run_path(path_name=filename)
    exec(open(filename).read())

def parseURL(url):
    """parse URL with IRSL original scheme

    Args:
        url (str): url [ url is like 'scheme://netloc/xxx/yyy/zzz' ]

    Returns:
        str: absolute path

    Examples:
        >>> parseURL('choreonoid://share/dir/file')
        /choreonoid/share/choreonoid-1.8/dir/file

        >>> parseURL('env://HOME/dir/file')
        /home/user/dir/file

        >>> parseURL('file:///dir/file')
        /dir/file

        >>> parseURL('file://./dir/file')
        /current_dir/dir/file

        >>> parseURL('file://~/dir/file')
        /home/user/dir/file
    """

    if not '://' in url:
        return url

    res = urlparse(url)

    if res.scheme == 'choreonoid':
        if res.netloc == 'share':
            return cnoid.Util.shareDirectory + res.path
        ##elif res.netloc == 'bin':
        ##elif res.netloc == 'lib':
        else:
            raise SyntaxError('unkown location {} / {}'.format(res.netloc, url))
    elif res.scheme == 'file':
        if res.netloc == '':
            return res.path
        elif res.netloc == '.':
            return os.getcwd() + res.path
        elif res.netloc == '~':
            return os.environ['HOME'] + res.path
        else:
            raise SyntaxError('unkown location {} / {}'.format(res.netloc, url))
    elif res.scheme == 'env':
        if res.netloc == '':
            raise SyntaxError('unkown location {} / {}'.format(res.netloc, url))
        return os.environ[res.netloc] + res.path
    elif res.scheme == 'http' or res.scheme == 'https':
        raise SyntaxError('not implemented scheme {} / {}'.format(res.scheme, url))
    else:
        raise SyntaxError('unkown scheme {} / {}'.format(res.scheme, url))

##
## cnoid Util
##
def isInChoreonoid():
    """isInChoreonoid

    Args:
        None

    Returns:
        boolean: True if this script running on python-console of Choreonoid

    """
    return (RootItem.instance is not None)

def loadRobot(fname):
    """Loading robot model (.body, .vrml, .urdf??)

    Args:
        fname (str): filename of robot-model

    Returns:
        cnoid.Body: instance of cnoid.Body.Body

    """
    rb = BodyLoader().load(str(fname))
    rb.updateLinkTree()
    rb.initializePosition()
    rb.calcForwardKinematics()
    return rb

def flushRobotView(name):
    #findItem(name).notifyKinematicStateChange()
    findItem(name).notifyKinematicStateUpdate()
    #MessageView.getInstance().flush()
    MessageView.instance.flush()

def loadProject(project_file):
    """Loading project file (currend project may be changed)

    Args:
        project_file (str): filename of project file (.cnoid)

    """
    cnoid.Base.ProjectManager.instance.loadProject(filename=project_file)

##
## cnoid Item
##
def getItemTreeView():
    """DEPRECATED: use cnoid.Base.ItemTreeView.instance
    """
    if callable(ItemTreeView.instance):
        return ItemTreeView.instance()
    else:
        return ItemTreeView.instance

def getRootItem():
    """DEPRECATED: use cnoid.Base.RootItem.instance
    """
    if callable(RootItem.instance):
        return RootItem.instance()
    else:
        return RootItem.instance

def getWorld(name = 'World'):
    """Getting or creating WorldItem

    Args:
        name (str, default = 'World') : name of WorldItem

    Returns:
        cnoid.Base.WorldItem : added or found WorldItem

    """
    rI = getRootItem()
    ret = rI.findItem(name)
    if ret == None:
        ret = WorldItem()
        ret.setName(name)
        rI.addChildItem(ret)
        getItemTreeView().checkItem(ret)
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
        world = getWorld()
    sim_ = world.findItem(simulator_name)
    if sim_ == None:
        sim_ = AISTSimulatorItem()
        world.addChildItem(sim_)
        getItemTreeView().checkItem(sim_)
    return sim_

def loadRobotItem(fname, name = None, world = True):
    """Load robot model and add it as a BodyItem

    Args:
        fname (str): filename (or path)
        name (str, optional): name of loaded robot-model
        world (boolean or WorldItem, default = True): if True, WorldItem is added

    Returns:
        cnoid.BodyPlugin.BodyItem : Loaded robot-model

    """
    # print('loadRobot: %s'%(fname))
    bI = BodyItem()
    bI.load(str(fname))
    if name:
        bI.setName(name)
    if callable(bI.body):
        rr = bI.body()
    else:
        rr = bI.body
    rr.updateLinkTree()
    rr.initializePosition()
    rr.calcForwardKinematics()
    bI.storeInitialState()
    if world == True:
        wd = getWorld()
        if callable(wd.childItem):
            wd.insertChildItem(bI, wd.childItem())
        else:
            wd.insertChildItem(bI, wd.childItem)
    elif type(world) is WorldItem:
        if callable(world.childItem):
            world.insertChildItem(bI, world.childItem())
        else:
            world.insertChildItem(bI, world.childItem)

    getItemTreeView().checkItem(bI)
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

def removeItem(item_):
    """Removing item

    Args:
        item (cnoid.Base.Item) : item to be removed

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
    elif type(name_or_body) is cnoid.Body.Body:
        for itm in RootItem.instance.getDescendantItems():
            if type(itm) is cnoid.BodyPlugin.BodyItem and itm.body == name_or_body:
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

##
## cnoid Position
##
def cnoidPosition(rotation = None, translation = None):
    """Concatnating translation part and rotation part

    Args:
         translation(numpy.array, optional) : 1x3 vector
         rotation(numpy.array, optional) : 3x3 matrix

    Returns:
         numpy.array : 4x4 homogeneous transformation matrix

    """
    ret = np.identity(4)
    if not (rotation is None):
        ret[:3, :3] = rotation
    if not (translation is None):
        ret[:3, 3] = translation
    return ret

def cnoidRotation(cPosition):
    """Extracting rotation part of 4x4 matrix

    Args:
         cPosition (numpy.array) : 4x4 homogeneous transformation matrix

    Returns:
         numpy.array : 3x3 matrix ( rotation part of cPosition )

    """
    return cPosition[:3, :3]

def cnoidTranslation(cPosition):
    """Extracting translation part of 4x4 matrix

    Args:
         cPosition (numpy.array) : 4x4 homogeneous transformation matrix

    Returns:
         numpy.array : 1x3 vector ( translation part of cPosition )

    """
    return cPosition[:3, 3]
