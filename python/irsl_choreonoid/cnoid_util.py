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
    return (RootItem.instance is not None)

def loadRobot(fname):
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
    cnoid.Base.ProjectManager.instance.loadProject(filename=project_file)

##
## cnoid Item
##
def getItemTreeView():
    if callable(ItemTreeView.instance):
        return ItemTreeView.instance()
    else:
        return ItemTreeView.instance

def getRootItem():
    if callable(RootItem.instance):
        return RootItem.instance()
    else:
        return RootItem.instance

def getWorld(name = 'World'):
    rI = getRootItem()
    ret = rI.findItem(name)
    if ret == None:
        ret = WorldItem()
        ret.setName(name)
        rI.addChildItem(ret)
        getItemTreeView().checkItem(ret)
    return ret

def addSimulator(world = None, simulator_name = 'AISTSimulator'):
    if world is None:
        world = getWorld()
    sim_ = world.findItem(simulator_name)
    if sim_ == None:
        sim_ = AISTSimulatorItem()
        world.addChildItem(sim_)
        getItemTreeView().checkItem(sim_)
    return sim_

def loadRobotItem(fname, name = None, world = True):
    '''Load robot model and add it as Item

    Parameters
    ----------
    fname : str
        file name of model
    name : str
        name of Item

    Returns
    -------
    instance of cnoid.Body.Body
    '''
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
    return RootItem.instance.findItem(name)

def findItems(name):
    return [ itm for itm in RootItem.instance.getDescendantItems() if itm.name == name ]

def removeItem(item_):
    item_.detachFromParentItem()

def findBodyItem(name_or_body):
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
  ret = np.identity(4)
  if not (rotation is None):
    ret[:3, :3] = rotation
  if not (translation is None):
    ret[:3, 3] = translation
  return ret

def cnoidRotation(cPosition):
  return cPosition[:3, :3]

def cnoidTranslation(cPosition):
  return cPosition[:3, 3]
