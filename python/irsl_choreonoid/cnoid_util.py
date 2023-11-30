from cnoid.Base import RootItem

from cnoid.Body import BodyLoader
from cnoid.Body import StdBodyWriter
from cnoid.Body import Body
from cnoid.Body import Link

try: ## for original-choreonoid
    from cnoid.URDFPlugin import URDFBodyWriter
except ImportError:
    pass

import cnoid.Util

import numpy as np

from .make_shapes import addUriToShape

from urllib.parse import urlparse
import os

##
## python utility
##
def load_script(filename):
    """Loading a script file

    Args:
        filename (str): URL [ URL is like 'scheme://netloc/xxx/yyy/zzz' ]

    """
    ### another way
    #import runpy
    #runpy.run_path(path_name=filename)
    exec(open(str(filename)).read())

def parseURL(url):
    """Parsing URL with IRSL original scheme

    Args:
        url (str): URL

    Returns:
        str: Absolute path

    Raises:
        SyntaxError : Unknown scheme or netloc is passed

    Examples:
        >>> parseURL('choreonoid://share/dir/file')
        /choreonoid/share/choreonoid-2.0/dir/file

        >>> parseURL('env://HOME/dir/file')
        /home/user/dir/file

        >>> parseURL('file:///dir/file')
        /dir/file

        >>> parseURL('file://./dir/file')
        /current_dir/dir/file

        >>> parseURL('file://~/dir/file')
        /home/user/dir/file

    Note:
    URL is like 'scheme://netloc/xxx/yyy/zzz'

    Implemented scheme is 'choreonoid', 'env', 'file'

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
def loadRobot(fname):
    """Loading robot model (.body, .vrml, .urdf??)

    Args:
        fname (str): filename of robot-model

    Returns:
        cnoid.Body: instance of cnoid.Body.Body

    """
    bl_ = BodyLoader()
    if hasattr(bl_, 'setMessageSinkStdErr'):
        bl_.setMessageSinkStdErr()
    rb = bl_.load(str(fname))
    if rb is None:
        return None
    rb.updateLinkTree()
    rb.initializePosition()
    rb.calcForwardKinematics()
    return rb

def exportBody(fname, body, extModelFileMode=None, fileUri=None, allInOne=True, fixMassParam=True):
    """
    Args:
        fname (str) :
        body ( cnoid.Body.Body ) :
        extModelFileMode (int, optional): 0; EmbedModels, 1; LinkToOriginalModelFiles, 2; ReplaceWithStdSceneFiles, 3; ReplaceWithObjModelFiles
        fileUri (str, optional) : Add URI for exporting mesh files
        allInOne (boolean, default=True) : Using with fileUri, if True, all shapes will be exported as a file.
        fixMassParam (boolean, default=False) : If True, links with mass==1.0 and inertia is identity is set small mass-parameter (they may loaded without mass parameter)

    """
    if fileUri is not None:
        for lk in body.links:
            nm = lk.name
            if allInOne:
                for idx in range(lk.visualShape.numChildren):
                    addUriToShape(lk.visualShape.getChild(idx), '{}_{}_vis.scen'.format(nm, idx), fileUri, allInOne=allInOne)
                for idx in range(lk.collisionShape.numChildren):
                    addUriToShape(lk.collisionShape.getChild(idx), '{}_{}_col.scen'.format(nm, idx), fileUri, allInOne=allInOne)
            else:
                addUriToShape(lk.visualShape, '{}_vis'.format(nm), fileUri)
                addUriToShape(lk.collisionShape, '{}_col'.format(nm), fileUri)
    bw = StdBodyWriter()
    bw.setMessageSinkStdErr()
    if extModelFileMode is not None:
        bw.setExtModelFileMode(extModelFileMode)
    if fixMassParam:
        for ll in body.links:
            mass = ll.mass
            II = ll.I
            if mass == 1.0 and II[0][0] == 1.0 and II[1][1] == 1.0 and II[2][2] == 1.0 and II[0][1] == 0.0 and II[0][2] == 0.0 and II[1][2] == 0.0:
                print('link: {}, small mass-paramters is set'.format(ll.name))
                ll.setMass(0.001)
                ll.setInertia( np.array( ((1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0)) ) * 1e-9 )
    return bw.writeBody(body, fname)

def exportURDF(fname, body, **kwargs):
    """
    Args:
        fname (str) :
        body ( cnoid.Body.Body ) :
        kwargs ( dict[str, param] ) :

    """
    ubw = URDFBodyWriter()
    ubw.setMessageSinkStdErr()
    for k, v in kwargs.items():
        exec('ubw.set{}(v)'.format(k))
    return ubw.writeBody(body, fname)

def castValueNode(_valuenode):
    """Casting cnoid.Util.ValueNode type to python primitive type

    Args:
        _valuenode (cnoid.Util.ValueNode) : ValueNode

    Returns:
        dict, list, int, float, bool, str : Instance of python primitive type

    """
    if _valuenode.isMapping():
        return mappingToDict(_valuenode.toMapping())
    elif _valuenode.isListing():
        return listingToList(_valuenode.toListing())
    elif _valuenode.isScalar():
        instr = _valuenode.toString()
        try:
            res = int(instr)
        except Exception as e:
            try:
                res = float(instr)
            except Exception as e:
                lowerstr = instr.lower()
                if lowerstr == 'true':
                    return True
                elif lowerstr == 'false':
                    return False
                else:
                    return instr
        return res

def mappingToDict(mapping):
    """Converting cnoid.Util.Mapping to dict

    Args:
        mapping (cnoid.Util.Mapping) : Instance of mapping to be converted

    Returns:
        dict : Converted dictionary

    """
    res = {}
    for key in mapping.keys():
        res[key] = castValueNode(mapping[key])
    return res

def listingToList(listing):
    """Converting cnoid.Util.Listing to list

    Args:
        listing (cnoid.Util.Listing) : Instance of listing to be converted

    Returns:
        list : Converted list

    """
    res = []
    for idx in range(len(listing)):
        res.append(castValueNode(listing[idx]))
    return res

def dictToMapping(indict):
    """Converting dict to cnoid.Util.Mapping

    Args:
        indict (dict) : Dict to be converted

    Returns:
        cnoid.Util.Mapping : Converted mapping

    """
    res = cnoid.Util.Mapping()
    for k, v in indict.items():
        if type(v) is dict:
            res.insert(k, dictToMapping(v))
        elif type(v) is list or type(v) is tuple:
            res.insert(k, listToListing(v))
        else:
            res.write(k, v)
    return res

def listToListing(inlist):
    """Converting list to cnoid.Util.Mapping

    Args:
        inlist (list) : List to be converted

    Returns:
        cnoid.Util.Mapping : Converted mapping

    """
    res = cnoid.Util.Listing()
    for v in inlist:
        if type(v) is dict:
            res.append(dictToMapping(v))
        elif type(v) is list or type(v) is tuple:
            res.append(listToListing(v))
        elif type(v) is bool:
            res.append(str(v).lower())
        else:
            res.append(v)
    return res

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

## only this function use cnoid.Base module
def isInChoreonoid():
    """isInChoreonoid

    Args:
        None

    Returns:
        boolean: True if this script running on python-console of Choreonoid

    """
    return (RootItem.instance is not None)
