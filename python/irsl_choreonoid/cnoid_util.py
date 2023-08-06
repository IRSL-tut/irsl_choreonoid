from cnoid.Base import RootItem

from cnoid.Body import BodyLoader
from cnoid.Body import Body
from cnoid.Body import Link

import cnoid.Util

import numpy as np

from urllib.parse import urlparse
import os

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
        str: Absolute path

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
    rb = BodyLoader().load(str(fname))
    rb.updateLinkTree()
    rb.initializePosition()
    rb.calcForwardKinematics()
    return rb

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
