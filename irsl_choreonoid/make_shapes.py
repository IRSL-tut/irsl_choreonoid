import numpy
from numpy import array as npa
import cnoid.AssimpPlugin
import cnoid.Util as cutil
from math import pi as PI

from .irsl_draw_object import *

##
RED    = npa([1, 0, 0], dtype='float32')
YELLOW = npa([1, 1, 0], dtype='float32')
GREEN  = npa([0, 1, 0], dtype='float32')
CYAN   = npa([0, 1, 1], dtype='float32')
BLUE   = npa([0, 0, 1], dtype='float32')
PURPLE = npa([1, 0, 1], dtype='float32')
##
ORANGE = npa([1, 0.5, 0], dtype='float32')
LIME   = npa([0.5, 1, 0], dtype='float32')
GREEN2 = npa([0, 1, 0.5], dtype='float32')
BLUE2  = npa([0, 0.5, 1], dtype='float32')
BLUE_PURPLE  = npa([0.5, 0, 1], dtype='float32')
RED_PURPLE   = npa([1, 0, 0.5], dtype='float32')
##
BLACK  = npa([0, 0, 0], dtype='float32')
WHITE  = npa([1, 1, 1], dtype='float32')
GRAY1  = npa([0.1, 0.1, 0.1], dtype='float32')
GRAY2  = npa([0.2, 0.2, 0.2], dtype='float32')
GRAY3  = npa([0.3, 0.3, 0.3], dtype='float32')
GRAY4  = npa([0.4, 0.4, 0.4], dtype='float32')
GRAY5  = npa([0.5, 0.5, 0.5], dtype='float32')
GRAY6  = npa([0.6, 0.6, 0.6], dtype='float32')
GRAY7  = npa([0.7, 0.7, 0.7], dtype='float32')
GRAY8  = npa([0.8, 0.8, 0.8], dtype='float32')
GRAY9  = npa([0.9, 0.9, 0.9], dtype='float32')

def __gets(klst, amap):
    for k in klst:
        if k in amap:
            return amap[k]
    return None

def generateMaterial(**kwargs):
    if 'material' in kwargs:
        return generateMaterial(**(kwargs['material']))

    valueset = False
    mat = cutil.SgMaterial()
    val = __gets(('AmbientIntensity', 'ambientintensity', 'Intensity', 'intensity', 'ambient-intensity'), kwargs)
    if val is not None:
        mat.setAmbientIntensity(val)
        valueset = True

    val = __gets(('DiffuseColor', 'diffusecolor', 'diffuse-color', 'diffuse'), kwargs)
    if val is not None:
        mat.setDiffuseColor(npa(val, dtype='float32'))
        valueset = True

    val = __gets(('EmissiveColor', 'emissivecolor', 'emissive-color', 'emissive'), kwargs)
    if val is not None:
        mat.setEmissiveColor(npa(val, dtype='float32'))
        valueset = True

    val = __gets(('SpecularExponent', 'specularexponent', 'specular-exponent'), kwargs)
    if val is not None:
        mat.setSpecularExponent(val, dtype='float32')
        valueset = True

    val = __gets(('SpecularColor', 'specularcolor', 'specular-color', 'specular'), kwargs)
    if val is not None:
        mat.setSpecularColor(npa(val, dtype='float32'))
        valueset = True

    val = __gets(('Transparency', 'transparency', 'Transparent', 'transparent'), kwargs)
    if val is not None:
        mat.setTransparency(val)
        valueset = True

    val = __gets(('color', 'Color'), kwargs)
    if val is not None:
        mat.setAmbientIntensity(1.0)
        mat.setDiffuseColor(npa(val, dtype='float32') * 0.7)
        mat.setEmissiveColor(npa(val, dtype='float32') * 0.3)
        valueset = True

    if valueset:
        return mat
    return None

def parseMeshGeneratorOption(mg, **kwargs):
    val = __gets(('DivisionNumber',), kwargs)
    if val is not None:
        mg.setDivisionNumber(val)
    val = __gets(('ExtraDivisionNumber',), kwargs)
    if val is not None:
        mg.setExtraDivisionNumber(val)
    val = __gets(('NormalGenerationEnabled',), kwargs)
    if val is not None:
        mg.setNormalGenerationEnabled(val)
    val = __gets(('BoundingBoxUpdateEnabled',), kwargs)
    if val is not None:
        mg.setBoundingBoxUpdateEnabled(val)

def __extractShape(sg_node):
    res = []
    if type(sg_node) is cutil.SgShape:
        res.append(sg_node)
    elif hasattr(sg_node, 'numChildren'):
        for idx in range(sg_node.numChildren):
            res += __extractShape(sg_node.getChild(idx))
    return res

def extractNode(sg_node, currentCoords=None, nodeTypes=None):
    res = []
    for ntp in nodeTypes:
        if isinstance(sg_node, ntp):
            res.append((sg_node, currentCoords))
    if sg_node.isGroupNode():
        if isinstance(sg_node, cutil.SgPosTransform):
            if currentCoords is None:
                currentCoords = coordinates(sg_node.T)
            else:
                currentCoords = currentCoords.copy().transform(coordinates(sg_node.T))
        for idx in range(sg_node.numChildren):
            ch = sg_node.getChild(idx)
            res += extractNode(ch, currentCoords=currentCoords, nodeTypes=nodeTypes)
    return res

def extractShapes(sg_node, currentCoords=None):
    """Extracting SgShape from SceneGraph

    Args:
        sg_node (cnoid.Util.SgNode) : Root node to start searching
        currentCoords (cnoid.IRSLCoords.coordinates, optional) : Offset coordinates to extracted object

    Returns:
        list [ tuple [ cnoid.Util.SgShape, cnoid.IRSLCoords.coordinates ] ] : List of extracted objects which is tuple of object-instance and coordinates of this object

    """
    return extractNode(sg_node, currentCoords=currentCoords, nodeTypes=[cutil.SgShape])

def extractPlots(sg_node, currentCoords=None):
    """Extracting SgPlot from SceneGraph

    Args:
        sg_node (cnoid.Util.SgNode) : Root node to start searching
        currentCoords (cnoid.IRSLCoords.coordinates, optional) : Offset coordinates to extracted object

    Returns:
        list [ tuple [ cnoid.Util.SgPlot, cnoid.IRSLCoords.coordinates ] ] : List of extracted objects which is tuple of object-instance and coordinates of this object

    """
    return extractNode(sg_node, currentCoords=currentCoords, nodeTypes=[cutil.SgPlot])

def extractDrawables(sg_node, currentCoords=None):
    """Extracting drawables (SgShape, SgPlot, SgText) from SceneGraph

    Args:
        sg_node (cnoid.Util.SgNode) : Root node to start searching
        currentCoords (cnoid.IRSLCoords.coordinates, optional) : Offset coordinates to extracted object

    Returns:
        list [ tuple [ instance of drawables, cnoid.IRSLCoords.coordinates ] ] : List of extracted objects which is tuple of object-instance and coordinates of this object

    """
    return extractNode(sg_node, currentCoords=currentCoords, nodeTypes=[cutil.SgShape, cutil.SgPlot, cutil.SgText])

def loadScene(fname, meshScale=None, fileUri=None, wrapped=True, rawShape=False, coords=None, **kwargs):
    """Loading scene(wrl, scene, ...) file using cnoid.Util.SceneLoader

    Args:
        fname (str) : File name to be loaded
        fileUri (str, optional) : If fileUri is set, shapes will be set Uri for exporting files
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgShape will be returned (ignore wrapped)
        coords (cnoid.IRSLCoords.coordinates, optional) :
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    """
    ld = cutil.SceneLoader()
    ld.setMessageSinkStdErr()
    ## add ld options
    sg = ld.load(fname)
    if sg is None:
        raise Exception(f'Loading scene was failed : {fname}')
    shapes = __extractShape(sg)

    mat = generateMaterial(**kwargs)

    if mat is not None:
        for shape in shapes:
            shape.setMaterial(mat)

    if fileUri is not None:
        sg.setUri(fname, fileUri)

    if meshScale is not None:
        scl_ = cutil.SgScaleTransform(meshScale)
        scl_.addChild(sg)
        sg = scl_

    if rawShape:
        if coords is not None:
            ret = cutil.SgPosTransform()
            ret.setPosition(coords.cnoidPosition)
            ret.addChild(sg)
            return ret
        else:
            return sg

    ret = None
    if type(sg) is cutil.SgPosTransform:
        ret = sg
    else:
        ret = cutil.SgPosTransform()
        ret.addChild(sg)
    if wrapped:
        ret = coordsWrapper(ret, original_object=sg)
        if coords is not None:
            ret.newcoords(coords)
    else:
        if coords is not None:
            ret.setPosition(coords.cnoidPosition)
    return ret

def loadMesh(fname, meshScale=None, fileUri=None, wrapped=True, rawShape=False, coords=None, forceGenerateNormals=True, creaseAngle=PI/6, **kwargs):
    """Loading mesh file using cnoid.AssimpPlugin module

    Args:
        fname (str) : File name to be loaded
        fileUri (str, optional) : If fileUri is set, shapes will be set Uri for exporting files
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgShape will be returned (ignore wrapped)
        coords (cnoid.IRSLCoords.coordinates, optional) :
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    """
    ld = cnoid.AssimpPlugin.AssimpSceneLoader()
    ld.setMessageSinkStdErr()
    ld.setForceGenerateNormals(forceGenerateNormals)
    ld.setCreaseAngle(creaseAngle)
    ## add ld options
    sg = ld.load(fname)
    if sg is None:
        raise Exception(f'Loading mesh was failed : {fname}')
    shapes = __extractShape(sg)

    mat = generateMaterial(**kwargs)

    if fileUri is not None:
        sg.setUri(fname, fileUri)

    if mat is not None:
        for shape in shapes:
            shape.setMaterial(mat)

    if meshScale is not None:
        scl_ = cutil.SgScaleTransform(meshScale)
        scl_.addChild(sg)
        sg = scl_

    if rawShape:
        if coords is not None:
            ret = cutil.SgPosTransform()
            ret.setPosition(coords.cnoidPosition)
            ret.addChild(sg)
            return ret
        else:
            return sg

    ret = cutil.SgPosTransform()
    ret.addChild(sg)
    if wrapped:
        ret = coordsWrapper(ret, original_object=sg)
        if coords is not None:
            ret.newcoords(coords)
    else:
        if coords is not None:
            ret.setPosition(coords.cnoidPosition)
    return ret

def __genShape(mesh, meshScale=None, wrapped=True, rawShape=False, coords=None, texture=None, **kwargs):
    sg = cutil.SgShape()
    sg.setMesh(mesh)
    if type(texture) is str:
        sg.setTextureImage(texture)

    mat = generateMaterial(**kwargs)

    if mat is not None:
        sg.setMaterial(mat)

    if meshScale is not None:
        scl_ = cutil.SgScaleTransform(meshScale)
        scl_.addChild(sg)
        sg = scl_

    if rawShape:
        if coords is not None:
            ret = cutil.SgPosTransform()
            ret.setPosition(coords.cnoidPosition)
            ret.addChild(sg)
            return ret
        else:
            return sg

    ret = cutil.SgPosTransform()
    ret.addChild(sg)
    if wrapped:
        ret = coordsWrapper(ret, original_object=sg)
        if coords is not None:
            ret.newcoords(coords)
    else:
        if coords is not None:
            ret.setPosition(coords.cnoidPosition)
    return ret

def makeBox(x, y = None, z = None, wrapped=True, rawShape=False, coords=None, texture=None, **kwargs):
    """Making 'Box' shape using cnoid.Util.MeshGenerator

    Args:
        x (float) : Length of x-axis, if y and z is None, making 'cube'
        y (float, optional) : Length of y-direction edge
        z (float, optional) : Length of z-direction edge
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgShape will be returned (ignore wrapped)
        coords (cnoid.IRSLCoords.coordinates, optional) :
        texture (str, optional) : Image file-name of texture
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    Note:
        Origin of generated shape is the center of it

    """
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    if texture is not None:
        tex_flag = 1
    else:
        tex_flag = 0
    if type(x) is numpy.ndarray or type(x) is list:
        mesh = mg.generateBox(npa(x, dtype='float64'), tex_flag)
    elif y is not None and z is not None:
        mesh = mg.generateBox(npa([x, y, z], dtype='float64'), tex_flag)
    elif type(x) is int or type(x) is float:
        mesh = mg.generateBox(npa([x, x, x], dtype='float64'), tex_flag)
    else:
        raise Exception(f'Invalid arguments x: {x}, y: {y}, z: {z}')

    if mesh is None:
        raise Exception(f'Generating mesh was failed x: {x}, y: {y}, z: {z}')

    return __genShape(mesh, wrapped=wrapped, rawShape=rawShape, coords=coords, texture=texture, **kwargs)

def makeCylinder(radius, height, wrapped=True, rawShape=False, coords=None, texture=None, **kwargs):
    """Making 'Cylinder' shape using cnoid.Util.MeshGenerator

    Args:
        radius (float) : Radius of the cylinder
        height (float) : Height of the cylinder
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgShape will be returned (ignore wrapped)
        coords (cnoid.IRSLCoords.coordinates, optional) :
        texture (str, optional) : Image file-name of texture
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    Note:
        Center circle with indicated radius on XZ-plane and sweep to both side with half of height, along y-direction

    """
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    if texture is not None:
        tex_flag = 1
    else:
        tex_flag = 0
    mesh = mg.generateCylinder(radius, height, tex_flag)
    return __genShape(mesh, wrapped=wrapped, rawShape=rawShape, coords=coords, texture=texture, **kwargs)

def makeSphere(radius, wrapped=True, rawShape=False, coords=None, texture=None, **kwargs):
    """Make 'Sphere' shape using cnoid.Util.MeshGenerator

    Args:
        radius (float) : Radius of the sphere
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgShape will be returned (ignore wrapped)
        coords (cnoid.IRSLCoords.coordinates, optional) :
        texture (str, optional) : Image file-name of texture
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    Note:
        Origin of generated shape is the center of it

    """
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    if texture is not None:
        tex_flag = 1
    else:
        tex_flag = 0
    mesh = mg.generateSphere(radius, tex_flag)
    return __genShape(mesh, wrapped=wrapped, rawShape=rawShape, coords=coords, texture=texture, **kwargs)

def makeCone(radius, height, wrapped=True, rawShape=False, coords=None, texture=None, **kwargs):
    """Making 'Cone' shape using cnoid.Util.MeshGenerator

    Args:
        radius (float) : Radius of the cone
        height (float) : Height of the cone
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgShape will be returned (ignore wrapped)
        coords (cnoid.IRSLCoords.coordinates, optional) :
        texture (str, optional) : Image file-name of texture
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    if texture is not None:
        tex_flag = 1
    else:
        tex_flag = 0
    mesh = mg.generateCone(radius, height, tex_flag)
    return __genShape(mesh, wrapped=wrapped, rawShape=rawShape, coords=coords, texture=texture, **kwargs)

def makeCapsule(radius, height, wrapped=True, rawShape=False, coords=None, **kwargs):
    """Makeing 'Capsule' shape using cnoid.Util.MeshGenerator

    Args:
        radius (float) : Radius of the cupsule
        height (float, optional) : Height of the capsule
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgShape will be returned (ignore wrapped)
        coords (cnoid.IRSLCoords.coordinates, optional) :
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    Note:
        Similar dimensions to 'makeCylinder' (bottom cricle is at minus y, cone's tip is at plus y)

    """
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateCapsule(radius, height)
    return __genShape(mesh, wrapped=wrapped, rawShape=rawShape, coords=coords, **kwargs)

def makeTorus(radius, corssSectionRadius, beginAngle = None, endAngle = None, wrapped=True, rawShape=False, coords=None, **kwargs):
    """Makeing 'Torus' shape using cnoid.Util.MeshGenerator

    Args:
        radius (float) : Outer radius of the torus
        crossSectionRadius (float) : Radius of cross section
        beginAngle (float, optional) : If beginAngle and endAngle is passed, part of whole torus is created
        endAngle (float, optional) : 
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgShape will be returned (ignore wrapped)
        coords (cnoid.IRSLCoords.coordinates, optional) :
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    if beginAngle is not None and endAngle is not None:
        mesh = mg.generateTorus(radius, corssSectionRadius, beginAngle, endAngle)
    else:
        mesh = mg.generateTorus(radius, corssSectionRadius)

    return __genShape(mesh, wrapped=wrapped, rawShape=rawShape, coords=coords, **kwargs)

def _makeExtrusionParam(crossSection, spine, orientation=None, scale=None, creaseAngle=None, beginCap=None, endCap=None, **kwargs):
    extconf = cutil.MeshGenerator.Extrusion()
    extconf.crossSection = npa(crossSection, dtype='float64')
    extconf.spine        = npa(spine, dtype='float64')
    if orientation is not None:
        extconf.orientation=orientation
    if scale is not None:
        extconf.scale=npa(scale, dtype='float64')
    if creaseAngle is not None:
        extconf.creaseAngle=creaseAngle
    if beginCap is not None:
        extconf.beginCap=beginCap
    if endCap is not None:
        extconf.endCap=endCap
    return extconf

def _makeExtrusion(_extrusion=None, wrapped=True, rawShape=False, coords=None, texture=None, **kwargs):
    if type(_extrusion) is not cutil.MeshGenerator.Extrusion:
        _extrusion = makeExtrusionParam(**kwargs)
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    if texture is not None:
        tex_flag = 1
    else:
        tex_flag = 0
    mesh = mg.generateExtrusion(_extrusion, tex_flag)
    return __genShape(mesh, wrapped=wrapped, rawShape=rawShape, coords=coords, texture=texture, **kwargs)

def makeExtrusion(crossSection, spine, wrapped=True, rawShape=False, coords=None, texture=None, **kwargs):
    """Makeing 'Extrusion' shape using cnoid.Util.MeshGenerator

    Args:
        crossSection ( list[list[float]],  N x 2 matrix) :  / arg for cnoid.Util.MeshGenerator.Extrusion
        spine ( list[list[float]], M x 3 matrix) : / arg for cnoid.Util.MeshGenerator.Extrusion
        orientation (list[AngleAxis], optional) :  / arg for cnoid.Util.MeshGenerator.Extrusion
        scale ( list[list[float]],  N x 2 matrix, optional) : / arg for cnoid.Util.MeshGenerator.Extrusion
        creaseAngle (float, optional) : / arg for cnoid.Util.MeshGenerator.Extrusion
        beginCap (boolean, optional) : / arg for cnoid.Util.MeshGenerator.Extrusion
        endCap (boolean, optional) : / arg for cnoid.Util.MeshGenerator.Extrusion
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgShape will be returned (ignore wrapped)
        coords (cnoid.IRSLCoords.coordinates, optional) :
        texture (str, optional) : Image file-name of texture
        kwargs ( dict[str, param] ) : Keywords for generating material, mesh, and makeExtrusionParam

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    param=_makeExtrusionParam(crossSection, spine, **kwargs)
    return _makeExtrusion(param, wrapped=wrapped, rawShape=rawShape, coords=coords, texture=texture, **kwargs)

def _makeElevationParam(xDimension, zDimension, xSpacing, zSpacing, height, ccw=None, creaseAngle=None, **kwargs):
    eg = cutil.MeshGenerator.ElevationGrid()
    eg.xDimension = xDimension
    eg.zDimension = zDimension
    eg.xSpacing = xSpacing
    eg.zSpacing = zSpacing
    eg.height = height
    if ccw is not None:
        eg.ccw = ccw
    if creaseAngle is not None:
        eg.creaseAngle = creaseAngle
    return eg

def _makeElevationGrid(_elevation_grid=None, wrapped=True, rawShape=False, coords=None, texture=None, **kwargs):
    if type(_elevation_grid) is not cutil.MeshGenerator.ElevationGrid:
        _elevation_grid = makeElevationParam(**kwargs)
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    if texture is not None:
        tex_flag = 1
    else:
        tex_flag = 0
    mesh = mg.generateElevationGrid(_elevation_grid, tex_flag)
    return __genShape(mesh, wrapped=wrapped, rawShape=rawShape, coords=coords, texture=texture, **kwargs)

def makeElevationGrid(xDimension, zDimension, xSpacing, zSpacing, height, wrapped=True, rawShape=False, coords=None, texture=None, **kwargs):
    """Makeing 'Extrusion' shape using cnoid.Util.MeshGenerator

    Args:
        xDimension (int) : Dimension of x-direction / arg for cnoid.Util.MeshGenerator.ElevationGrid
        zDimension (int) : Dimension of z-direction / arg for cnoid.Util.MeshGenerator.ElevationGrid
        xSpacing (float) : X length is (xDimension - 1) x xSpacing / arg for cnoid.Util.MeshGenerator.ElevationGrid
        zSpacing (float) : Z length is (zDimension - 1) x zSpacing / arg for cnoid.Util.MeshGenerator.ElevationGrid
        height (list[float]) : Size is ( xDimension x zDimension ) / arg for cnoid.Util.MeshGenerator.ElevationGrid
        ccw (boolean, optional) : / arg for cnoid.Util.MeshGenerator.ElevationGrid
        creaseAngl (float, optional) : / arg for cnoid.Util.MeshGenerator.ElevationGrid
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgShape will be returned (ignore wrapped)
        texture (str, optional) : Image file-name of texture
        coords (cnoid.IRSLCoords.coordinates, optional) :
        kwargs ( dict[str, param] ) : Keywords for generating material, mesh, and makeElevationParam

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    param=_makeElevationParam(xDimension, zDimension, xSpacing, zSpacing, height, texture=texture, **kwargs)
    return _makeElevationGrid(param, wrapped=wrapped, rawShape=rawShape, coords=coords, **kwargs)

def makePoints(points, pointSize=10.0, colors=None, colorIndices=None, wrapped=True, rawShape=False, coords=None, **kwargs):
    """Makeing '3D point cloud' shape

    Args:
        points (numpy.array) : N x 3 matrix (N is number of points)
        pointSize (float, default=1.0) :
        colors ( list[list[float]], optional ) :
        colorIndices ( list[int], optional ) :
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgPointSet will be returned (ignore wrapped)
        coords (cnoid.IRSLCoords.coordinates, optional) :
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    ps=cutil.SgPointSet()
    ps.pointSize=pointSize
    ps.setVertices(npa(points, dtype='float32'))
    if colors is not None:
        if type(colors) is list:
            ps.setColors(npa(colors, dtype='float32'))
        else:
            ps.setColors(colors)
    if colorIndices is not None:
        ps.setColorIndices(colorIndices)
    elif colors is not None:
        lc = len(colors)
        lp = ps.sizeOfVertices
        ps.setColorIndices( [ i % lc for i in range(lp) ] )
    ps.updateBoundingBox()
    if rawShape:
        return ps
    res=cutil.SgPosTransform()
    res.addChild(ps)
    if coords is not None:
        res.setPosition(coords.cnoidPosition)
    if wrapped:
        res = coordsWrapper(res, original_object=ps)
    return res

def makeText(text, textHeight=1.0, color=None, wrapped=True, rawShape=False, coords=None, **kwargs):
    """Makeing 'Text' shape

    Args:
        text (str) : String to be displayed
        textHeight (float, default=1.0) :
        color ( list[float], optional ) :
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgText will be returned (ignore wrapped)
        coords (cnoid.IRSLCoords.coordinates, optional) :
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    tx = cutil.SgText()
    tx.text = text
    tx.textHeight = textHeight
    if color is not None:
        tx.color = npa(color, dtype='float32')
    if rawShape:
        return tx
    res=cutil.SgPosTransform()
    res.addChild(tx)
    if coords is not None:
        res.setPosition(coords.cnoidPosition)
    if wrapped:
        res = coordsWrapper(res, original_object=tx)
    return res

def makeLines(line_points, line_indices=None, lineWidth=5.0, colors=None, colorIndices=None, coords=None, wrapped=True, rawShape=False, **kwargs):
    """Makeing Lines

    Args:
        line_points (numpy.array) : N x 3 matrix (N is number of points)
        line_indices ( list [ tuple [int] ] ) : example, [ (0, 1), (2, 3) ] represents two lines, line0 is from point0 to point1, line1 is from point2 to point3
        lineWidth (float, default=5.0) :
        colors ( list[list[float]], optional ) :
        colorIndices ( list[int], optional ) :
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgText will be returned (ignore wrapped)
        coords (cnoid.IRSLCoords.coordinates, optional) :
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    ls=cutil.SgLineSet()
    ls.lineWidth=lineWidth
    ls.setVertices(npa(line_points, dtype='float32'))
    if line_indices is None:
        line_indices = []
        for idx in range(len(line_points)-1):
            line_indices.append([idx, idx+1])
    for ln in line_indices:
        ls.addLine(ln[0] , ln[1])
    if colors is not None:
        ls.setColors(npa(colors, dtype='float32'))
    if colorIndices is not None:
        ls.setColorIndices(colorIndices)
    elif colors is not None:
        ls.setColorIndices( [0] * ls.sizeOfVertices )
    ls.updateBoundingBox()
    if rawShape:
        return ls
    res=cutil.SgPosTransform()
    res.addChild(ls)
    if coords is not None:
        res.setPosition(coords.cnoidPosition)
    if wrapped:
        res = coordsWrapper(res, original_object=ls)
    return res

def makeTetrahedron(base_width, base_height, height, base_center=None, center_x=None, center_y=None, wrapped=True, rawShape=False, coords=None, **kwargs):
    """Making 'Tetrahedron' shape

    Args:
        base_width (float) :
        base_height (float) :
        height (float) :
        base_center (float, optional) :
        center_x (float, optional) :
        center_y(float, optional) :
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        rawShape (boolean, default = False) : If True, instance of cnoid.Util.SgShape will be returned (ignore wrapped)
        coords (cnoid.IRSLCoords.coordinates, optional) :
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    if base_center is None:
        base_center = base_width * 0.5
    if center_x is None:
        center_x = base_width * 0.5
    if center_y is None:
        center_y = base_height * 0.5
    mesh = cutil.SgMesh()
    vt = npa(
        [[          0.,          0., 0.],
         [  base_width,          0., 0.],
         [ base_center, base_height, 0.],
         [    center_x,    center_y, height]], dtype='float32')
    idx = [0, 1, 3, 1, 2, 3, 2, 0, 3, 0, 2, 1]
    mesh.setVertices(vt)
    mesh.setFaceVertexIndices(idx)
    mf = cutil.MeshFilter()
    mf.generateNormals(mesh, 1.57, False)
    return __genShape(mesh, wrapped=wrapped, rawShape=rawShape, coords=coords, **kwargs)

### utility functions
def makeAxis(radius=0.075, length=1.0, axisLength=0.35, axisRadius=0.125, axisRatio=None, color=None, scale=None, coords=None, wrapped=True, **kwargs):
    """Makeing single axis shape using cylinder and cone

    Args:
        radius (float, default=0.15) :
        length (float, default=0.8) :
        axisLength (float, default=0.3) :
        axisRadius (float, default=0.25) :
        axisRatio (float, optional) :
        color ( list[float], optional ) :
        scale (float, optional ) :
        coords (cnoid.IRSLCoords.coordinates, optional) :
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    R0=radius
    if axisRatio is None:
        ll=axisLength
        rr=axisRadius
    else:
        ll = axisRatio * length
        rr = ll
    L0=length-ll
    bd0 = makeCylinder(R0, L0, color=color, **kwargs)
    bd0.translate(npa([0,L0/2.0,0]))
    a0 = makeCone(rr, ll, color=color, **kwargs)
    a0.translate(npa([0,L0+ll/2,0]))
    res=cutil.SgPosTransform()
    if scale is not None:
        current = cutil.SgScaleTransform(scale)
        res.addChild(current)
    else:
        current = res
    current.addChild(bd0.target)
    current.addChild(a0.target)
    if coords is not None:
        res.setPosition(coords.cnoidPosition)
    if wrapped:
        res = coordsWrapper(res, original_object=current)
    return res

def make3DAxis(radius=0.15, length=0.8, axisLength=0.3, axisRadius=0.25, axisRatio=None, color=None, scale=None, x_color=[1, 0, 0], y_color=[0, 1, 0], z_color=[0, 0, 1], coords=None, wrapped=True, **kwargs):
    """Makeing '3D-axis' shape using cylinder and cone

    Args:
        radius (float, default=0.15) :
        length (float, default=0.8) :
        axisLength (float, default=0.3) :
        axisRadius (float, default=0.25) :
        axisRatio (float, optional) :
        color ( list[float], optional ) :
        scale (float, optional ) :
        x_color ( list[float], default=[1,0,0] ) :
        y_color ( list[float], default=[0,1,0] ) :
        z_color ( list[float], default=[0,0,1] ) :
        coords (cnoid.IRSLCoords.coordinates, optional) :
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    R0=radius
    if axisRatio is None:
        ll=axisLength
        rr=axisRadius
    else:
        ll = axisRatio * length
        rr = ll
    L0=length-ll

    if color is None:
        col = y_color
    else:
        col = color
    bd0 = makeCylinder(R0, L0, color=col, **kwargs)
    bd0.translate(npa([0,L0/2.0,0]))
    a0 = makeCone(rr, ll, color=col, **kwargs)
    a0.translate(npa([0,L0+ll/2,0]))
    ##
    if color is None:
        col = z_color
    else:
        col = color
    bd1 = makeCylinder(R0, L0, color=col, **kwargs)
    bd1.rotate(PI/2, cutil.UnitX)
    bd1.translate(npa([0,L0/2,0]))
    a1 = makeCone(rr, ll, color=col, **kwargs)
    a1.rotate(PI/2, cutil.UnitX)
    a1.translate(npa([0,L0+ll/2,0]))
    ##
    if color is None:
        col = x_color
    else:
        col = color
    bd2 = makeCylinder(R0, L0, color=col, **kwargs)
    bd2.rotate(-PI/2, cutil.UnitZ)
    bd2.translate(npa([0,L0/2,0]))
    a2 = makeCone(rr, ll, color=col, **kwargs)
    a2.rotate(-PI/2, cutil.UnitZ)
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
    if coords is not None:
        res.setPosition(coords.cnoidPosition)
    if wrapped:
        res = coordsWrapper(res, original_object=current)
    return res

def make3DAxisBox(width=0.2, length=0.8, color=None, scale=None, x_color=[1, 0, 0], y_color=[0, 1, 0], z_color=[0, 0, 1], coords=None, wrapped=True, **kwargs):
    """Makeing '3D-axis' shape using box

    Args:
        width (float, default=0.2) :
        length (float, default=0.8) :
        color ( list[float], optional ) :
        scale (float, optional ) :
        x_color ( list[float], default=[1,0,0] ) :
        y_color ( list[float], default=[0,1,0] ) :
        z_color ( list[float], default=[0,0,1] ) :
        coords (cnoid.IRSLCoords.coordinates, optional) :
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    RR=width
    L0=length
    if color is None:
        col = y_color
    else:
        col = color
    bd0 = makeBox(RR, L0, RR, color=col, **kwargs)
    bd0.translate(npa([0, L0/2, 0]))
    if color is None:
        col = z_color
    else:
        col = color
    bd1 = makeBox(RR, RR, L0, color=col, **kwargs)
    bd1.translate(npa([0, 0, L0/2]))
    if color is None:
        col = x_color
    else:
        col = color
    bd2 = makeBox(L0, RR, RR, color=col, **kwargs)
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
    if coords is not None:
        res.setPosition(coords.cnoidPosition)
    if wrapped:
        res = coordsWrapper(res, original_object=current)
    return res

def makeCoords(length=1.0, lineWidth=2.0, color=None, x_color=[1,0,0], y_color=[0,1,0], z_color=[0,0,1], coords=None, wrapped=True, **kwargs):
    """Makeing '3D-axis' shape using line

    Args:
        length (float, default=0.8) :
        lineWidth (float, default=2.0) :
        color ( list[float], optional ) :
        x_color ( list[float], default=[1,0,0] ) :
        y_color ( list[float], default=[0,1,0] ) :
        z_color ( list[float], default=[0,0,1] ) :
        coords (cnoid.IRSLCoords.coordinates, optional) :
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    ### material??
    ls=cutil.SgLineSet()
    ls.lineWidth=lineWidth
    ls.setVertices(npa([[0,0,0],[length,0,0],[0,length,0],[0,0,length]], dtype='float32'))
    ls.addLine(0,1)
    ls.addLine(0,2)
    ls.addLine(0,3)
    if color is not None:
        x_color = color
        y_color = color
        z_color = color
    ls.setColors(npa([x_color, y_color, z_color], dtype='float32'))
    ls.resizeColorIndicesForNumLines(ls.numLines)
    ls.setLineColor(0, 0)
    ls.setLineColor(1, 1)
    ls.setLineColor(2, 2)
    res=cutil.SgPosTransform()
    res.addChild(ls)
    if coords is not None:
        res.setPosition(coords.cnoidPosition)
    if wrapped:
        res = coordsWrapper(res, original_object=ls)
    return res

def makeCross(length=1.0, lineWidth=2.0, color=None, x_color=[1,0,0], y_color=[0,1,0], z_color=[0,0,1], coords=None, wrapped=True, **kwargs):
    """Makeing '3D-axis' shape using crossing line

    Args:
        length (float, default=0.8) :
        lineWidth (float, default=2.0) :
        color ( list[float], optional ) :
        x_color ( list[float], default=[1,0,0] ) :
        y_color ( list[float], default=[0,1,0] ) :
        z_color ( list[float], default=[0,0,1] ) :
        coords (cnoid.IRSLCoords.coordinates, optional) :
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    ### material??
    if color is not None:
        x_color = color
        y_color = color
        z_color = color
    ls=cutil.SgLineSet()
    ls.lineWidth=lineWidth
    ls.setVertices(npa([[-length,0,0],[length,0,0],[0,-length,0],[0,length,0],[0,0,-length],[0,0,length]], dtype='float32'))
    ls.addLine(0,1)
    ls.addLine(2,3)
    ls.addLine(4,5)
    ls.setColors(npa([x_color, y_color, z_color], dtype='float32'))
    ls.resizeColorIndicesForNumLines(ls.numLines)
    ls.setLineColor(0, 0)
    ls.setLineColor(1, 1)
    ls.setLineColor(2, 2)
    res=cutil.SgPosTransform()
    res.addChild(ls)
    if coords is not None:
        res.setPosition(coords.cnoidPosition)
    if wrapped:
        res = coordsWrapper(res, original_object=ls)
    return res

def makeLineAlignedShape(start, end, size=0.001, shape='box', verbose=False, **kwargs):
    """Makeing object which is aligned with a desginated line

    Args:
        start (numpy.ndarray) : 3D vector representing start-point of a line
        end (numpy.ndarray) : 3D vector representing end-point of a line
        size (float) : Size of generated object (radius or edge length)
        shape (str) : Type of Shape, 'box', 'cylinder', 'capsule', 'cone'
        verbose (bool, default=False) : If True, printing debug message
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    midp = (start + end)*0.5
    direction = (end - start)
    length=numpy.linalg.norm(direction)
    coordinates.normalizeVector(direction)
    if shape in ('cylinder', 'Cylinder'):
        obj = makeCylinder(size, length, **kwargs)
    elif shape in ('capsule', 'Capsule'):
        obj = makeCapsule(size, length, **kwargs)
    elif shape in ('cone', 'Cone'):
        obj = makeCone(size, length, **kwargs)
    else:
        obj = makeBox(size, length, size, **kwargs)
    ##
    obj.locate(midp, coordinates.wrt.world)
    v0 = numpy.cross(direction, coordinates.Y)
    if numpy.linalg.norm(v0) < 0.5:
        v0 = numpy.cross(direction, coordinates.Z)
    coordinates.normalizeVector(v0)
    v2 = numpy.cross(v0, direction)
    rot=numpy.column_stack([v0, direction, v2])
    obj.orient_with_matrix(rot, coordinates.wrt.world)
    if verbose:
        print('length: {}'.format(length))
        print('direction: {}'.format(direction))
        print('mid:  {}'.format(midp))
        print('rot: {}'.format(rot))
    return obj

def makeParallelogram(height, length, width, offset=None, angle=None, front_cut=None, back_cut=None, **kwargs):
    """Making a geometry, 'Parallelogram'

    Args:
        height (float) : Height of the geometry
        length (float) : Length of bottom edge
        width (float) : Width of the geometry
        offset (float, optional) : Offset of a top edge and a bottom edge
        angle (float, optional) : range is [0, PI]
        front_cut (float, optional) : Cut from front-side if angle > 0
        back_cut (float, optional) : Cut from back-side if angle > 0
        **kwargs : Passing to irsl_choreonoid.make_shapes.makeExtrusion

    Retuns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    if angle is not None:
        offset = height / math.tan(angle)
    else:
        if offset is None:
            raise Exception('offset or angle should be set')
        angle = math.atan2(height, offset)
    if (front_cut is not None or back_cut is not None) and angle > 0.0:
        if front_cut is not None:
            cross_s = [ [front_cut, 0], [length, 0], ]
        else:
            cross_s = [ [        0, 0], [length, 0], ]
        if back_cut is not None:
            cross_s += [ [(length + offset - back_cut), (offset - back_cut) * math.tan(angle)], [(length + offset - back_cut), height], [offset, height], ]
        else:
            cross_s += [ [(length + offset), height], [offset, height], ]
        if front_cut is not None:
            cross_s += [ [front_cut, front_cut * math.tan(angle) ], [front_cut, 0] ]
        else:
            cross_s += [ [0, 0] ]
    else:
        cross_s = [ [0, 0], [length, 0], [(length + offset), height], [offset, height], [0, 0 ]]
    return makeExtrusion(cross_s, spine=[[0, width*-0.5, 0], [0, width*0.5, 0]], **kwargs)

def makeBasket(width, height, tall, thickness = 0.1, bottom_thickness = 0.1, wrapped=True, rawShape=False, **kwargs):
    """Making basket shape

    Args:
        width (float) : width of bottom plate (x-axis)
        height (float) : height of bottom plate (y-axis)
        tall (float) : tall of basket
        thickness (float, default = 0.1): thickness of walls
        bottom_thickness (float, default = 0.1): thickness of bottom plate
        wrapped (boolean, default=True) : Just passing to makeBox
        rawShape(boolean, default=False) : Just passing to makeBox
        kwargs ( dict[str, param] ) : Extra keyword arguments passing to makeBox

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    wall0 = makeBox(width, thickness, tall, **kwargs).translate(npa([0,  height*0.5, tall*0.5]))
    wall1 = makeBox(width, thickness, tall, **kwargs).translate(npa([0, -height*0.5, tall*0.5]))
    wall2 = makeBox(thickness, height - thickness, tall, **kwargs).translate(npa([ 0.5*(width- thickness), 0, tall*0.5]))
    wall3 = makeBox(thickness, height - thickness, tall, **kwargs).translate(npa([-0.5*(width- thickness), 0, tall*0.5]))
    bottm = makeBox(width - 2*thickness, height - thickness, bottom_thickness, **kwargs).translate(npa([0, 0, bottom_thickness*0.5]))
    if rawShape:
        sg = cutil.SgGroup()
    else:
        sg = cutil.SgPosTransform()
    sg.addChild(wall0.target)
    sg.addChild(wall1.target)
    sg.addChild(wall2.target)
    sg.addChild(wall3.target)
    sg.addChild(bottm.target)
    if rawShape:
        return sg
    ret = sg
    if wrapped:
        ret = coordsWrapper(sg, original_object=sg)
    return ret

def makeTableSingleLeg(width, height, tall, thickness = 0.05, bottom_thickness = 0.04, bottom_width = 0.4, bottom_height = 0.4, leg_size = 0.1, wrapped=True, rawShape=False, **kwargs):
    """Making table with single leg

    Args:
        width (float) : width of top plate (x-axis)
        height (float) : height of top plate (y-axis)
        tall (float) : tall of table
        thickness (float, default = 0.05): thickness of top plate
        bottom_thickness (float, default = 0.04): thickness of bottom plate
        bottom_width (float, default = 0.4): width of bottom plate
        bottom_height (float, default = 0.4): height of bottom plate
        leg_size (float, default = 0.1): leg size
        wrapped (boolean, default=True) : Just passing to makeBox
        rawShape(boolean, default=False) : Just passing to makeBox
        kwargs ( dict[str, param] ) : Extra keyword arguments passing to makeBox

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    plate = makeBox(width, height, thickness, **kwargs).translate(npa([0, 0, tall - 0.5*thickness]))
    leg = makeBox(leg_size, leg_size, tall - thickness - bottom_thickness, **kwargs).translate(npa([0, 0, 0.5*(tall - thickness + bottom_thickness)]))
    bottom = makeBox(bottom_width, bottom_height, bottom_thickness, **kwargs).translate(npa([0, 0, 0.5*bottom_thickness]))
    if rawShape:
        sg = cutil.SgGroup()
    else:
        sg = cutil.SgPosTransform()
    sg.addChild(plate.target)
    sg.addChild(leg.target)
    sg.addChild(bottom.target)
    if rawShape:
        return sg
    ret = sg
    if wrapped:
        ret = coordsWrapper(sg, original_object=sg)
    return ret

def makeTable4Legs(width, height, tall, thickness = 0.05, leg_size = 0.1, wrapped=True, rawShape=False, **kwargs):
    """Making table with 4 legs

    Args:
        width (float) : width of top plate (x-axis)
        height (float) : height of top plate (y-axis)
        tall (float) : tall of table
        thickness (float, default = 0.05): thickness of top plate
        leg_size (float, default = 0.1): leg size
        wrapped (boolean, default=True) : Just passing to makeBox
        rawShape(boolean, default=False) : Just passing to makeBox
        kwargs ( dict[str, param] ) : Extra keyword arguments passing to makeBox

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    plate = makeBox(width, height, thickness, **kwargs).translate(npa([0, 0, tall - 0.5*thickness]))
    leg0  = makeBox(leg_size, leg_size, tall - thickness, **kwargs).translate(npa([ 0.5*(width - leg_size), 0.5*(height - leg_size), 0.5*(tall - thickness)]))
    leg1  = makeBox(leg_size, leg_size, tall - thickness, **kwargs).translate(npa([-0.5*(width - leg_size), 0.5*(height - leg_size), 0.5*(tall - thickness)]))
    leg2  = makeBox(leg_size, leg_size, tall - thickness, **kwargs).translate(npa([ 0.5*(width - leg_size),-0.5*(height - leg_size), 0.5*(tall - thickness)]))
    leg3  = makeBox(leg_size, leg_size, tall - thickness, **kwargs).translate(npa([-0.5*(width - leg_size),-0.5*(height - leg_size), 0.5*(tall - thickness)]))
    if rawShape:
        sg = cutil.SgGroup()
    else:
        sg = cutil.SgPosTransform()
    sg.addChild(plate.target)
    sg.addChild(leg0.target)
    sg.addChild(leg1.target)
    sg.addChild(leg2.target)
    sg.addChild(leg3.target)
    if rawShape:
        return sg
    ret = sg
    if wrapped:
        ret = coordsWrapper(sg, original_object=sg)
    return ret

def makeRoundTable(radius, tall, thickness = 0.05, bottom_thickness = 0.04, bottom_width = 0.4, bottom_height = 0.4, leg_size = 0.1, wrapped=True, rawShape=False, **kwargs):
    """Making round table with single leg

    Args:
        radius (float) : width of top plate (x-axis)
        tall (float) : tall of table
        thickness (float, default = 0.05): thickness of top plate
        bottom_thickness (float, default = 0.04): thickness of bottom plate
        bottom_width (float, default = 0.4): width of bottom plate
        bottom_height (float, default = 0.4): height of bottom plate
        leg_size (float, default = 0.1): leg size
        wrapped (boolean, default=True) : Just passing to makeBox
        rawShape(boolean, default=False) : Just passing to makeBox
        kwargs ( dict[str, param] ) : Extra keyword arguments passing to makeBox

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    plate = makeCylinder(radius, thickness, **kwargs).translate(npa([0, 0, tall - 0.5*thickness])).rotate(PI/2, coordinates.X)
    leg = makeBox(leg_size, leg_size, tall - thickness - bottom_thickness, **kwargs).translate(npa([0, 0, 0.5*(tall - thickness + bottom_thickness)]))
    bottom = makeBox(bottom_width, bottom_height, bottom_thickness, **kwargs).translate(npa([0, 0, 0.5*bottom_thickness]))
    if rawShape:
        sg = cutil.SgGroup()
    else:
        sg = cutil.SgPosTransform()
    sg.addChild(plate.target)
    sg.addChild(leg.target)
    sg.addChild(bottom.target)
    if rawShape:
        return sg
    ret = sg
    if wrapped:
        ret = coordsWrapper(sg, original_object=sg)
    return ret

def makeWireframeCone(wrapped=True, rawShape=False, scale=[1, 1, 1], **kwargs):
    """Making cone with lines

    Args:
        wrapped (boolean, default=True) : Just passing to makeBox
        rawShape(boolean, default=False) : Just passing to makeBox
        kwargs ( dict[str, param] ) : Extra keyword arguments passing to makeBox

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    lst = [ [ 0., 0., 0.],
            [ 0.5*scale[0],  0.5*scale[1], scale[2]],
            [ 0.5*scale[0], -0.5*scale[1], scale[2]],
            [-0.5*scale[0], -0.5*scale[1], scale[2]],
            [-0.5*scale[0],  0.5*scale[1], scale[2]] ]
    line_indices = [ (0, 1), (0, 2), (0, 3), (0, 4),
                     (1, 2), (2, 3), (3, 4), (4, 1) ]
    return makeLines(lst, line_indices=line_indices, wrapped=wrapped, rawShape=rawShape, **kwargs)

def makeBoxFromBoundingBox(bbox, line=False, wrapped=True, rawShape=False, **kwargs):
    """Making box with the same size and position passed bounding-box

    Args:
        bbox ( cnoid.Util.BoundingBox or object has 'boundingBox' method ) :
        line (boolean, default=False): If True, BoundingBox is shown with line
        wrapped (boolean, default=True) : Just passing to makeBox
        rawShape(boolean, default=False) : Just passing to makeBox
        kwargs ( dict[str, param] ) : Extra keyword arguments passing to makeBox

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    """
    if type(bbox) is cutil.BoundingBox:
        pass
    elif hasattr(bbox, 'boundingBox'):
        bbox = bbox.boundingBox()
    else:
        raise Exception('{} (type: {}) is not BoundingBox and does not has method: boundingBox'.format(bbox, type(bbox)))
    if line:
        m0 = bbox.min()
        m1 = bbox.max()
        lst = [ [ m0[0], m0[1], m0[2] ],
                [ m1[0], m0[1], m0[2] ],
                [ m1[0], m1[1], m0[2] ],
                [ m0[0], m1[1], m0[2] ],
                [ m0[0], m0[1], m1[2] ],
                [ m1[0], m0[1], m1[2] ],
                [ m1[0], m1[1], m1[2] ],
                [ m0[0], m1[1], m1[2] ] ]
        line_indices = [ (0, 1), (1, 2), (2, 3), (3, 0),
                         (4, 5), (5, 6), (6, 7), (7, 4),
                         (0, 4), (1, 5), (2, 6), (3, 7) ]
        return makeLines(lst, line_indices=line_indices, wrapped=wrapped, rawShape=rawShape, **kwargs)
    else:
        sz = bbox.size()
        cds = coordinates(bbox.center())
        return makeBox(sz[0], sz[1], sz[2], coords=cds, wrapped=wrapped, rawShape=rawShape, **kwargs)

def _crossPoint2D(p0, n0, p1, n1):
    """Getting crossing point

    Args:
        p0 (numpy.array) : 2D point
        n0 (numpy.array) : 2D vector
        p1 (numpy.array) : 2D point
        n1 (numpy.array) : 2D vector

    Returns:
        numpy.array : Cross point (2D vector) of p0 + a * n0 = p1 + b * n1
    """
    ## n0.dot(n1)
    N = numpy.linalg.inv(numpy.vstack([n0, n1]).transpose())
    res = N.dot(p1 - p0)
    return res[0] * n0 + p0

def makeLineAlignedWall(points, height=1.0, thickness=0.1, **kwargs):
    """Making a wall which is aligned with a multi-segment line
    points on XY plane, extrude them by Z-axis(height)

    Args:
        points ( list[numpy.array] ) : List of 2D point, representing a line on XY-plane
        height (float, default=1.0) : Height of the wall
        thickness (float, default=0.1) : Thickness of the wall

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Created object as a node of SceneGraph or wrapped class for interactive programming

    Examples:

        >>> points=[]
        >>> NN = 36
        >>> for idx in range(NN+1):
        >>>     xx = 2*PI/NN*idx
        >>>     ss = math.sin(xx)
        >>>     points.append(fv(xx, ss))
        >>> sp = mkshapes.makeLineAlignedWall(points)

    """
    n_lst = []
    for idx in range(len(points) - 1):
        ntmp = (points[idx+1] - points[idx])
        ntmp /= numpy.linalg.norm(ntmp)
        n_lst.append(ntmp)
    Amat = npa([[0., -thickness], [ thickness, 0.]])
    Bmat = npa([[0.,  thickness], [-thickness, 0.]])
    plst = []
    plst.append(points[0] + Bmat.dot(n_lst[0])) ##
    for idx in range(1, len(points)-1):
        p = _crossPoint2D(points[idx]   + Bmat.dot(n_lst[idx]  ), n_lst[idx],
                          points[idx-1] + Bmat.dot(n_lst[idx-1]), n_lst[idx-1])
        plst.append(p)
    plst.append(points[-1] + Bmat.dot(n_lst[-1])) ##
    ###
    plst.append(points[-1] + Amat.dot(n_lst[-1])) ##
    for idx in reversed(range(1, len(points)-1)):
        p = _crossPoint2D(points[idx]   + Amat.dot(n_lst[idx]  ), n_lst[idx],
                          points[idx-1] + Amat.dot(n_lst[idx-1]), n_lst[idx-1])
        plst.append(p)
    plst.append(points[0] + Amat.dot(n_lst[0])) ##
    #plst.append(plst[0])
    qlst = []
    for p in reversed(plst):
        qlst.append(npa([p[0], -p[1]]))
    qlst.append(qlst[0])
    return makeExtrusion(qlst, [[0., 0., 0.], [0., 0., height]], **kwargs)
##
## Function for exporting
##
def exportMesh(fname, sg_node, meshScale=None, verbose=False, generatePrimitiveMesh=True, ignoreURDFPrimitive=False, outputType=None, expandVertices=False):
    """Exporting SgNode as a mesh file (using Assimp)

    Args:
        fname (str) : File name to be saved
        sg_node ( cnoid.Util.SgNode ) : Root node of scene to be saved
        meshScale (float, optional) : 
        verbose ( boolean, default=False ) :
        generatePrimitiveMesh ( boolean, default=True ) :
        ignoreURDFPrimitive (boolean, default=False ) :
        outputType (str, optional) : 

    """
    if isinstance(sg_node, coordsWrapper):
        if isinstance(sg_node.target, cutil.SgNode):
            sg_node=sg_node.target
        else:
            sg_node=sg_node.object
    wt = cnoid.AssimpPlugin.AssimpSceneWriter()
    wt.setMessageSinkStdErr()

    if verbose:
        wt.setVerbose(True)
    wt.generatePrimitiveMesh(generatePrimitiveMesh)
    wt.ignoreURDFPrimitive(ignoreURDFPrimitive)
    if outputType is not None:
        wt.outputType = outputType
    wt.setExpandVertices(expandVertices)
    if meshScale is not None:
        scl_=cnoid.Util.SgScaleTransform()
        scl_.setScale(meshScale)
        scl_.addChild(sg_node)
        sg_node = scl_

    return wt.writeScene(fname, sg_node)

def exportScene(fname, sg_node, meshScale=None, exportMesh=False, **kwargs):
    """Exporting SgNode as .scen file

    Args:
        fname (str) : File name to be saved
        sg_node ( cnoid.Util.SgNode ) : Root node of scene to be saved
        meshScale (float, optional) : 
        exportMesh (boolean, default=False) : Exporting mesh instead of primitive type
        kwargs ( dict[str, param] ) : Extra keyword arguments for using to execute ''StdSceneWriter.<keyword> = <value>''

    """
    if isinstance(sg_node, coordsWrapper):
        if isinstance(sg_node.target, cutil.SgNode):
            sg_node=sg_node.target
        else:
            sg_node=sg_node.object
    wt = cutil.StdSceneWriter()
    wt.setMessageSinkStdErr()

    for k,v in kwargs.items():
        exec('wt.{} = {}'.format(k, v))

    if exportMesh:
        res = extractShapes(sg_node)
        if len(res) > 0:
            # v = npa([0.,0.,0.])
            for sp, cds in res:
                #sp.shape.mesh.translate(v)
                sp.mesh.setMeshType()

    if meshScale is not None:
        scl_=cnoid.Util.SgScaleTransform()
        scl_.setScale(meshScale)
        scl_.addChild(sg_node)
        sg_node = scl_

    return wt.writeScene(fname, sg_node)

### util
def addUriToShape(sg_node, base_name='mesh', base_uri='file:///tmp', allInOne=False):
    """Exporting SgNode as .scen file

    Args:
        sg_node ( cnoid.Util.SgNode ) : Root node of scene to be add uri
        base_name ( str ) : Base name of uri
        base_uri ( str ) : Base uri to be add

    Note:
       uri : {base_uri}/{base_name}_{counter}

    """
    extracts = extractShapes(sg_node)
    if allInOne:
        hasUri=True
        for shape, coords in extracts:
            if shape.mesh.primitiveType == cutil.SgMesh.MeshType and not shape.mesh.hasUri():
                hasUri=False
                break
        if hasUri: ## all shapes has uri
            ## do nothing ??
            pass
        else:
            withoutUri=True
            for shape, coords in extracts:
                if shape.mesh.primitiveType == cutil.SgMesh.MeshType and shape.mesh.hasUri():
                    withoutUri = False
                    break;
            if withoutUri:
                ### all shapes do not have uri
                sg_node.setUri(base_name, '{}/{}'.format(base_uri, base_name))
            else:
                ### some have uri, some do not
                cntr = 0
                for shape, coords in extracts:
                    if shape.mesh.primitiveType == cutil.SgMesh.MeshType and not shape.mesh.hasUri():
                        fn = '{}_{}'.format(base_name, cntr)
                        shape.mesh.setUri(fn, '{}/{}'.format(base_uri, fn))
                        cntr += 1
        return

    cntr = 0
    for shape, coords in extracts:
        if shape.mesh.primitiveType == cutil.SgMesh.MeshType:
            fn = '{}_{}'.format(base_name, cntr)
            shape.mesh.setUri(fn, '{}/{}'.format(base_uri, fn))
            cntr += 1
