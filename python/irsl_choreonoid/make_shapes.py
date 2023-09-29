import numpy
from numpy import array as npa
import cnoid.AssimpPlugin
import cnoid.Util as cutil
from math import pi as PI

from .irsl_draw_object import *

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
        mat.setDiffuseColor(npa(val, dtype='float64'))
        valueset = True

    val = __gets(('EmissiveColor', 'emissivecolor', 'emissive-color', 'emissive'), kwargs)
    if val is not None:
        mat.setEmissiveColor(npa(val, dtype='float64'))
        valueset = True

    val = __gets(('SpecularExponent', 'specularexponent', 'specular-exponent'), kwargs)
    if val is not None:
        mat.setSpecularExponent(val, dtype='float64')
        valueset = True

    val = __gets(('SpecularColor', 'specularcolor', 'specular-color', 'specular'), kwargs)
    if val is not None:
        mat.setSpecularColor(npa(val, dtype='float64'))
        valueset = True

    val = __gets(('Transparency', 'transparency', 'Transparent', 'transparent'), kwargs)
    if val is not None:
        mat.setTransparency(val)
        valueset = True

    val = __gets(('color', 'Color'), kwargs)
    if val is not None:
        mat.setAmbientIntensity(1.0)
        mat.setDiffuseColor(npa(val, dtype='float64') * 0.7)
        mat.setEmissiveColor(npa(val, dtype='float64') * 0.3)
        valueset = True

    if valueset:
        return mat
    return None

def parseMeshGeneratorOption(mg, **kwargs):
    val = __gets(('DivisionNumber',), kwargs)
    if val is not None:
        mg.setDivisionNumber(val)
    val = __gets(('NormalGenerationEnabled',), kwargs)
    if val is not None:
        mg.setNormalGenerationEnabled(val)
    val = __gets(('setBoundingBoxUpdateEnabled',), kwargs)
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

def loadScene(fname, wrapped=True, coords=None, **kwargs):
    """Loading scene(wrl, scene, ...) file using cnoid.Util.SceneLoader

    Args:
        fname (str) : File name to be loaded
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    """
    ld = cutil.SceneLoader()
    ld.setMessageSinkStdErr()

    sg = ld.load(fname)
    shapes = __extractShape(sg)

    mat = generateMaterial(**kwargs)

    if mat is not None:
        for shape in shapes:
            shape.setMaterial(mat)

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

def loadMesh(fname, wrapped=True, coords=None, **kwargs):
    """Loading mesh file using cnoid.AssimpPlugin module

    Args:
        fname (str) : File name to be loaded
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    """
    ld = cnoid.AssimpPlugin.AssimpSceneLoader()
    ld.setMessageSinkStdErr()
    sg = ld.load(fname)
    if sg is None:
        raise Exception(f'Loading mesh was failed : {fname}')

    mat = generateMaterial(**kwargs)

    if mat is not None:
        sg.setMaterial(mat)

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

def __genShape(mesh, wrapped=True, coords=None, **kwargs):
    sg = cutil.SgShape()
    sg.setMesh(mesh)

    mat = generateMaterial(**kwargs)

    if mat is not None:
        sg.setMaterial(mat)

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

def makeBox(x, y = None, z = None, wrapped=True, coords=None, **kwargs):
    """Making 'Box' shape using cnoid.Util.MeshGenerator

    Args:
        x (float) : Length of x-axis, if y and z is None, making 'cube'
        y (float, optional) : Length of y-direction edge
        z (float, optional) : Length of z-direction edge
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    Note:
        Origin of generated shape is the center of it

    """
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    if type(x) is numpy.ndarray or type(x) is list:
        mesh = mg.generateBox(npa(x, dtype='float64'))
    elif y is not None and z is not None:
        mesh = mg.generateBox(npa([x, y, z], dtype='float64'))
    elif type(x) is int or type(x) is float:
        mesh = mg.generateBox(npa([x, x, x], dtype='float64'))
    else:
        raise Exception(f'Invalid arguments x: {x}, y: {y}, z: {z}')

    if mesh is None:
        raise Exception(f'Generating mesh was failed x: {x}, y: {y}, z: {z}')

    return __genShape(mesh, wrapped=wrapped, coords=coords, **kwargs)

def makeCylinder(radius, height, wrapped=True, coords=None, **kwargs):
    """Making 'Cylinder' shape using cnoid.Util.MeshGenerator

    Args:
        radius (float) : Radius of the cylinder
        height (float) : Height of the cylinder
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    Note:
        Center circle with indicated radius on XZ-plane and sweep to both side with half of height, along y-direction

    """
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateCylinder(radius, height)
    return __genShape(mesh, wrapped=wrapped, coords=coords, **kwargs)

def makeSphere(radius, wrapped=True, coords=None, **kwargs):
    """Make 'Sphere' shape using cnoid.Util.MeshGenerator

    Args:
        radius (float) : Radius of the sphere
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    Note:
        Origin of generated shape is the center of it

    """
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateSphere(radius)
    return __genShape(mesh, wrapped=wrapped, coords=coords, **kwargs)

def makeCone(radius, height, wrapped=True, coords=None, **kwargs):
    """Making 'Cone' shape using cnoid.Util.MeshGenerator

    Args:
        radius (float) : Radius of the cone
        height (float) : Height of the cone
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    """
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateCone(radius, height)
    return __genShape(mesh, wrapped=wrapped, coords=coords, **kwargs)

def makeCapsule(radius, height, wrapped=True, coords=None, **kwargs):
    """Makeing 'Capsule' shape using cnoid.Util.MeshGenerator

    Args:
        radius (float) : Radius of the cupsule
        height (float, optional) : Height of the capsule
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    Note:
        Similar dimensions to 'makeCylinder' (bottom cricle is at minus y, cone's tip is at plus y)

    """
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateCapsule(radius, height)
    return __genShape(mesh, wrapped=wrapped, coords=coords, **kwargs)

def makeTorus(radius, corssSectionRadius, beginAngle = None, endAngle = None, wrapped=True, coords=None, **kwargs):
    """Makeing 'Torus' shape using cnoid.Util.MeshGenerator

    Args:
        radius (float) : Outer radius of the torus
        crossSectionRadius (float) : Radius of cross section
        beginAngle (float, optional) : If beginAngle and endAngle is passed, part of whole torus is created
        endAngle (float, optional) : 
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    """
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    if beginAngle is not None and endAngle is not None:
        mesh = mg.generateTorus(radius, corssSectionRadius, beginAngle, endAngle)
    else:
        mesh = mg.generateTorus(radius, corssSectionRadius)

    return __genShape(mesh, wrapped=wrapped, coords=coords, **kwargs)

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

def _makeExtrusion(_extrusion=None, wrapped=True, coords=None, **kwargs):
    if type(_extrusion) is not cutil.MeshGenerator.Extrusion:
        _extrusion = makeExtrusionParam(**kwargs)
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateExtrusion(_extrusion)
    return __genShape(mesh, wrapped=wrapped, coords=coords, **kwargs)

def makeExtrusion(crossSection, spine, wrapped=True, coords=None, **kwargs):
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
        kwargs ( dict[str, param] ) : Keywords for generating material, mesh, and makeExtrusionParam

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    """
    param=_makeExtrusionParam(crossSection, spine, **kwargs)
    return _makeExtrusion(param, wrapped=wrapped, coords=coords, **kwargs)

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

def _makeElevationGrid(_elevation_grid=None, wrapped=True, coords=None, **kwargs):
    if type(_elevation_grid) is not cutil.MeshGenerator.ElevationGrid:
        _elevation_grid = makeElevationParam(**kwargs)
    mg = cutil.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateElevationGrid(_elevation_grid)
    return __genShape(mesh, wrapped=wrapped, coords=coords, **kwargs)

def makeElevationGrid(xDimension, zDimension, xSpacing, zSpacing, height, wrapped=True, coords=None, **kwargs):
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
        kwargs ( dict[str, param] ) : Keywords for generating material, mesh, and makeElevationParam

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    """
    param=_makeElevationParam(xDimension, zDimension, xSpacing, zSpacing, height, **kwargs)
    return _makeElevationGrid(param, wrapped=wrapped, coords=coords, **kwargs)

def make3DAxis(coords=None, wrapped=True, radius=0.15, length=0.8, axisLength=0.3, axisRadius=0.25, axisRatio=None, color=None, scale=None, x_color=[1, 0, 0], y_color=[0, 1, 0], z_color=[0, 0, 1], **kwargs):
    """Makeing '3D-axis' shape using cylinder and cone

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
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

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
    bd0.translate(npa([0,L0/2,0]))
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

def make3DAxisBox(coords=None, wrapped=True, width=0.2, length=0.8, color=None, scale=None, x_color=[1, 0, 0], y_color=[0, 1, 0], z_color=[0, 0, 1], **kwargs):
    """Makeing '3D-axis' shape using box

    Args:
        width (float, default=0.2) :
        length (float, default=0.8) :
        color ( list[float], optional ) :
        scale (float, optional ) :
        coords (cnoid.IRSLCoords.coordinates, optional) :
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

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

def makeCoords(coords=None, wrapped=True, length=1.0, lineWidth=2.0, color=None, x_color=[1,0,0], y_color=[0,1,0], z_color=[0,0,1], **kwargs):
    """Makeing '3D-axis' shape using line

    Args:
        length (float, default=0.8) :
        lineWidth (float, default=2.0) :
        color ( list[float], optional ) :
        scale (float, optional ) :
        coords (cnoid.IRSLCoords.coordinates, optional) :
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    """
    ### material??
    ls=cutil.SgLineSet()
    ls.lineWidth=lineWidth
    ls.setVertices(npa([[0,0,0],[length,0,0],[0,length,0],[0,0,length]], dtype='float32'))
    ls.addLine(0,1)
    ls.addLine(0,2)
    ls.addLine(0,3)
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

def makeCross(coords=None, wrapped=True, length=1.0, lineWidth=2.0, color=None, x_color=[1,0,0], y_color=[0,1,0], z_color=[0,0,1], **kwargs):
    """Makeing '3D-axis' shape using crossing line

    Args:
        length (float, default=0.8) :
        lineWidth (float, default=2.0) :
        color ( list[float], optional ) :
        scale (float, optional ) :
        coords (cnoid.IRSLCoords.coordinates, optional) :
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

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

def makePoints(points, coords=None, wrapped=True, pointSize=1.0, colors=None, colorIndicaes=None, **kwargs):
    """Makeing '3D point cloud' shape

    Args:
        points (numpy.array) : N x 3 matrix (N is number of points)
        pointSize (float, default=1.0) :
        color ( list[float], optional ) :
        colorIndices ( list[float], optional ) :
        scale (float, optional ) :
        coords (cnoid.IRSLCoords.coordinates, optional) :
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    """
    pass
