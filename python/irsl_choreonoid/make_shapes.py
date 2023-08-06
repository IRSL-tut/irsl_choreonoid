import numpy as np
import cnoid.AssimpPlugin
import cnoid.Util

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
    mat = cnoid.Util.SgMaterial()
    val = __gets(('AmbientIntensity', 'ambientintensity', 'Intensity', 'intensity', 'ambient-intensity'), kwargs)
    if val is not None:
        mat.setAmbientIntensity(val)
        valueset = True

    val = __gets(('DiffuseColor', 'diffusecolor', 'diffuse-color', 'diffuse'), kwargs)
    if val is not None:
        mat.setDiffuseColor(np.array(val))
        valueset = True

    val = __gets(('EmissiveColor', 'emissivecolor', 'emissive-color', 'emissive'), kwargs)
    if val is not None:
        mat.setEmissiveColor(np.array(val))
        valueset = True

    val = __gets(('SpecularExponent', 'specularexponent', 'specular-exponent'), kwargs)
    if val is not None:
        mat.setSpecularExponent(val)
        valueset = True

    val = __gets(('SpecularColor', 'specularcolor', 'specular-color', 'specular'), kwargs)
    if val is not None:
        mat.setSpecularColor(np.array(val))
        valueset = True

    val = __gets(('Transparency', 'transparency', 'Transparent', 'transparent'), kwargs)
    if val is not None:
        mat.setTransparency(val)
        valueset = True

    val = __gets(('color', 'Color'), kwargs)
    if val is not None:
        mat.setAmbientIntensity(1.0)
        mat.setDiffuseColor(np.array(val) * 0.7)
        mat.setEmissiveColor(np.array(val) * 0.3)
        valueset = True

    if valueset:
        return mat
    return None

def parseMeshGeneratorOption(mg, **kwargs):
    val = __gets(('DivisionNumber'), kwargs)
    if val is not None:
        mg.setDivisionNumber(val)
    val = __gets(('NormalGenerationEnabled'), kwargs)
    if val is not None:
        mg.setNormalGenerationEnabled(val)
    val = __gets(('setBoundingBoxUpdateEnabled'), kwargs)
    if val is not None:
        mg.setBoundingBoxUpdateEnabled(val)

def __extractShape(sg_node):
    res = []
    if type(sg_node) is cnoid.Util.SgShape:
        res.append(sg_node)
    elif hasattr(sg_node, 'numChildren'):
        for idx in range(sg_node.numChildren):
            res += __extractShape(sg_node.getChild(idx))
    return res

def loadScene(fname, wrapped=True, **kwargs):
    """Loading scene(wrl, scene, ...) file using cnoid.Util.SceneLoader

    Args:
        fname (str) : File name to be loaded
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    """
    ld = cnoid.Util.SceneLoader()
    ld.setMessageSinkStdErr()

    sg = ld.load(fname)
    shapes = __extractShape(sg)

    mat = generateMaterial(**kwargs)

    if mat is not None:
        for shape in shapes:
            shape.setMaterial(mat)

    ret = None
    if type(sg) is cnoid.Util.SgPosTransform:
        ret = sg
    else:
        ret = cnoid.Util.SgPosTransform()
        ret.addChild(sg)
    if wrapped:
        ret = coordsWrapper(ret)
    return ret

def loadMesh(fname, wrapped=True, **kwargs):
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

    ret = cnoid.Util.SgPosTransform()
    ret.addChild(sg)
    if wrapped:
        ret = coordsWrapper(ret)
    return ret

def __genShape(mesh, wrapped=True, **kwargs):
    sg = cnoid.Util.SgShape()
    sg.setMesh(mesh)

    mat = generateMaterial(**kwargs)

    if mat is not None:
        sg.setMaterial(mat)

    ret = cnoid.Util.SgPosTransform()
    ret.addChild(sg)
    if wrapped:
        ret = coordsWrapper(ret)
    return ret

def makeBox(x, y = None, z = None, wrapped=True, **kwargs):
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
    mg = cnoid.Util.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    if type(x) is np.ndarray:
        mesh = mg.generateBox(x)
    elif y is not None and z is not None:
        mesh = mg.generateBox(np.array([x, y, z]))
    elif type(x) is int or type(x) is float:
        mesh = mg.generateBox(np.array([x, x, x]))
    else:
        raise Exception(f'Invalid arguments x: {x}, y: {y}, z: {z}')

    if mesh is None:
        raise Exception(f'Generating mesh was failed x: {x}, y: {y}, z: {z}')

    return __genShape(mesh, wrapped=wrapped, **kwargs)

def makeCylinder(radius, height, wrapped=True, **kwargs):
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
    mg = cnoid.Util.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateCylinder(radius, height)
    return __genShape(mesh, wrapped=wrapped, **kwargs)

def makeSphere(radius, wrapped=True, **kwargs):
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
    mg = cnoid.Util.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateSphere(radius)
    return __genShape(mesh, wrapped=wrapped, **kwargs)

def makeCone(radius, height, wrapped=True, **kwargs):
    """Making 'Cone' shape using cnoid.Util.MeshGenerator

    Args:
        radius (float) : Radius of the cone
        height (float) : Height of the cone
        wrapped (boolean, default = True) : If True, the loaded scene is wrapped by irsl_choreonoid.irsl_draw_object.coordsWrapper
        kwargs ( dict[str, param] ) : Keywords for generating material and mesh

    Returns:
        cnoid.Util.SgPosTransform or irsl_choreonoid.irsl_draw_object.coordsWrapper : Loaded scene as a node of SceneGraph or wrapped class for interactive programming

    """
    mg = cnoid.Util.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateCone(radius, height)
    return __genShape(mesh, wrapped=wrapped, **kwargs)

def makeCapsule(radius, height, wrapped=True, **kwargs):
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
    mg = cnoid.Util.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateCapsule(radius, height)
    return __genShape(mesh, wrapped=wrapped, **kwargs)

def makeTorus(radius, corssSectionRadius, beginAngle = None, endAngle = None, wrapped=True, **kwargs):
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
    mg = cnoid.Util.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    if beginAngle is not None and endAngle is not None:
        mesh = mg.generateTorus(radius, corssSectionRadius, beginAngle, endAngle)
    else:
        mesh = mg.generateTorus(radius, corssSectionRadius)

    return __genShape(mesh, wrapped=wrapped, **kwargs)

### not implemented
def makeExtrusion(extrusion, wrapped=True, **kwargs):
    mg = cnoid.Util.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateExtrusion(extrusion)
    return __genShape(mesh, wrapped=wrapped, **kwargs)

def makeElevationGrid(elevation_grid, wrapped=True, **kwargs):
    mg = cnoid.Util.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateElevationGrid(elevation_grid)
    return __genShape(mesh, wrapped=wrapped, **kwargs)

def makeCoords(coords): ## LineArray
    pass
