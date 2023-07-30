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

    val = __gets(('DiffuseColor', 'diffusecolor', 'diffuse-color'), kwargs)
    if val is not None:
        mat.setDiffuseColor(val)
        valueset = True

    val = __gets(('EmissiveColor', 'emissivecolor', 'emissive-color'), kwargs)
    if val is not None:
        mat.setEmissiveColor(val)
        valueset = True

    val = __gets(('SpecularExponent', 'specularexponent', 'specular-exponent'), kwargs)
    if val is not None:
        mat.setSpecularExponent(val)
        valueset = True

    val = __gets(('SpecularColor', 'specularcolor', 'specular-color'), kwargs)
    if val is not None:
        mat.setSpecularColor(val)
        valueset = True

    val = __gets(('Transparency', 'transparency', 'Transparent', 'transparent'), kwargs)
    if val is not None:
        mat.setTransparency(val)
        valueset = True

    val = __gets(('color', 'Color'), kwargs)
    if val is not None:
        mat.setAmbientIntensity(1.0)
        mat.setDiffuseColor(val * 0.7)
        mat.setEmissiveColor(val * 0.3)
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
    """Loading scene(wrl, scene, ...) file

    Args:

    Returns:

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
    """Loading mesh file

    Args:

    Returns:

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
    """make 'Box' shape

    Args:

    Returns:

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
    """make 'Cylinder' shape

    Args:

    Returns:

    """
    mg = cnoid.Util.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateCylinder(radius, height)
    return __genShape(mesh, wrapped=wrapped, **kwargs)

def makeSphere(radius, wrapped=True, **kwargs):
    """make 'Sphere' shape

    Args:

    Returns:

    """
    mg = cnoid.Util.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateCylinder(radius)
    return __genShape(mesh, wrapped=wrapped, **kwargs)

def makeCone(radius, height, wrapped=True, **kwargs):
    """make 'Cone' shape

    Args:

    Returns:

    """
    mg = cnoid.Util.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateCone(radius, height)
    return __genShape(mesh, wrapped=wrapped, **kwargs)

def makeCapsule(radius, height, wrapped=True, **kwargs):
    """make 'Capsule' shape

    Args:

    Returns:

    """
    mg = cnoid.Util.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    mesh = mg.generateCapsule(radius, height)
    return __genShape(mesh, wrapped=wrapped, **kwargs)

def makeTorus(radius, corssSectionRadius, beginAngle = None, endAngle = None, wrapped=True, **kwargs):
    """make 'Torus' shape

    Args:

    Returns:

    """
    mg = cnoid.Util.MeshGenerator()
    parseMeshGeneratorOption(mg, **kwargs)
    if beginAngle is not None and endAngle is not None:
        mesh = mg.generateTorus(radius, corssSectionRadius, beginAngle, endAngle)
    else:
        mesh = mg.generateTorus(radius, corssSectionRadius)

    return __genShape(mesh, wrapped=wrapped, **kwargs)

### not implemented
def makeExtrusion(**kwargs):
    pass

def makeElevationGrid(**kwargs):
    pass

def makeCoords(coords): ## LineArray
    pass
