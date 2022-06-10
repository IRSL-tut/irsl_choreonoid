import cnoid.Base
import cnoid.Util
import cnoid.DrawInterface as di
import numpy as np

di_instance = di.DrawInterface()

mg = cnoid.Util.MeshGenerator()
bx_mesh = mg.generateBox(np.array([1,1,1]))

sg_shape = cnoid.Util.SgShape()
sg_shape.setMesh(bx_mesh)

sg_mat = cnoid.Util.SgMaterial()
sg_mat.setAmbientIntensity(1)
sg_mat.setDiffuseColor(np.array([0,0,0.5]))
sg_mat.setEmissiveColor(np.array([0,0,0.2]))
sg_mat.setSpecularExponent(0.0)
sg_mat.setSpecularColor(np.array([0,0,0]))
sg_mat.setTransparency(0.0)

sg_shape.setMaterial(sg_mat)

di_instance.add_object(sg_shape, True)
