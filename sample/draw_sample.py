import cnoid.Base
import cnoid.Util
import cnoid.DrawInterface as di
import numpy as np

## インターフェースのためのinstanceを作る
di_instance = di.DrawInterface()

# MeshGenerator(形状メッシュを生成)のインスタンス
mg = cnoid.Util.MeshGenerator()
# Boxのメッシュを作成 (縦横高さが1 [1, 1, 1])
bx_mesh = mg.generateBox(np.array([1,1,1]))

# Shapeのインスタンス（表示用）をつくる
sg_shape = cnoid.Util.SgShape()
# Shapeにメッシュをセット
sg_shape.setMesh(bx_mesh)

# マテリアル（表示色等を設定できる）のインスタンスを作る
sg_mat = cnoid.Util.SgMaterial()
sg_mat.setAmbientIntensity(1)
sg_mat.setDiffuseColor(np.array([0,0,0.5]))
sg_mat.setEmissiveColor(np.array([0,0,0.2]))
sg_mat.setSpecularExponent(0.0)
sg_mat.setSpecularColor(np.array([0,0,0]))
sg_mat.setTransparency(0.0)

# Shapeにマテリアルをセット
sg_shape.setMaterial(sg_mat)

# 画面に表示させる
di_instance.add_object(sg_shape, True)

# 画面から除く
di_instance.remove_object(sg_shape, True)
