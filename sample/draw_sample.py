import cnoid.Base
import cnoid.Util
import numpy as np
import irsl_choreonoid.make_shapes as mkshapes
from cnoid.DrawInterface import GeneralDrawInterface as GDI
from cnoid.IRSLCoords import coordinates

## インターフェースのためのinstanceを作る
gdi = GDI()

# MeshGenerator(形状メッシュを生成)のインスタンス
sg = mkshapes.makeBox(np.array([0.3,0.3,0.0002]), color=np.array([5.,0.,1.]))

# 画面に表示させる
gdi.addObject(sg, True)
gdi.viewAll()

# move the object by coordinates
cds = coordinates()
cds.translate(np.array([0, 0, 1.0]))
cds.rotate(0.4, np.array([0.1, 0.2, 0.3]))
gdi.setOrigin(cds)
gdi.flush()

# 画面から除く
gdi.remove_object(sg_shape, True)
