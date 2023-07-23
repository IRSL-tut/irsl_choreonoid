### wapper version
import irsl_choreonoid.draw_coords as dc
import irsl_choreonoid.make_shapes as mkshapes
from numpy import array as npa

## インターフェースのためのinstanceを作る
gdi = dc.GeneralDrawInterfaceWrapped()

# Box 形状を作る
sg = mkshapes.makeBox(npa([0.3,0.3,0.0002]), color=npa([5.,0.,1.]))
# 画面に表示させる
gdi.addPyObject(sg, True)
# 移動させる
sg.translate(npa([0, 0, 0.5]))
# 全体表示
sg.viewAll()

# Meshfileをロードする
wb = mkshapes.loadMesh('/home/irsl/1_box.stl')
# 画面に表示させる
gdi.addPyObject(wb, True)
# 回転させる
wb.rotate(0.45, npa([0, 1, 0]))
# 全体表示
sg.viewAll()

# オブジェクトの基準座標系を移動させる
gdi.translate(npa([0, 0, 1.0]))
# 全体表示
sg.viewAll()

# Scenefile(wrl or scene)をロードする
wa = mkshapes.loadScene('/home/irsl/tmp.wrl')
# 画面に表示させる
gdi.addPyObject(wa, True)

# 画面から除く
gdi.removePyObject(wb)
