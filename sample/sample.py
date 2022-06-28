
# DISPLAY=:0 choreonoid
import irsl_choreonoid.sample_robot as sr
import irsl_choreonoid.robot_util as ru
import numpy as np
import cnoid.IRSLUtil as iu
import math

## ロボットモデルの初期化
rr = sr.init_sample_robot()
## 姿勢を与える
rr.set_pose('default')
## 画面の更新
rr.flush()
## 左右脚先の中点
cds0 = rr.foot_mid_coords()

## 左右脚先の中点を与えた座標に一致させる（以下の場合は原点）
rr.fix_leg_to_coords(np.identity(4))
rr.flush()## 画面の更新

## 腰を前後左右に動かして重心を左右脚先の中点上に一致させる
rr.move_centroid_on_foot()
rr.flush()## 画面の更新
cds1 = rr.foot_mid_coords()## 左右脚先の中点

rr.robot.mass ## ロボットの質量
rr.robot.calcCenterOfMass() ## 重心の計算
rr.robot.centerOfMass ## 計算された重心位置

## 矢印の表示（重心）
mass_coords = ru.DrawCoords()
mass_coords.draw(iu.cnoidPosition(rr.robot.centerOfMass))

## 矢印の表示（左手）
larm_coords = ru.DrawCoords()
larm_coords.draw(rr.larm_end_effector())

# print joints
#for j in rr.joint_list():
#    print('{}:{:f}'.format(j.name, j.q * 180 / math.pi))
