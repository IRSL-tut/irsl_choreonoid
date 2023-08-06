# PYTHONPATH=$PYTHONPATH:$(dirname $(which choreonoid))/../lib/choreonoid-2.0/python python3 ik_test.py

import cnoid.IRSLCoords as ic
import irsl_choreonoid.sample_robot as sr
import numpy as np
import math

## for Test
import unittest

class TestRobotModel(unittest.TestCase):
    def test_set_pose(self):
        rr = sr.init_sample_robot()
        rr.flush()
        av_org = rr.angleVector()

        rr.set_pose('default')
        rr.flush()
        av_default = rr.angleVector()

        diff = np.linalg.norm(av_org - av_default)

        self.assertTrue(diff > 0.1)

    def test_end_effector(self):
        rr = sr.init_sample_robot()
        rhcds = rr.end_effector('rarm')
        lhcds = rr.end_effector('larm')
        rlcds = rr.end_effector('rleg')
        llcds = rr.end_effector('lleg')

    def test_mid_coords(self):
        rr = sr.init_sample_robot()
        rr.flush()

        cds0 = rr.foot_mid_coords()

    def test_fix_coords(self):
        rr = sr.init_sample_robot()

        rot = ic.angleAxisNormalized(0.2, np.array([1.0, 2.0, 3.0]))
        pos = np.array([1.0, 2.0, 3.0])
        cds = ic.coordinates(pos, rot)
        rr.robot.rootLink.setCoords(cds)
        rr.flush()

        cds_org = rr.foot_mid_coords()

        rr.fix_leg_to_coords(ic.coordinates()) ## coordinates
        rr.flush()

        cds_fix = rr.foot_mid_coords()

        diff_cds = cds_org.transformation(cds_fix)
        self.assertTrue(np.linalg.norm(diff_cds.pos) > 3.2)

        self.assertTrue(np.linalg.norm(cds_fix.pos) < 0.0001)
        ang = cds_fix.getRotationAngle()
        self.assertTrue(ang[3] < 0.0001)

    def test_move_centroid(self):
        rr = sr.init_sample_robot()
        rr.flush()
        rr.set_pose('default')
        rr.flush()
        rr.fix_leg_to_coords(ic.coordinates()) ## coordinates
        rr.flush()
        cds_org = rr.foot_mid_coords()

        loop = rr.move_centroid_on_foot_cnoid(debug=True)
        self.assertNotEqual(loop, 20)
        rr.flush()
        cds_moved = rr.foot_mid_coords()## 左右脚先の中点
        self.assertTrue(np.linalg.norm(cds_moved.pos) < 0.001)
        cog = rr.robot.calcCenterOfMass()
        cog[2] = 0.0
        self.assertTrue(np.linalg.norm(cog) < 0.001)

    #def test_fail(self):
    #    self.assertTrue(False)

if __name__ == '__main__':
    unittest.main()
