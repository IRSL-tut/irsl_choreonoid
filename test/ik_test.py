# PYTHONPATH=$PYTHONPATH:$(dirname $(which choreonoid))/../lib/choreonoid-2.0/python python3 ik_test.py

import cnoid.IRSLCoords as ic
import irsl_choreonoid.sample_robot as sr
import numpy as np
import math

## for Test
import unittest

class TestRobotModel(unittest.TestCase):
    def test_set_pose(self):
        rr = sr.SampleRobot()
        av_org = rr.angleVector()

        rr.setDefaultPose()
        av_default = rr.angleVector()

        diff = np.linalg.norm(av_org - av_default)

        self.assertTrue(diff > 0.1)

    def test_end_effector(self):
        rr = sr.SampleRobot()
        rr.setDefaultPose()
        ## TODO check
        rhcds = rr.rarm
        lhcds = rr.larm
        rlcds = rr.rleg
        llcds = rr.lleg

    def test_mid_coords(self):
        rr = sr.SampleRobot()

        ## TODO check
        cds0 = rr.footMidCoords()

    def test_fix_coords(self):
        rr = sr.SampleRobot()

        rot = ic.angleAxisNormalized(0.2, np.array([1.0, 2.0, 3.0]))
        pos = np.array([1.0, 2.0, 3.0])
        cds = ic.coordinates(pos, rot)
        rr.rootCoords(cds)

        cds_org = rr.footMidCoords()

        rr.fixLegToCoords(ic.coordinates()) ## coordinates

        cds_fix = rr.footMidCoords()

        diff_cds = cds_org.transformation(cds_fix)
        self.assertTrue(np.linalg.norm(diff_cds.pos) > 3.2)

        self.assertTrue(np.linalg.norm(cds_fix.pos) < 0.0001)
        ang = cds_fix.getRotationAngle()
        self.assertTrue(ang[3] < 0.0001)

    def test_move_centroid(self):
        rr = sr.SampleRobot()
        rr.setDefaultPose()
        rr.fixLegToCoords(ic.coordinates()) ## coordinates

        succ, loop = rr.rleg.move(np.array([0.05, 0, 0]))
        self.assertTrue(succ)

        succ, loop = rr.lleg.move(np.array([0.05, 0, 0]))
        self.assertTrue(succ)

        rr.fixLegToCoords(ic.coordinates()) ## coordinates
        cds_org = rr.footMidCoords()

        succ, loop = rr.moveCentroidOnFoot(debug=True)
        self.assertTrue(succ)

        cds_moved = rr.footMidCoords()## 左右脚先の中点
        self.assertTrue(np.linalg.norm(cds_moved.pos) < 0.001)

        cog = np.array(rr.centerOfMass)
        cog[2] = 0.0
        self.assertTrue(np.linalg.norm(cog) < 0.001)

    #def test_fail(self):
    #    self.assertTrue(False)

if __name__ == '__main__':
    unittest.main()
