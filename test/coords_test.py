# PYTHONPATH=$PYTHONPATH:$(dirname $(which choreonoid))/../lib/choreonoid-2.0/python python3 coords_test.py

##import sys
##sys.path.append('/path_to/catkin_ws/devel/lib/choreonoid-2.0/python/cnoid')

#exec(open('coords_test.py').read())

## under lib/choreonoid-2.0/cnoid
from cnoid.IRSLCoords import coordinates
import cnoid.IRSLCoords as ic
import numpy as np
import math

## for Test
import unittest

class TestCnoidPosition(unittest.TestCase):
    def test_initialize(self):
        pos = np.array([1.0, 2.0, 3.0])
        rot = ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))
        cnoid_p = ic.cnoidPosition(rot, pos)

        ##
        self.assertTrue(ic.eps_eq(cnoid_p[:3, 3], pos))
        self.assertTrue(ic.eps_eq(cnoid_p[:3, :3], rot))

        ##
        self.assertEqual(cnoid_p[3, 3], 1.0)
        self.assertEqual(cnoid_p[3, 0], 0.0)
        self.assertEqual(cnoid_p[3, 1], 0.0)
        self.assertEqual(cnoid_p[3, 2], 0.0)

        ##
        self.assertTrue(ic.eps_eq(ic.Position_translation(cnoid_p), pos))
        self.assertTrue(ic.eps_eq(ic.Position_rotation(cnoid_p), rot))

class TestCoordinates(unittest.TestCase):
    def test_initialize(self):
        rot = ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))
        quat = ic.rotationToQuaternion(rot)
        pos = np.array([1.0, 2.0, 3.0])
        zr3 = np.zeros(3)
        id3 = np.identity(3)

        p_cds = coordinates(pos)
        self.assertTrue(ic.eps_eq(p_cds.pos, pos))
        self.assertTrue(ic.eps_eq(p_cds.rot, id3))

        q_cds = coordinates(quat)
        r_cds = coordinates(rot)
        self.assertTrue(ic.eps_eq(q_cds.pos, zr3))
        self.assertTrue(ic.eps_eq(r_cds.pos, zr3))
        self.assertTrue(ic.eps_eq(q_cds.rot, r_cds.rot))

        pq_cds = coordinates(pos, quat)
        pr_cds = coordinates(pos, rot)
        cnoid_p = ic.cnoidPosition(rot, pos)
        cnoid_cds = coordinates(cnoid_p)
        self.assertTrue(pq_cds.equal(cnoid_cds))
        self.assertTrue(cnoid_cds.equal(pr_cds))
        self.assertTrue(pr_cds.equal(pq_cds))

        self.assertTrue(ic.eps_eq(pr_cds.quaternion, quat))

    def test_equal_pointer(self):
        tmp_cds = coordinates()
        pos0 = np.array([1.0, 2.0, 3.0])
        # rot0 = ic.angleAxisNormalized(0.2, np.array([1.0, 0.0, 0.0]))
        tmp_cds.pos = pos0
        self.assertTrue(ic.eps_eq(tmp_cds.pos[0], pos0[0])) ## true
        pos0[0] = 10.0
        self.assertFalse(ic.eps_eq(tmp_cds.pos[0], pos0[0])) ## false

    def test_vector(self):
        rot_a = ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))
        pos_a = np.array([1.0, 2.0, 3.0])
        cds_a = coordinates(pos_a, rot_a)

        vec = np.array([0.3, 0.2, 0.1])
        res = cds_a.rotate_vector(vec)
        self.assertFalse( ic.eps_eq(res, vec) )## not modify vec
        self.assertTrue ( ic.eps_eq(rot_a.dot(vec), res) )

        cds_a.rotateVector(vec)
        self.assertTrue( ic.eps_eq(res, vec) )## vec modified

        vec = np.array([0.3, 0.2, 0.1])
        res = cds_a.inverse_rotate_vector(vec)
        self.assertFalse( ic.eps_eq(res, vec) )## not modify vec
        self.assertTrue ( ic.eps_eq(rot_a.transpose().dot(vec), res) )

        cds_a.inverseRotateVector(vec)
        self.assertTrue( ic.eps_eq(res, vec) )## vec modified

        vec = np.array([0.3, 0.2, 0.1])
        res = cds_a.transform_vector(vec)
        self.assertFalse( ic.eps_eq(res, vec) )## not modify vec
        self.assertTrue ( ic.eps_eq(rot_a.dot(vec) + pos_a, res) )

        cds_a.transformVector(vec)
        self.assertTrue( ic.eps_eq(res, vec) )## vec modified

        vec = np.array([0.3, 0.2, 0.1])
        res = cds_a.inverse_transform_vector(vec)
        self.assertFalse( ic.eps_eq(res, vec) )## not modify vec
        self.assertTrue ( ic.eps_eq(rot_a.transpose().dot(vec) -  rot_a.transpose().dot(pos_a), res) )

        cds_a.inverseTransformVector(vec)
        self.assertTrue( ic.eps_eq(res, vec) )## vec modified

    def test_rotate(self):
        rot_a = ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))
        pos_a = np.array([1.0, 2.0, 3.0])
        cds_a = coordinates(pos_a, rot_a)

        axis_b = np.array([2.0, 3.0, 1.0])
        axis_b *= 1.0/np.linalg.norm(axis_b)

        rot_b = ic.angleAxisNormalized(0.4, axis_b)
        # pos_b = np.array([1.4, 0.7, 0.3])
        # cds_b = coordinates(rot_a, pos_a)

        cds_al = coordinates(pos_a, rot_a)
        cds_al.rotate_with_matrix(rot_b)
        self.assertTrue(ic.eps_eq(cds_al.rot, rot_a.dot(rot_b)))

        cds_aw = coordinates(pos_a, rot_a)
        cds_aw.rotate_with_matrix(rot_b, coordinates.wrt.world)
        self.assertTrue(ic.eps_eq(cds_aw.rot, rot_b.dot(rot_a)))
        self.assertFalse(cds_al.equal(cds_aw))

        cds_aal = coordinates(pos_a, rot_a)
        cds_aal.rotate(0.4, axis_b)

        self.assertTrue(cds_aal.equal(cds_al))

        cds_aaw = coordinates(pos_a, rot_a)
        cds_aaw.rotate(0.4, axis_b, coordinates.wrt.world)
        self.assertFalse(cds_aal.equal(cds_aaw))
        self.assertTrue(cds_aaw.equal(cds_aw))

        rot_w = ic.angleAxisNormalized(0.3, np.array([1.0, 0.5, 1.0]))
        pos_w = np.array([1.4, 0.7, 0.3])
        cds_w = coordinates(pos_w, rot_w)

        cds_a_wrt = coordinates(pos_a, rot_a)
        cds_a_wrt.rotate_with_matrix(rot_b, cds_w)

        cds_aa_wrt = coordinates(pos_a, rot_a)
        cds_aa_wrt.rotate(0.4, axis_b, cds_w)

        cds_a_wrt_m = coordinates(pos_a, rot_a)
        cds_a_wrt_m.rotate_with_matrix(rot_b, rot_w)

        cds_aa_wrt_m = coordinates(pos_a, rot_a)
        cds_aa_wrt_m.rotate(0.4, axis_b, rot_w)

        self.assertTrue(cds_a_wrt.equal(cds_aa_wrt))
        self.assertTrue(cds_a_wrt.equal(cds_a_wrt_m))
        self.assertTrue(cds_aa_wrt.equal(cds_aa_wrt_m))
        self.assertTrue(ic.eps_eq(cds_a_wrt.rot, rot_w.dot(rot_b).dot(rot_w.transpose()).dot(rot_a)))

    def test_orient(self):
        rot_a = ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))
        pos_a = np.array([1.0, 2.0, 3.0])
        cds_a = coordinates(pos_a, rot_a)

        axis_b = np.array([2.0, 3.0, 1.0])
        axis_b *= 1.0/np.linalg.norm(axis_b)

        rot_b = ic.angleAxisNormalized(0.4, axis_b)
        # pos_b = np.array([1.4, 0.7, 0.3])
        # cds_b = coordinates(rot_a, pos_a)

        cds_al = coordinates(pos_a, rot_a)
        cds_al.orient_with_matrix(rot_b)
        self.assertTrue(ic.eps_eq(cds_al.rot, rot_a.dot(rot_b)))

        cds_aw = coordinates(pos_a, rot_a)
        cds_aw.orient_with_matrix(rot_b, coordinates.wrt.world)
        self.assertTrue(ic.eps_eq(cds_aw.rot, rot_b))
        self.assertFalse(cds_al.equal(cds_aw))

        cds_aal = coordinates(pos_a, rot_a)
        cds_aal.orient(0.4, axis_b)

        self.assertTrue(cds_aal.equal(cds_al))

        cds_aaw = coordinates(pos_a, rot_a)
        cds_aaw.orient(0.4, axis_b, coordinates.wrt.world)
        self.assertFalse(cds_aal.equal(cds_aaw))
        self.assertTrue(cds_aaw.equal(cds_aw))

        rot_w = ic.angleAxisNormalized(0.3, np.array([1.0, 0.5, 1.0]))
        pos_w = np.array([1.4, 0.7, 0.3])
        cds_w = coordinates(pos_w, rot_w)

        cds_a_wrt = coordinates(pos_a, rot_a)
        cds_a_wrt.orient_with_matrix(rot_b, cds_w)

        cds_aa_wrt = coordinates(pos_a, rot_a)
        cds_aa_wrt.orient(0.4, axis_b, cds_w)

        cds_a_wrt_m = coordinates(pos_a, rot_a)
        cds_a_wrt_m.orient_with_matrix(rot_b, rot_w)

        cds_aa_wrt_m = coordinates(pos_a, rot_a)
        cds_aa_wrt_m.orient(0.4, axis_b, rot_w)

        self.assertTrue(cds_a_wrt.equal(cds_aa_wrt))
        self.assertTrue(cds_a_wrt.equal(cds_a_wrt_m))
        self.assertTrue(cds_aa_wrt.equal(cds_aa_wrt_m))
        self.assertTrue(ic.eps_eq(cds_a_wrt.rot, rot_w.dot(rot_b)))

    def test_translate(self):
        rot_a = ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))
        pos_a = np.array([1.0, 2.0, 3.0])
        pos_b = np.array([0.5, 0.7, 1.2])

        cds_a = coordinates(pos_a, rot_a)
        cds_a.translate(pos_b)
        self.assertTrue(ic.eps_eq(cds_a.rot, ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))))
        self.assertTrue(ic.eps_eq(cds_a.pos, pos_a + rot_a.dot(pos_b)))

        cds_a = coordinates(pos_a, rot_a)
        cds_a.translate(pos_b, coordinates.wrt.world)
        self.assertTrue(ic.eps_eq(cds_a.rot, ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))))
        self.assertTrue(ic.eps_eq(cds_a.pos, pos_a + pos_b))

        rot_w = ic.angleAxisNormalized(0.3, np.array([1.0, 0.5, 1.0]))
        pos_w = np.array([1.4, 0.7, 0.3])
        cds_w = coordinates(pos_w, rot_w)
        cds_a = coordinates(pos_a, rot_a)
        cds_a.translate(pos_b, cds_w)
        self.assertTrue(ic.eps_eq(cds_a.rot, ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))))
        self.assertTrue(ic.eps_eq(cds_a.pos, pos_a + rot_w.dot(pos_b)))

    def test_locate(self):
        rot_a = ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))
        pos_a = np.array([1.0, 2.0, 3.0])
        pos_b = np.array([0.5, 0.7, 1.2])

        cds_a = coordinates(pos_a, rot_a)
        cds_a.locate(pos_b)
        self.assertTrue(ic.eps_eq(cds_a.rot, ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))))
        self.assertTrue(ic.eps_eq(cds_a.pos, pos_a + rot_a.dot(pos_b)))

        cds_a = coordinates(pos_a, rot_a)
        cds_a.locate(pos_b, coordinates.wrt.world)
        self.assertTrue(ic.eps_eq(cds_a.rot, ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))))
        self.assertTrue(ic.eps_eq(cds_a.pos, pos_b))

        rot_w = ic.angleAxisNormalized(0.3, np.array([1.0, 0.5, 1.0]))
        pos_w = np.array([1.4, 0.7, 0.3])
        cds_w = coordinates(pos_w, rot_w)
        cds_a = coordinates(pos_a, rot_a)
        cds_a.locate(pos_b, cds_w)
        self.assertTrue(ic.eps_eq(cds_a.rot, ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))))
        self.assertTrue(ic.eps_eq(cds_a.pos, pos_w + rot_w.dot(pos_b)))

    def test_move_to(self):
        rot_a = ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))
        pos_a = np.array([1.0, 2.0, 3.0])
        cds_a_org = coordinates(pos_a, rot_a)

        rot_b = ic.angleAxisNormalized(0.4, np.array([2.0, 3.0, 1.0]))
        pos_b = np.array([1.4, 0.7, 0.3])
        cds_b = coordinates(pos_b, rot_b)

        cds_a = coordinates(pos_a, rot_a)
        cds_a.move_to(cds_b)
        self.assertTrue(ic.eps_eq(cds_a.toPosition(), cds_a_org.toPosition().dot(cds_b.toPosition())))

        cds_a = coordinates(pos_a, rot_a)
        cds_a.move_to(cds_b, coordinates.wrt.world)
        self.assertTrue(ic.eps_eq(cds_a.toPosition(), cds_b.toPosition()))

        rot_w = ic.angleAxisNormalized(0.3, np.array([1.0, 0.5, 1.0]))
        pos_w = np.array([1.4, 0.7, 0.3])
        cds_w = coordinates(pos_w, rot_w)

        cds_a = coordinates(pos_a, rot_a)
        cds_a.move_to(cds_b, cds_w)

        self.assertTrue(ic.eps_eq(cds_a.toPosition(), cds_w.toPosition().dot(cds_b.toPosition())))

    def test_inverse(self):
        rot_a = ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))
        pos_a = np.array([1.0, 2.0, 3.0])
        cds_a = coordinates(pos_a, rot_a)
        cds_org = cds_a.copy()
        inv_a = cds_a.inverse_transformation()

        self.assertTrue( (cds_a is not cds_org) )
        self.assertTrue(cds_a.equal(cds_org))

        self.assertTrue(ic.eps_eq(cds_a.toPosition().dot(inv_a.toPosition()), np.identity(4)))

        cds_a.transform(inv_a)

        self.assertTrue(ic.eps_eq(cds_a.pos, np.zeros(3)))
        self.assertTrue(ic.eps_eq(cds_a.rot, np.identity(3)))

        self.assertFalse(cds_a.equal(cds_org))

        cds_org.inverse() ## modify cds_org
        self.assertTrue(inv_a.equal(cds_org))

    def test_transformation(self):
        rot_a = ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))
        pos_a = np.array([1.0, 2.0, 3.0])
        cds_a = coordinates(pos_a, rot_a)

        rot_b = ic.angleAxisNormalized(0.4, np.array([2.0, 3.0, 1.0]))
        pos_b = np.array([1.4, 0.7, 0.3])
        cds_b = coordinates(pos_b, rot_b)

        trs_l = cds_a.transformation(cds_b)
        self.assertTrue(ic.eps_eq(trs_l.toPosition(), cds_a.inverse_transformation().toPosition().dot(cds_b.toPosition())))

        trs_w = cds_a.transformation(cds_b, coordinates.wrt.world)
        self.assertTrue(ic.eps_eq(trs_w.toPosition(), cds_b.toPosition().dot(cds_a.inverse_transformation().toPosition())))

        rot_w = ic.angleAxisNormalized(0.3, np.array([1.0, 0.5, 1.0]))
        pos_w = np.array([1.4, 0.7, 0.3])
        cds_w = coordinates(pos_w, rot_w)

        trs_wrt = cds_a.transformation(cds_b, cds_w)

        self.assertTrue(ic.eps_eq(trs_wrt.toPosition(), cds_w.inverse_transformation().toPosition().dot(cds_b.toPosition()).dot(cds_a.inverse_transformation().toPosition()).dot(cds_w.toPosition())))

    def test_transform(self):
        rot_a = ic.angleAxisNormalized(0.6, np.array([1.0, 2.0, 3.0]))
        pos_a = np.array([1.0, 2.0, 3.0])
        cds_a_org = coordinates(pos_a, rot_a)
        cds_a = coordinates(pos_a, rot_a)

        rot_b = ic.angleAxisNormalized(0.4, np.array([2.0, 3.0, 1.0]))
        pos_b = np.array([1.4, 0.7, 0.3])
        cds_b = coordinates(pos_b, rot_b)

        cds_a.transform(cds_b)
        self.assertTrue(ic.eps_eq(cds_a.toPosition(), cds_a_org.toPosition().dot(cds_b.toPosition())))

        cds_a = coordinates(pos_a, rot_a)
        cds_a.transform(cds_b, coordinates.wrt.world)

        self.assertTrue(ic.eps_eq(cds_a.toPosition(), cds_b.toPosition().dot(cds_a_org.toPosition())))

        rot_w = ic.angleAxisNormalized(0.3, np.array([1.0, 0.5, 1.0]))
        pos_w = np.array([1.4, 0.7, 0.3])
        cds_w = coordinates(pos_w, rot_w)

        cds_a = coordinates(pos_a, rot_a)
        cds_a.transform(cds_b, cds_w)

        self.assertTrue(ic.eps_eq(cds_a.toPosition(), cds_w.toPosition().dot(cds_b.toPosition()).dot(cds_w.inverse_transformation().toPosition()).dot(cds_a_org.toPosition())))

if __name__ == '__main__':
    unittest.main()
