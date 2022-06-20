#include <cnoid/EigenTypes>

#pragma once

#define __irsl_use_inline__

namespace cnoid {
    enum coords_wrt {
        default_wrt,
        local,
        world
    };

    inline bool eps_eq(const double a, const double b, const double eps = 0.00001) {
        return fabs((a)-(b)) <= eps;
    };

    //void calcRodrigues(Matrix3& out_R, const Vector3& axis, double q); => angle_axis
#ifndef __irsl_use_inline__
    Vector3 matrix_log(const Matrix3& m);
    void rotm3times (Matrix3& m12, const Matrix3& m1, const Matrix3& m2);
    void difference_rotation(Vector3& ret_dif_rot, const Matrix3& self_rot, const Matrix3& target_rot);
    void mid_rot(Matrix3& mid_rot, const double p, const Matrix3& rot1, const Matrix3& rot2, const double eps = 0.00001);

    void mid_coords(Position &mid_coords, const double p, const Position &c1, const Position &c2, const double eps = 0.00001);
#else
    inline Vector3 matrix_log(const Matrix3& m) {
        Vector3 mlog;
        double q0, th;
        Vector3 q;
        double norm;

        Quaternion eiq(m);
        q0 = eiq.w();
        q = eiq.vec();
        norm = q.norm();
        if (norm > 0) {
            if ((q0 > 1.0e-10) || (q0 < -1.0e-10)) {
                th = 2 * std::atan(norm / q0);
            } else if (q0 > 0) {
                th = M_PI / 2;
            } else {
                th = -M_PI / 2;
            }
            mlog = (th / norm) * q ;
        } else {
            mlog = Vector3::Zero();
        }
        return mlog;
    }

    // matrix product using quaternion normalization
    inline void rotm3times (Matrix3& m12, const Matrix3& m1, const Matrix3& m2)
    {
        Quaternion eiq1(m1);
        Quaternion eiq2(m2);
        Quaternion eiq3;
        eiq3 = eiq1 * eiq2;
        eiq3.normalize();
        m12 = eiq3.toRotationMatrix();
    }

    inline void difference_rotation(Vector3& ret_dif_rot, const Matrix3& self_rot, const Matrix3& target_rot)
    {
        //ret_dif_rot = self_rot * hrp::omegaFromRot(self_rot.transpose() * target_rot);
        ret_dif_rot = self_rot * matrix_log ( self_rot.transpose() * target_rot );
    }

    inline void mid_rot(Matrix3& mid_rot, const double p, const Matrix3& rot1, const Matrix3& rot2, const double eps)
    {
        Matrix3 r(rot1.transpose() * rot2);
        Vector3 omega(matrix_log(r));
        if (eps_eq(omega.norm(),0.0, eps)) { // c1.rot and c2.rot are same
            mid_rot = rot1;
        } else {
            //calcRodrigues(r, omega.normalized(), omega.norm()*p);
            r = AngleAxis(omega.norm()*p, omega.normalized());
            //mid_rot = c1.rot * r;
            rotm3times(mid_rot, rot1, r);
        }
    }

    inline void mid_coords(Position &mid_coords, const double p, const Position &c1, const Position &c2, const double eps) { // eps = 0.00001
        Matrix3 c1_rot(c1.linear());
        Matrix3 c2_rot(c2.linear());
        Matrix3 ret_rot;
        mid_rot(ret_rot, p, c1_rot, c2_rot, eps);
        mid_coords.translation() = (1 - p) * c1.translation() + p * c2.translation();
        mid_coords.linear() = ret_rot;
    }

#endif
    // from rats coordinates
    // coordinates => Position(Isometry3d)
#ifndef __irsl_use_inline__
    //void rotate_with_matrix (const hrp::Matrix33& mat, const std::string& wrt = ":local");
    void rotate_with_matrix(Position &coords, const Matrix3 &mat, const coords_wrt wrt = local ); //wrt
    //void rotate (const double theta, const hrp::Vector3& axis, const std::string& wrt = ":local");
    void rotate(Position &coords, const double theta, const Vector3& axis, const coords_wrt wrt = local ); // wrt
    //void difference(hrp::Vector3& dif_pos, hrp::Vector3& dif_rot, const coordinates& c);
    void difference(const Position &coords, Vector3& dif_pos, Vector3& dif_rot, const Position& c);
    //void inverse_transformation(coordinates& inv);
    void inverse_transformation(const Position &coords, Position& inv);
    //void transformation(coordinates& tc, coordinates c, const std::string& wrt = ":local") const;
    void transformation(const Position &coords, Position& trans_coords, const Position &c, const coords_wrt wrt = local ); // wrt
    //void transform(const coordinates& c, const std::string& wrt = ":local");
    void transform(Position &coords, const Position& c, const coords_wrt wrt = local ); // wrt
#else
    inline void rotate_with_matrix(Position &coords, const Matrix3 &mat, const coords_wrt wrt = local )
    {
        Matrix3 rot;
        Matrix3 rot_org = coords.linear();
        if (wrt == local) {
            rotm3times(rot, rot_org, mat);
        } else if (wrt == world) {
            rotm3times(rot, mat, rot_org);
        } else {
            return;
        }
        coords.linear() = rot;
    }

    inline void rotate(Position &coords, const double theta, const Vector3& axis, const coords_wrt wrt = local )
    {
        AngleAxis tmpr(theta, axis);
        rotate_with_matrix(coords, tmpr.toRotationMatrix(), wrt);
    }

    inline void difference(const Position &coords, Vector3& dif_pos, Vector3& dif_rot, const Position& c)
    {
        dif_pos = c.translation() - coords.translation();
        difference_rotation(dif_rot, coords.linear(), c.linear());
    }

    inline void inverse_transformation(const Position &coords, Position& inv)
    {
        inv = coords.inverse();
    }

    inline void transform(Position &coords, const Position& c, const coords_wrt wrt = local )
    {
        if (wrt == local) {
            Position tmp = (coords * c);
            Matrix3 rot;
            rotm3times(rot, coords.linear(), c.linear());
            coords.translation() = tmp.translation();
            coords.linear() = rot;
        } else if (wrt == world) {
            Position tmp = (c * coords);
            Matrix3 rot;
            rotm3times(rot, c.linear(), coords.linear());
            coords.translation() = tmp.translation();
            coords.linear() = rot;
        }
    }

    inline void transformation(const Position &coords, Position& trans_coords, const Position &c, const coords_wrt wrt = local )
    {
        if(wrt == local) {
            // coords^-1 * c
            inverse_transformation(coords, trans_coords);
            transform(trans_coords, c);
        } else if (wrt == world) {
            // c * coords^-1
            Position inv_trans;
            inverse_transformation(coords, inv_trans);
            trans_coords = c;
            transform(trans_coords, inv_trans);
        }
    }

#endif
    //from EusLisp definition
    //rotate_vector
    //inverse_rotate_vector
    //transform_vector
    //inverse_transform_vector
    //
    //--transformation :local :world :wrt
    //inverse_transformation :local :world :wrt
    //
    //move-to :local :world :wrt
    //translate :local :world :wrt
    //locate :local :world :wrt
    //--transform  :local :world :wrt  WAW^-1 T
}
