#include <cnoid/EigenTypes>
#include <memory>
#include <iostream>

#pragma once

using Matrix4RM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Matrix3RM = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

// numpy (RowMajor) / Eigen (ColumnMajor)
// Ref<T> for a writable reference,
// const Ref<const T>& for a const reference
// Ref<const T>, might still be interesting if you want to change the matrix that is referenced (not its content) through a call to Ref constructor using placement new.
typedef Eigen::Ref<const Matrix4RM> ref_mat4;
typedef Eigen::Ref<const Matrix3RM> ref_mat3;
typedef Eigen::Ref<const cnoid::Vector4>   ref_vec4;
typedef Eigen::Ref<const cnoid::Vector3>   ref_vec3;

#define __irsl_use_inline__

namespace cnoid {
    inline bool eps_eq(const double a, const double b, const double eps = 0.00001) {
        return fabs((a)-(b)) <= eps;
    }
    inline bool eps_eq(const Vector3 &a, const Vector3 &b, const double eps = 0.00001) {
        if (!eps_eq(a(0), b(0), eps)) return false;
        if (!eps_eq(a(1), b(1), eps)) return false;
        if (!eps_eq(a(2), b(2), eps)) return false;
        return true;
    }
    inline bool eps_eq(const Vector4 &a, const Vector4 &b, const double eps = 0.00001) {
        if (!eps_eq(a(0), b(0), eps)) return false;
        if (!eps_eq(a(1), b(1), eps)) return false;
        if (!eps_eq(a(2), b(2), eps)) return false;
        if (!eps_eq(a(3), b(3), eps)) return false;
        return true;
    }
    inline bool eps_eq(const Matrix3 &a, const Matrix3 &b, const double eps = 0.00001) {
        const double *aptr = a.data();
        const double *bptr = b.data();
        for(size_t i = 0; i < a.size(); i++) {
            if (!eps_eq(*aptr++, *bptr++, eps)) return false;
        }
        return true;
    }
    inline bool eps_eq(const Matrix3RM &a, const Matrix3RM &b, const double eps = 0.00001) {
        const double *aptr = a.data();
        const double *bptr = b.data();
        for(size_t i = 0; i < a.size(); i++) {
            if (!eps_eq(*aptr++, *bptr++, eps)) return false;
        }
        return true;
    }
    inline bool eps_eq(const Matrix4 &a, const Matrix4 &b, const double eps = 0.00001) {
        const double *aptr = a.data();
        const double *bptr = b.data();
        for(size_t i = 0; i < a.size(); i++) {
            if (!eps_eq(*aptr++, *bptr++, eps)) return false;
        }
        return true;
    }
    inline bool eps_eq(const Matrix4RM &a, const Matrix4RM &b, const double eps = 0.00001) {
        const double *aptr = a.data();
        const double *bptr = b.data();
        for(size_t i = 0; i < a.size(); i++) {
            if (!eps_eq(*aptr++, *bptr++, eps)) return false;
        }
        return true;
    }
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
    // m12 <= m1 * m2
    inline void rotm3times (Matrix3& m12, const Matrix3& m1, const Matrix3& m2)
    {
        Quaternion eiq1(m1);
        Quaternion eiq2(m2);
        Quaternion eiq3;
        eiq3 = eiq1 * eiq2;
        eiq3.normalize();
        m12 = eiq3.toRotationMatrix();
    }
    // m1 <= m1 * m2
    inline void rot_with_normalize (Matrix3& m1, const Matrix3& m2)
    {
        Quaternion eiq1(m1);
        Quaternion eiq2(m2);
        Quaternion eiq3;
        eiq3 = eiq1 * eiq2;
        eiq3.normalize();
        m1 = eiq3.toRotationMatrix();
    }

    inline void difference_rotation(Vector3& ret_dif_rot, const Matrix3& self_rot, const Matrix3& target_rot)
    {
        //ret_dif_rot = self_rot * hrp::omegaFromRot(self_rot.transpose() * target_rot);
        ret_dif_rot = self_rot * matrix_log ( self_rot.transpose() * target_rot );
    }
    inline void difference_rotation__(Vector3& ret_dif_rot, const Matrix3& self_rot, const Matrix3& target_rot)
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
    // copy from hrpsys-base/rtc/ImpedanceController/RatsMatrix
    struct coordinates {
        enum wrt {
            default_wrt,
            local,
            world,
            parent
        };
        enum axis {
            default_axis,
            x,
            y,
            z,
            inv_x,
            inv_y,
            inv_z
        };

        Vector3 pos;
        Matrix3 rot;

        coordinates() : pos(Vector3::Zero()), rot(Matrix3::Identity()) {};
        coordinates(const Vector3& p, const Matrix3& r) : pos(p), rot(r) {};
        coordinates(const Vector3& p, const Quaternion& q) : pos(p), rot(q) {};
        coordinates(const Vector3& p, const AngleAxis& ax) : pos(p), rot(ax) {};
        coordinates(const Vector3& p) : pos(p), rot(Matrix3::Identity()) {};
        coordinates(const Matrix3& r) : pos(Vector3::Zero()), rot(r) {};
        coordinates(const Quaternion& q) : pos(Vector3::Zero()), rot(q) {};
        coordinates(const AngleAxis& ax) : pos(Vector3::Zero()), rot(ax) {};
        coordinates(const coordinates& c) : pos(c.pos), rot(c.rot) {};
        coordinates(const Position &p) : pos(p.translation()), rot(p.linear()) {};
        virtual ~coordinates() { }

        void set(const Vector3 &p) { pos = p; }
        void set(const Matrix3 &r) { rot = r; }
        void set(const Quaternion &q) { rot = q; }
        void set(const AngleAxis &ax) { rot = ax; }

        bool equal(const coordinates &c, const double eps = 0.00001)
        {
            if (!eps_eq(pos, c.pos, eps)) return false;
            if (!eps_eq(rot, c.rot, eps)) return false;
            return true;
        }

        void toPosition(Position &p) const
        {
            p.setIdentity();
            p.translation() = pos;
            p.linear() = rot;
        }
        coordinates& operator=(const coordinates& c)
        {
            if (this != &c) {
                pos = c.pos;
                rot = c.rot;
            }
            return *this;
        }
        coordinates& operator=(const Position& p)
        {
            pos = p.translation();
            rot = p.linear();
            return *this;
        }
        void rotate_with_matrix (const Matrix3& mat, const wrt wrt = local )
        {
#if 0
            Matrix3 rot_org(rot);
            if (wrt == local) {
                // alias(rot) = rot * mat;
                rotm3times(rot, rot_org, mat);
            } else if(wrt == world) {
                // alias(rot) = mat * rot;
                rotm3times(rot, mat, rot_org);
            } else {
                //std::cerr << "**** invalid wrt! ****" << std::endl;
            }
#endif
            if (wrt == local) {
                // alias(rot) = rot * mat;
                rotm3times(rot, rot, mat);
            } else if(wrt == world || wrt == parent) {
                // alias(rot) = mat * rot;
                rotm3times(rot, mat, rot);
            }
        }
        void rotate_with_matrix (const Matrix3& mat, const coordinates &wrt )
        {
            Matrix3 r2t = wrt.rot.transpose();
            //rot =  W * mat * W^1 * rot
            rotm3times(r2t, mat, r2t);
            rotm3times(r2t, wrt.rot, r2t);
            rotm3times(rot, r2t, rot);
        }
        void rotate (const double theta, const Vector3& axis, const wrt wrt = local )
        {
            Eigen::AngleAxis<double> tmpr(theta, axis);
            rotate_with_matrix(tmpr.toRotationMatrix(), wrt);
        }
        void rotate (const double theta, const Vector3& axis, const coordinates &wrt )
        {
            Eigen::AngleAxis<double> tmpr(theta, axis);
            rotate_with_matrix(tmpr.toRotationMatrix(), wrt);
        }
        void orient_with_matrix (const Matrix3& mat, const wrt wrt = local )
        {
            if (wrt == local) {
                rotm3times(rot, rot, mat);
            } else if (wrt == world || wrt == parent) {
                rot = mat;
            }
        }
        void orient_with_matrix (const Matrix3& mat, const coordinates &wrt )
        {
            Matrix3 r2 = wrt.rot;
            //rot =  W * mat
            rotm3times(rot, wrt.rot, mat);
        }
        //void rotate (const double theta, const Vector3& axis, const coordinates &wrt)
        /* void difference_rotation(Vector3& dif_rot, const coordinates& c) const { */
        /*   dif_rot = rot * matrix_log(rot.transpose() * c.rot); */
        /* } */
        void difference(Vector3& dif_pos, Vector3& dif_rot, const coordinates& c) const
        {
            dif_pos = c.pos - pos;
            difference_rotation__(dif_rot, rot, c.rot);
        }
        void difference_position(Vector3& dif_pos, const coordinates& c) const
        {
            dif_pos = c.pos - pos;
        }
        void difference_rotation(Vector3& dif_rot, const coordinates& c) const
        {
            difference_rotation__(dif_rot, rot, c.rot);
        }
        //abc
        void inverse_transformation(coordinates& inv) const
        {
            inv.rot = rot.transpose();
            inv.pos = inv.rot * (-1 * pos);
        }
        //void transformation(coordinates& tc, coordinates c, const wrt wrt = local ) const
        void transformation(coordinates& tc, const coordinates &c, const wrt wrt = local ) const
        {
#if 0
            tc = *this;//??
            inverse_transformation(tc);
            if (wrt == local) {
                tc.transform(c);
            } else if(wrt == world || wrt == parent) {
                coordinates tmp(c);
                tmp.transform(tc);
                tc = tmp;
            } else {
                //std::cerr << "**** invalid wrt! ****" << std::endl;
            }
#endif
            coordinates tmp(c); // safe if &tc == &c
            inverse_transformation(tc);
            tc.transform(tmp, wrt);
        }
        void transformation(coordinates& tc, const coordinates &c, const coordinates &wrt) const
        {
            coordinates tmp(c); // safe if &tc == &c
            coordinates inv_self;
            inverse_transformation(inv_self);

            wrt.inverse_transformation(tc);
            tc.transform(tmp);
            tc.transform(inv_self);
            tc.transform(wrt);
        }
        void transform(const coordinates& c, const wrt wrt = local )
        {
            if (wrt == local) {
                pos += rot * c.pos;
#if 0
                Matrix3 rot_org(rot);
                rotm3times(rot, rot_org, c.rot);
#endif
                rotm3times(rot, rot, c.rot);
            } else if (wrt == world || wrt == parent) {
#if 0
                Vector3 p(c.pos);
                Matrix3 r(c.rot);
                p += r * pos;
                rotm3times(r, c.rot, rot);
                pos = p;
                rot = r;
#endif
                pos = c.pos + (c.rot * pos);
                rotm3times(rot, c.rot, rot);
            } else {
                //std::cerr << "**** invalid wrt! ****" << std::endl;
            }
        }
        void transform(const coordinates& c, const coordinates &wrt )
        {
            coordinates tmp(wrt);
            coordinates inv_wrt;
            //self := w * c * w^-1 * self
            wrt.inverse_transformation(inv_wrt);
            tmp.transform(c);
            tmp.transform(inv_wrt);
            tmp.transform(*this);

            *this = tmp;
        }

        ////
        void rotate_vector(Vector3 &vec) const
        {
            vec = rot * vec;
        }
        void inverse_rotate_vector(Vector3 &vec) const
        {
            vec = vec.transpose() * rot;
        }
        void transform_vector(Vector3 &vec) const
        {
            vec = (rot * vec);
            vec += pos;
        }
        void inverse_transform_vector(Vector3 &vec) const
        {
            vec = rot.transpose() * (vec - pos);
        }
        void move_to (const coordinates &c, const wrt wrt = local )
        {
            if (wrt == local) {
                transform(c);
            } else if (wrt == world || wrt == parent) {
                *this = c;
            }
        }
        void move_to (const coordinates &c, const coordinates &wrt)
        {
            coordinates tmp(wrt);
            tmp.transform(c);
            *this = tmp;
        }
        void translate(const Vector3 &vec, const wrt wrt = local )
        {
            if (wrt == local) {
                pos += rot * vec;
            } else if (wrt == world || wrt == parent) {
                pos += vec;
            }
        }
        void translate(const Vector3 &vec, const coordinates &wrt)
        {
            pos += wrt.rot * vec;
        }
        void locate(const Vector3 &vec, const wrt wrt = local )
        {
            if (wrt == local) {
                pos += rot * vec;
            } else if (wrt == world || wrt == parent) {
                pos = vec;
            }
        }
        void locate(const Vector3 &vec, const coordinates &wrt)
        {
            pos = wrt.pos + (wrt.rot * vec);
        }
        // void orient(const Matrix3 &mat, const wrt wrt = local )
        // euler-angle
        // rpy-angle
        // rotation-angle
        // x_axis
        // y_axis
        // z_axis
        // reset_coords
    };

typedef std::shared_ptr< coordinates > coordinatesPtr;

#ifndef __irsl_use_inline__
    //void rotate_with_matrix (const hrp::Matrix33& mat, const std::string& wrt = ":local");
    void rotate_with_matrix(Position &coords, const Matrix3 &mat, const coordinates::wrt wrt = coordinates::wrt::local ); //wrt
    //void rotate (const double theta, const hrp::Vector3& axis, const std::string& wrt = ":local");
    void rotate(Position &coords, const double theta, const Vector3& axis, const coordinates::wrt wrt = coordinates::wrt::local ); // wrt
    //void difference(hrp::Vector3& dif_pos, hrp::Vector3& dif_rot, const coordinates& c);
    void difference(const Position &coords, Vector3& dif_pos, Vector3& dif_rot, const Position& c);
    //void inverse_transformation(coordinates& inv);
    void inverse_transformation(const Position &coords, Position& inv);
    //void transformation(coordinates& tc, coordinates c, const std::string& wrt = ":local") const;
    void transformation(const Position &coords, Position& trans_coords, const Position &c, const coordinates::wrt wrt = coordinates::wrt::local ); // wrt
    //void transform(const coordinates& c, const std::string& wrt = ":local");
    void transform(Position &coords, const Position& c, const coordinates::wrt wrt = coordinates::wrt::local ); // wrt
#else
    inline void rotate_with_matrix(Position &coords, const Matrix3 &mat, const coordinates::wrt wrt = coordinates::wrt::local )
    {
        Matrix3 rot = coords.linear();
        if (wrt == coordinates::wrt::local) {
            rotm3times(rot, rot, mat);
        } else if (wrt == coordinates::wrt::world || wrt == coordinates::wrt::parent) {
            rotm3times(rot, mat, rot);
        } else {
            return;
        }
        coords.linear() = rot;
    }

    inline void rotate(Position &coords, const double theta, const Vector3& axis, const coordinates::wrt wrt = coordinates::wrt::local )
    {
        AngleAxis tmpr(theta, axis);
        rotate_with_matrix(coords, tmpr.toRotationMatrix(), wrt);
    }

    inline void difference(const Position &coords, Vector3& dif_pos, Vector3& dif_rot, const Position& c)
    {
        dif_pos = c.translation() - coords.translation();
        difference_rotation(dif_rot, coords.linear(), c.linear());
    }
    inline void difference_rotation(const Position &coords, Vector3& dif_rot, const Position& c)
    {
        difference_rotation(dif_rot, coords.linear(), c.linear());
    }
    inline void difference_position(const Position &coords, Vector3& dif_pos, const Position& c)
    {
        dif_pos = c.translation() - coords.translation();
    }
    inline void inverse_transformation(const Position &coords, Position& inv)
    {
        inv = coords.inverse();
    }

    inline void transform(Position &coords, const Position& c, const coordinates::wrt wrt = coordinates::wrt::local )
    {
        if (wrt == coordinates::wrt::local) {
            Position tmp = (coords * c);
            Matrix3 rot;
            rotm3times(rot, coords.linear(), c.linear());
            coords.translation() = tmp.translation();
            coords.linear() = rot;
        } else if (wrt == coordinates::wrt::world || wrt == coordinates::wrt::parent) {
            Position tmp = (c * coords);
            Matrix3 rot;
            rotm3times(rot, c.linear(), coords.linear());
            coords.translation() = tmp.translation();
            coords.linear() = rot;
        }
    }

    inline void transformation(const Position &coords, Position& trans_coords, const Position &c, const coordinates::wrt wrt = coordinates::wrt::local )
    {
#if 0
        if(wrt == coordinates::wrt::local) {
            // coords^-1 * c
            inverse_transformation(coords, trans_coords);
            transform(trans_coords, c);
        } else if (wrt == coordinates::wrt::world || wrt == coordinates::wrt::parent) {
            // c * coords^-1
            Position inv_trans;
            inverse_transformation(coords, inv_trans);
            trans_coords = c;
            transform(trans_coords, inv_trans);
        }
#endif
        Position tmp(c);
        inverse_transformation(coords, trans_coords);
        transform(trans_coords, tmp, wrt);
    }

#endif
    inline void mid_coords(coordinates& mid_coords, const double p, const coordinates& c1, const coordinates& c2, const double eps = 0.00001)
    {
        mid_coords.pos = (1 - p) * c1.pos + p * c2.pos;
        mid_rot(mid_coords.rot, p, c1.rot, c2.rot, eps);
    };
}
