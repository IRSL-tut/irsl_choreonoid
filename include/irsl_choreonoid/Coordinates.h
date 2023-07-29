#include <cnoid/EigenTypes>
#include <cnoid/EigenUtil> // rotFromRpy
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

//#define __irsl_use_inline__
// all functions are inline function ...
namespace cnoid {
    typedef Isometry3 cnoidPosition;

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
    inline void _rotm3times (Matrix3& m12, const Matrix3& m1, const Matrix3& m2)
    {
        Quaternion eiq1(m1);
        Quaternion eiq2(m2);
        Quaternion eiq3;
        eiq3 = eiq1 * eiq2;
        eiq3.normalize();
        m12 = eiq3.toRotationMatrix();
    }
    // m1 <= m1 * m2
    inline void _rot_with_normalize (Matrix3& m1, const Matrix3& m2)
    {
        Quaternion eiq1(m1);
        Quaternion eiq2(m2);
        Quaternion eiq3;
        eiq3 = eiq1 * eiq2;
        eiq3.normalize();
        m1 = eiq3.toRotationMatrix();
    }

    inline void _difference_rotation (Vector3& ret_dif_rot, const Matrix3& self_rot, const Matrix3& target_rot)
    {
        //ret_dif_rot = self_rot * hrp::omegaFromRot(self_rot.transpose() * target_rot);
        ret_dif_rot = self_rot * matrix_log ( self_rot.transpose() * target_rot );
    }

    inline void _mid_rot(Matrix3& mid_rot, const double p, const Matrix3& rot1, const Matrix3& rot2, const double eps)
    {
        Matrix3 r(rot1.transpose() * rot2);
        Vector3 omega(matrix_log(r));
        if (eps_eq(omega.norm(),0.0, eps)) { // c1.rot and c2.rot are same
            mid_rot = rot1;
        } else {
            //calcRodrigues(r, omega.normalized(), omega.norm()*p);
            r = AngleAxis(omega.norm()*p, omega.normalized());
            //mid_rot = c1.rot * r;
            _rotm3times(mid_rot, rot1, r);
        }
    }

    inline void mid_coords_pos(cnoidPosition &mid_coords, const double p, const cnoidPosition &c1, const cnoidPosition &c2, const double eps) { // eps = 0.00001
        Matrix3 c1_rot(c1.linear());
        Matrix3 c2_rot(c2.linear());
        Matrix3 ret_rot;
        _mid_rot(ret_rot, p, c1_rot, c2_rot, eps);
        mid_coords.translation() = (1 - p) * c1.translation() + p * c2.translation();
        mid_coords.linear() = ret_rot;
    }

    // from rats coordinates
    // coordinates => Position(Isometry3d)
    // copy from hrpsys-base/rtc/ImpedanceController/RatsMatrix
    // original idea from https://github.com/euslisp/EusLisp/blob/master/lisp/l/coordinates.l
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

        coordinates() : pos(Vector3::Zero()), rot(Matrix3::Identity()) {}
        coordinates(const Vector3& p, const Matrix3& r) : pos(p), rot(r) {}
        coordinates(const Vector3& p, const Quaternion& q) : pos(p), rot(q) {}
        coordinates(const Vector3& p, const AngleAxis& ax) : pos(p), rot(ax) {}
        coordinates(const Vector3& p) : pos(p), rot(Matrix3::Identity()) {}
        coordinates(const Matrix3& r) : pos(Vector3::Zero()), rot(r) {}
        coordinates(const Quaternion& q) : pos(Vector3::Zero()), rot(q) {}
        coordinates(const AngleAxis& ax) : pos(Vector3::Zero()), rot(ax) {}
        coordinates(const coordinates& c) : pos(c.pos), rot(c.rot) {}
        coordinates(const cnoidPosition& p) : pos(p.translation()), rot(p.linear()) {}
        coordinates(const Vector3& pp, double r, double p, double y) : pos(pp) { setRPY(r, p, y); }
        coordinates(const Vector3& pp, const Vector3& rpy) : pos(pp) { setRPY(rpy); }
        coordinates(double r, double p, double y) : pos(Vector3::Zero()) { setRPY(r, p, y); }
        virtual ~coordinates() {
#if 0
          std::cerr << "coordinates del: ";
          std::cerr <<  std::hex << reinterpret_cast<void *>(this) << std::endl;
#endif
        }

        bool isInitial() const {
            if(pos.norm() != 0.0) return false;
            AngleAxis aa_(rot);
            if(aa_.angle() != 0.0) return false;
            return true;
        }
        bool isInitial(double eps) {
            if(pos.norm() > eps) return false;
            AngleAxis aa_(rot);
            if(aa_.angle() > eps) return false;
            return true;
        }
        void set(const Vector3 &p) { pos = p; }
        void set(const Matrix3 &r) { rot = r; }
        void set(const Quaternion &q) { rot = q; }
        void set(const AngleAxis &ax) { rot = ax; }
        void newcoords(const coordinates &c) { *this = c; }
        void newcoords(const cnoidPosition &p) { *this = p; }
        bool equal(const coordinates &c, const double eps = 0.00001) const
        {
            if (!eps_eq(pos, c.pos, eps)) return false;
            if (!eps_eq(rot, c.rot, eps)) return false;
            return true;
        }

        void toPosition(cnoidPosition &p) const
        {
            p.setIdentity();
            p.translation() = pos;
            p.linear() = rot;
        }
        cnoidPosition toPosition() const
        {
            cnoidPosition p;
            p.setIdentity();
            p.translation() = pos;
            p.linear() = rot;
            return p;
        }
        coordinates& operator=(const coordinates& c)
        {
            if (this != &c) {
                pos = c.pos;
                rot = c.rot;
            }
            return *this;
        }
        coordinates& operator=(const cnoidPosition& p)
        {
            pos = p.translation();
            rot = p.linear();
            return *this;
        }
        coordinates &rotate_with_matrix (const Matrix3& mat, const wrt wrt = local )
        {
#if 0
            Matrix3 rot_org(rot);
            if (wrt == local) {
                // alias(rot) = rot * mat;
                _rotm3times(rot, rot_org, mat);
            } else if(wrt == world) {
                // alias(rot) = mat * rot;
                _rotm3times(rot, mat, rot_org);
            } else {
                //std::cerr << "**** invalid wrt! ****" << std::endl;
            }
#endif
            if (wrt == local) {
                // alias(rot) = rot * mat;
                _rotm3times(rot, rot, mat);
            } else if(wrt == world || wrt == parent) {
                // alias(rot) = mat * rot;
                _rotm3times(rot, mat, rot);
            }
            return *this;
        }
        coordinates &rotate_with_matrix (const Matrix3& mat, const coordinates &wrt )
        {
            Matrix3 r2t = wrt.rot.transpose();
            //rot =  W * mat * W^1 * rot
            _rotm3times(r2t, mat, r2t);
            _rotm3times(r2t, wrt.rot, r2t);
            _rotm3times(rot, r2t, rot);
            return *this;
        }
        coordinates &rotate_with_matrix (const Matrix3& mat, const Matrix3 &wrt )
        {
            Matrix3 r2t = wrt.transpose();
            //rot =  W * mat * W^1 * rot
            _rotm3times(r2t, mat, r2t);
            _rotm3times(r2t, wrt, r2t);
            _rotm3times(rot, r2t, rot);
            return *this;
        }
        coordinates &rotate (const double theta, const Vector3& axis, const wrt wrt = local )
        {
            Eigen::AngleAxis<double> tmpr(theta, axis);
            rotate_with_matrix(tmpr.toRotationMatrix(), wrt);
            return *this;
        }
        coordinates &rotate (const double theta, const Vector3& axis, const coordinates &wrt )
        {
            Eigen::AngleAxis<double> tmpr(theta, axis);
            rotate_with_matrix(tmpr.toRotationMatrix(), wrt);
            return *this;
        }
        coordinates &rotate (const double theta, const Vector3& axis, const Matrix3 &wrt )
        {
            Eigen::AngleAxis<double> tmpr(theta, axis);
            rotate_with_matrix(tmpr.toRotationMatrix(), wrt);
            return *this;
        }
        coordinates &orient_with_matrix (const Matrix3& mat, const wrt wrt = local )
        {
            if (wrt == local) {
                _rotm3times(rot, rot, mat);
            } else if (wrt == world || wrt == parent) {
                rot = mat;
            }
            return *this;
        }
        coordinates &orient_with_matrix (const Matrix3& mat, const coordinates &wrt )
        {
            //Matrix3 r2 = wrt.rot;
            //rot =  W * mat
            _rotm3times(rot, wrt.rot, mat);
            return *this;
        }
        coordinates &orient_with_matrix (const Matrix3& mat, const Matrix3 &wrt )
        {
            //Matrix3 r2 = wrt.rot;
            //rot =  W * mat
            _rotm3times(rot, wrt, mat);
            return *this;
        }
        coordinates &orient (const double theta, const Vector3& axis, const wrt wrt = local )
        {
            Eigen::AngleAxis<double> tmpr(theta, axis);
            orient_with_matrix(tmpr.toRotationMatrix(), wrt);
            return *this;
        }
        coordinates &orient (const double theta, const Vector3& axis, const coordinates &wrt )
        {
            Eigen::AngleAxis<double> tmpr(theta, axis);
            orient_with_matrix(tmpr.toRotationMatrix(), wrt);
            return *this;
        }
        coordinates &orient (const double theta, const Vector3& axis, const Matrix3 &wrt )
        {
            Eigen::AngleAxis<double> tmpr(theta, axis);
            orient_with_matrix(tmpr.toRotationMatrix(), wrt);
            return *this;
        }
        //void rotate (const double theta, const Vector3& axis, const coordinates &wrt)
        /* void difference_rotation(Vector3& dif_rot, const coordinates& c) const { */
        /*   dif_rot = rot * matrix_log(rot.transpose() * c.rot); */
        /* } */
        void difference(Vector3& dif_pos, Vector3& dif_rot, const coordinates& c) const
        {
            dif_pos = c.pos - pos;
            _difference_rotation(dif_rot, rot, c.rot);
        }
        void difference_position(Vector3& dif_pos, const coordinates& c) const
        {
            dif_pos = c.pos - pos;
        }
        void difference_rotation(Vector3& dif_rot, const coordinates& c) const
        {
            _difference_rotation(dif_rot, rot, c.rot);
        }
        //abc
        coordinates &inverse_transformation(coordinates& inv) const
        {
            inv.rot = rot.transpose();
            inv.pos = - (inv.rot * pos);
            return inv;
        }
        //void transformation(coordinates& tc, coordinates c, const wrt wrt = local ) const
        coordinates &transformation(coordinates& tc, const coordinates &c, const wrt wrt = local ) const
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
            // return self^-1 * c (local)
            // return c * self^-1 (world)
            coordinates tmp(c); // safe if &tc == &c
            inverse_transformation(tc);
            tc.transform(tmp, wrt);
            return tc;
        }
        coordinates &transformation(coordinates& tc, const coordinates &c, const coordinates &wrt) const
        {
            // return w^-1 * c * self^-1 * w
            coordinates tmp(c); // safe if &tc == &c
            coordinates inv_self;
            inverse_transformation(inv_self);

            wrt.inverse_transformation(tc);
            tc.transform(tmp);
            tc.transform(inv_self);
            tc.transform(wrt);
            return tc;
        }
        coordinates &transform(const coordinates& c, const wrt wrt = local )
        {
            if (wrt == local) {
                // self := self * c
                pos += rot * c.pos;
#if 0
                Matrix3 rot_org(rot);
                _rotm3times(rot, rot_org, c.rot);
#endif
                _rotm3times(rot, rot, c.rot);
            } else if (wrt == world || wrt == parent) {
                // self := c * self
#if 0
                Vector3 p(c.pos);
                Matrix3 r(c.rot);
                p += r * pos;
                _rotm3times(r, c.rot, rot);
                pos = p;
                rot = r;
#endif
                pos = c.pos + (c.rot * pos);
                _rotm3times(rot, c.rot, rot);
            } else {
                //std::cerr << "**** invalid wrt! ****" << std::endl;
            }
            return *this;
        }
        coordinates &transform(const coordinates& c, const coordinates &wrt )
        {
            coordinates tmp(wrt);
            coordinates inv_wrt;
            //self := w * c * w^-1 * self
            wrt.inverse_transformation(inv_wrt);
            tmp.transform(c);
            tmp.transform(inv_wrt);
            tmp.transform(*this);

            *this = tmp;
            return *this;
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
        coordinates &move_to (const coordinates &c, const wrt wrt = local )
        {
            if (wrt == local) {
                transform(c);
            } else if (wrt == world || wrt == parent) {
                *this = c;
            }
            return *this;
        }
        coordinates &move_to (const coordinates &c, const coordinates &wrt)
        {
            coordinates tmp(wrt);
            tmp.transform(c);
            *this = tmp;
            return *this;
        }
        coordinates &translate(const Vector3 &vec, const wrt wrt = local )
        {
            if (wrt == local) {
                pos += rot * vec;
            } else if (wrt == world || wrt == parent) {
                pos += vec;
            }
            return *this;
        }
        coordinates &translate(const Vector3 &vec, const coordinates &wrt)
        {
            pos += wrt.rot * vec;
            return *this;
        }
        coordinates &locate(const Vector3 &vec, const wrt wrt = local )
        {
            if (wrt == local) {
                pos += rot * vec;
            } else if (wrt == world || wrt == parent) {
                pos = vec;
            }
            return *this;
        }
        coordinates &locate(const Vector3 &vec, const coordinates &wrt)
        {
            pos = wrt.pos + (wrt.rot * vec);
            return *this;
        }
        // new functions
        void x_axis(Vector3 &ax) const {
            ax = rot.col(0);
        }
        void y_axis(Vector3 &ay) const {
            ay = rot.col(1);
        }
        void z_axis(Vector3 &az) const {
            az = rot.col(2);
        }
        void rotationAngle(double &angle, Vector3 &axis) const {
            AngleAxis _a(rot);
            angle = _a.angle();
            axis  = _a.axis();
        }
        void setRotationAngle(const double angle, const Vector3 &axis, bool normalize = true) {
            Vector3 ax_(axis);
            double an_ = angle;
            if (normalize) {
                ax_.normalize();
            } else {
                an_ *= ax_.norm();
                ax_.normalize();
            }
            AngleAxis aa(an_, ax_);
            rot = aa;
        }
        void rotNormalize() {
            normalizeRotation(rot);
        }
        void getRPY(Vector3 &rpy) const {
            rpy = rpyFromRot(rot);
        }
        void setRPY(const Vector3 &rpy) {
            rot = rotFromRpy(rpy);
        }
        void setRPY(double r, double p, double y) {
            rot = rotFromRpy(r,p,y);
        }
        coordinates &inverse() {
            Matrix3 rot_t = rot.transpose();
            pos = -1 * (rot_t * pos);
            rot = rot_t;
            return *this;
        }
        coordinates &mid_coords(coordinates &mid_coords, const double p, const coordinates &c2, const double eps = 0.00001) const
        {
            mid_coords.pos = (1 - p) * pos + p * c2.pos;
            _mid_rot(mid_coords.rot, p, rot, c2.rot, eps);
            return mid_coords;
        }
        // euler-angle
        // rotation-angle
        // reset_coords
    };

typedef std::shared_ptr< coordinates > coordinatesPtr;
#if 0 // deprecated
    inline void mid_coords(coordinates& mid_coords, const double p, const coordinates& c1, const coordinates& c2, const double eps = 0.00001)
    {
        mid_coords.pos = (1 - p) * c1.pos + p * c2.pos;
        _mid_rot(mid_coords.rot, p, c1.rot, c2.rot, eps);
    };
#endif

    //// for cnoid::Poisition
    inline void rotate_with_matrix(cnoidPosition &coords, const Matrix3 &mat, const coordinates::wrt wrt = coordinates::wrt::local )
    {
        Matrix3 rot = coords.linear();
        if (wrt == coordinates::wrt::local) {
            _rotm3times(rot, rot, mat);
        } else if (wrt == coordinates::wrt::world || wrt == coordinates::wrt::parent) {
            _rotm3times(rot, mat, rot);
        } else {
            return;
        }
        coords.linear() = rot;
    }

    inline void rotate(cnoidPosition &coords, const double theta, const Vector3& axis, const coordinates::wrt wrt = coordinates::wrt::local )
    {
        AngleAxis tmpr(theta, axis);
        rotate_with_matrix(coords, tmpr.toRotationMatrix(), wrt);
    }

    inline void difference(const cnoidPosition &coords, Vector3& dif_pos, Vector3& dif_rot, const cnoidPosition& c)
    {
        dif_pos = c.translation() - coords.translation();
        _difference_rotation(dif_rot, coords.linear(), c.linear());
    }
    inline void difference_rotation(const cnoidPosition &coords, Vector3& dif_rot, const cnoidPosition& c)
    {
        _difference_rotation(dif_rot, coords.linear(), c.linear());
    }
    inline void difference_position(const cnoidPosition &coords, Vector3& dif_pos, const cnoidPosition& c)
    {
        dif_pos = c.translation() - coords.translation();
    }
    inline void inverse_transformation(const cnoidPosition &coords, cnoidPosition& inv)
    {
        inv = coords.inverse();
    }

    inline void transform(cnoidPosition &coords, const cnoidPosition& c, const coordinates::wrt wrt = coordinates::wrt::local )
    {
        if (wrt == coordinates::wrt::local) {
            cnoidPosition tmp = (coords * c);
            Matrix3 rot;
            _rotm3times(rot, coords.linear(), c.linear());
            coords.translation() = tmp.translation();
            coords.linear() = rot;
        } else if (wrt == coordinates::wrt::world || wrt == coordinates::wrt::parent) {
            cnoidPosition tmp = (c * coords);
            Matrix3 rot;
            _rotm3times(rot, c.linear(), coords.linear());
            coords.translation() = tmp.translation();
            coords.linear() = rot;
        }
    }

    inline void transformation(const cnoidPosition &coords, cnoidPosition& trans_coords, const cnoidPosition &c, const coordinates::wrt wrt = coordinates::wrt::local )
    {
#if 0
        if(wrt == coordinates::wrt::local) {
            // coords^-1 * c
            inverse_transformation(coords, trans_coords);
            transform(trans_coords, c);
        } else if (wrt == coordinates::wrt::world || wrt == coordinates::wrt::parent) {
            // c * coords^-1
            cnoidPosition inv_trans;
            inverse_transformation(coords, inv_trans);
            trans_coords = c;
            transform(trans_coords, inv_trans);
        }
#endif
        cnoidPosition tmp(c);
        inverse_transformation(coords, trans_coords);
        transform(trans_coords, tmp, wrt);
    }

}
