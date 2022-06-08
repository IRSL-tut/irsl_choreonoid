#include "irsl_choreonoid/EigenUtil.h"

namespace cnoid {
  //void calcRodrigues(Matrix3& out_R, const Vector3& axis, double q)
  //AngleAxis(q, axis) 

  Vector3 matrix_log(const Matrix3& m) {
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
  void rotm3times (Matrix3& m12, const Matrix3& m1, const Matrix3& m2) {
    Quaternion eiq1(m1);
    Quaternion eiq2(m2);
    Quaternion eiq3;
    eiq3 = eiq1 * eiq2;
    eiq3.normalize();
    m12 = eiq3.toRotationMatrix();
  }

  void difference_rotation(Vector3& ret_dif_rot, const Matrix3& self_rot, const Matrix3& target_rot)
  {
    //ret_dif_rot = self_rot * hrp::omegaFromRot(self_rot.transpose() * target_rot);
    ret_dif_rot = self_rot * matrix_log ( self_rot.transpose() * target_rot );
  }

  void mid_rot(Matrix3& mid_rot, const double p, const Matrix3& rot1, const Matrix3& rot2, const double eps) {
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
  };

#if 0
  void mid_coords(coordinates& mid_coords, const double p, const coordinates& c1, const coordinates& c2, const double eps) { // eps = 0.00001
    mid_coords.pos = (1 - p) * c1.pos + p * c2.pos;
    mid_rot(mid_coords.rot, p, c1.rot, c2.rot, eps);
  };
#endif
  void mid_coords(Position &mid_coords, const double p, const Position &c1, const Position &c2, const double eps) { // eps = 0.00001
    Matrix3 c1_rot(c1.linear());
    Matrix3 c2_rot(c2.linear());
    Matrix3 ret_rot;
    mid_rot(ret_rot, p, c1_rot, c2_rot, eps);
    mid_coords.translation() = (1 - p) * c1.translation() + p * c2.translation();
    mid_coords.linear() = ret_rot;
  }
}
