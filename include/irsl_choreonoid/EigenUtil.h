#include <cnoid/EigenTypes>

#pragma once

namespace cnoid {
    inline bool eps_eq(const double a, const double b, const double eps = 0.00001) {
        return fabs((a)-(b)) <= eps;
    };

    void calcRodrigues(Matrix3& out_R, const Vector3& axis, double q);

    Vector3 matrix_log(const Matrix3& m);
    void rotm3times (Matrix3& m12, const Matrix3& m1, const Matrix3& m2);
    void difference_rotation(Vector3& ret_dif_rot, const Matrix3& self_rot, const Matrix3& target_rot);
    void mid_rot(Matrix3& mid_rot, const double p, const Matrix3& rot1, const Matrix3& rot2, const double eps = 0.00001);

    void mid_coords(Position &mid_coords, const double p, const Position &c1, const Position &c2, const double eps = 0.00001);


  //from EusLisp definition
  //rotate_vector
  //inverse_rotate_vector
  //transform_vector
  //inverse_transform_vector
  //
  //transformation
  //inverse_transformation :local :world :wrt
  //
  //move-to :local :world :wrt
  //translate :local :world :wrt
  //locate :local :world :wrt
  //transform  :local :world :wrt  WAW^-1 T
}
