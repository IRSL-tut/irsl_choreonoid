/**
   @author YoheiKakiuchi
*/

#include <memory>
#include <pybind11/pybind11.h>

#include "irsl_choreonoid/EigenUtil.h"
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>

using namespace cnoid;
namespace py = pybind11;

using Matrix4RM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Matrix3RM = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

Matrix4RM mid_coords_(const double p, Eigen::Ref<const Matrix4RM> c1, Eigen::Ref<const Matrix4RM> c2, const double eps) {
  Position cnoid_pos;
  Position p_c1(c1), p_c2(c2);
  cnoid_pos.setIdentity();
  mid_coords(cnoid_pos, p, p_c1, p_c2, eps);
  return cnoid_pos.matrix();
}

PYBIND11_MODULE(IRSLUtil, m)
{
    m.doc() = "Utility for choreonoid";

    //py::module::import("cnoid.Util");

    m.def("mid_coords", mid_coords_, py::arg("p"), py::arg("c1"), py::arg("c2"), py::arg("eps") = 0.00001);

    m.def("PositionInverse", [](Eigen::Ref<const Matrix4RM> in_p) { Position p(in_p); return p.inverse().matrix(); });
    m.def("Position_translation", [](Eigen::Ref<const Matrix4RM> in_p) { Position p(in_p); return Vector3(p.translation()); });
    m.def("Position_quaternion", [](Eigen::Ref<const Matrix4RM> in_p) { Quaternion q(Position(in_p).linear());
        return Vector4(q.x(), q.y(), q.z(), q.w()); });
    m.def("Position_rotation", [](Eigen::Ref<const Matrix4RM> in_p) { Position p(in_p); return Matrix3RM(p.linear()); });
    m.def("rotationToQuaternion", [](Eigen::Ref<const Matrix3RM> rot) { Quaternion q(rot);
        return Vector4(q.x(), q.y(), q.z(), q.w()); });
    m.def("quaternionToRotation", [](Eigen::Ref<const Vector4> q) { return Matrix3RM(Quaternion(q)); });

    m.def("cnoidPosition", [](Eigen::Ref<const Matrix3RM> rot, Eigen::Ref<const Vector3> trans) -> Isometry3::MatrixType {
        Position ret; ret.setIdentity(); ret.linear() = rot; ret.translation() = trans; return ret.matrix(); });
    m.def("cnoidPosition", [](Eigen::Ref<const Vector4> q, Eigen::Ref<const Vector3> trans) -> Isometry3::MatrixType {
        Position ret; ret.setIdentity(); ret.linear() = Matrix3(Quaternion(q)); ret.translation() = trans; return ret.matrix(); });
    m.def("cnoidPosition", [](Eigen::Ref<const Vector3> trans) -> Isometry3::MatrixType {
        Position ret; ret.setIdentity(); ret.translation() = trans; return ret.matrix(); });
    m.def("cnoidPosition", [](Eigen::Ref<const Matrix3RM> rot) -> Isometry3::MatrixType {
        Position ret; ret.setIdentity(); ret.linear() = rot; return ret.matrix(); });
    m.def("cnoidPosition", [](Eigen::Ref<const Vector4> q) -> Isometry3::MatrixType {
        Position ret; ret.setIdentity(); ret.linear() = Matrix3(Quaternion(q)); return ret.matrix(); });

    m.def("normalizeVector", [](Eigen::Ref<const Vector3> v3) { return v3.normalized(); });
    m.def("normalizeVector", [](Eigen::Ref<const Vector4> v4) { return v4.normalized(); });

    m.def("angleAxisNormalized", [](double angle, Eigen::Ref<const Vector3> axis){
        Vector3 ax_ = axis.normalized(); return Matrix3RM(AngleAxis(angle, ax_)); });
}
