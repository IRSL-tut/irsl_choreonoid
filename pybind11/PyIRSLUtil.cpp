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

typedef Eigen::Ref<const Matrix4RM> ref_mat4;
typedef Eigen::Ref<const Matrix3RM> ref_mat3;
typedef Eigen::Ref<const Vector4>   ref_vec4;
typedef Eigen::Ref<const Vector3>   ref_vec3;


Matrix4RM mid_coords_(const double p, ref_mat4 c1, ref_mat4 c2, const double eps)
{
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

    m.def("PositionInverse", [](ref_mat4 in_p) { Position p(in_p); return p.inverse().matrix(); });
    m.def("Position_translation", [](ref_mat4 in_p) { Position p(in_p); return Vector3(p.translation()); });
    m.def("Position_quaternion", [](ref_mat4 in_p) { Quaternion q(Position(in_p).linear());
            return Vector4(q.x(), q.y(), q.z(), q.w()); });
    m.def("Position_rotation", [](ref_mat4 in_p) { Position p(in_p); return Matrix3RM(p.linear()); });
    m.def("rotationToQuaternion", [](ref_mat3 rot) { Quaternion q(rot);
            return Vector4(q.x(), q.y(), q.z(), q.w()); });
    m.def("quaternionToRotation", [](ref_vec4 q) { return Matrix3RM(Quaternion(q)); });

    m.def("cnoidPosition", [](ref_mat3 rot, ref_vec3 trans) -> Isometry3::MatrixType {
            Position ret; ret.setIdentity(); ret.linear() = rot; ret.translation() = trans; return ret.matrix(); });
    m.def("cnoidPosition", [](ref_vec4 q, ref_vec3 trans) -> Isometry3::MatrixType {
            Position ret; ret.setIdentity(); ret.linear() = Matrix3(Quaternion(q)); ret.translation() = trans; return ret.matrix(); });
    m.def("cnoidPosition", [](ref_vec3 trans) -> Isometry3::MatrixType {
            Position ret; ret.setIdentity(); ret.translation() = trans; return ret.matrix(); });
    m.def("cnoidPosition", [](ref_mat3 rot) -> Isometry3::MatrixType {
            Position ret; ret.setIdentity(); ret.linear() = rot; return ret.matrix(); });
    m.def("cnoidPosition", [](ref_vec4 q) -> Isometry3::MatrixType {
            Position ret; ret.setIdentity(); ret.linear() = Matrix3(Quaternion(q)); return ret.matrix(); });

    m.def("normalizeVector", [](ref_vec3 v3) { return v3.normalized(); });
    m.def("normalizeVector", [](ref_vec4 v4) { return v4.normalized(); });

    m.def("angleAxisNormalized", [](double angle, ref_vec3 axis) {
            Vector3 ax_ = axis.normalized(); return Matrix3RM(AngleAxis(angle, ax_)); });
    m.def("Position_rotate_with_matrix", [](ref_mat4 cds, ref_mat3 mat, const std::string &wrt) -> Isometry3::MatrixType {
            Position coords(cds);
            Matrix3 mat_(mat);
            if (wrt == "local") {
                rotate_with_matrix(coords, mat_, local);
            } else if (wrt == "world") {
                rotate_with_matrix(coords, mat_, world);
            }
            return coords.matrix(); },
        py::arg("cds"), py::arg("mat"), py::arg("wrt") = "local");
    m.def("Position_rotate", [](ref_mat4 cds, const double theta, ref_vec3 axis, const std::string &wrt) -> Isometry3::MatrixType {
            Position coords(cds);
            if (wrt == "local") {
                rotate(coords, theta, axis, local);
            } else if (wrt == "world") {
                rotate(coords, theta, axis, world);
            }
            return coords.matrix(); },
        py::arg("cds"), py::arg("theta"), py::arg("axis"), py::arg("wrt") = "local");
    m.def("Position_transform", [](ref_mat4 cds, ref_mat4 c, const std::string &wrt) -> Isometry3::MatrixType {
            Position coords(cds);
            Position c_(c);
            if (wrt == "local") {
                transform(coords, c_, local);
            } else if (wrt == "world") {
                transform(coords, c_, world);
            }
            return coords.matrix(); },
        py::arg("cds"), py::arg("c"), py::arg("wrt") = "local");
    m.def("Position_transformation", [](ref_mat4 cds, ref_mat4 c, const std::string &wrt) -> Isometry3::MatrixType {
            Position coords(cds);
            Position c_(c);
            Position trans_c;
            if (wrt == "local") {
                transformation(coords, trans_c, c_, local);
            } else if (wrt == "world") {
                transformation(coords, trans_c, c_, world);
            }
            return trans_c.matrix(); },
        py::arg("cds"), py::arg("c"), py::arg("wrt") = "local");
}
