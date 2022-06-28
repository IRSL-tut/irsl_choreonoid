/**
   @author YoheiKakiuchi
*/

#include <sstream>
#include <pybind11/pybind11.h>

#include "irsl_choreonoid/EigenUtil.h"
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>

using namespace cnoid;
namespace py = pybind11;

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

    m.def("eps_eq", [] (const double a, const double b, const double eps) { return eps_eq(a, b, eps); },
          py::arg("a"), py::arg("b"), py::arg("eps") = 0.00001);
    m.def("eps_eq", [] (ref_vec3 a, ref_vec3 b, const double eps) {
            if (!eps_eq(a(0), b(0), eps)) return false;
            if (!eps_eq(a(1), b(1), eps)) return false;
            if (!eps_eq(a(2), b(2), eps)) return false;
            return true; }, py::arg("a"), py::arg("b"), py::arg("eps") = 0.00001);
    m.def("eps_eq", [] (ref_vec4 a, ref_vec4 b, const double eps) {
            if (!eps_eq(a(0), b(0), eps)) return false;
            if (!eps_eq(a(1), b(1), eps)) return false;
            if (!eps_eq(a(2), b(2), eps)) return false;
            if (!eps_eq(a(3), b(3), eps)) return false;
            return true; }, py::arg("a"), py::arg("b"), py::arg("eps") = 0.00001);
    m.def("eps_eq", [] (ref_mat3 a, ref_mat3 b, const double eps) {
            const double *aptr = a.data();
            const double *bptr = b.data();
            for(size_t i = 0; i < a.size(); i++) {
                if (!eps_eq(*aptr++, *bptr++, eps)) return false;
            }
            return true; }, py::arg("a"), py::arg("b"), py::arg("eps") = 0.00001);
    m.def("eps_eq", [] (ref_mat4 a, ref_mat4 b, const double eps) {
            const double *aptr = a.data();
            const double *bptr = b.data();
            for(size_t i = 0; i < a.size(); i++) {
                if (!eps_eq(*aptr++, *bptr++, eps)) return false;
            }
            return true; }, py::arg("a"), py::arg("b"), py::arg("eps") = 0.00001);
    m.def("PositionInverse", [](ref_mat4 in_p) -> Matrix4RM { Position p(in_p); return p.inverse().matrix(); });
    m.def("Position_translation", [](ref_mat4 in_p) { Position p(in_p); return Vector3(p.translation()); });
    m.def("Position_quaternion", [](ref_mat4 in_p) { Quaternion q(Position(in_p).linear());
            return Vector4(q.x(), q.y(), q.z(), q.w()); });
    m.def("Position_rotation", [](ref_mat4 in_p) { Position p(in_p); return Matrix3RM(p.linear()); });
    m.def("rotationToQuaternion", [](ref_mat3 rot) { Quaternion q(rot);
            return Vector4(q.x(), q.y(), q.z(), q.w()); });
    m.def("quaternionToRotation", [](ref_vec4 q) { return Matrix3RM(Quaternion(q)); });

    m.def("cnoidPosition", [](ref_mat3 rot, ref_vec3 trans) -> Matrix4RM {
            Position ret; ret.setIdentity(); ret.linear() = rot; ret.translation() = trans; return ret.matrix(); });
    m.def("cnoidPosition", [](ref_vec4 q, ref_vec3 trans) -> Matrix4RM {
            Position ret; ret.setIdentity(); ret.linear() = Matrix3(Quaternion(q)); ret.translation() = trans; return ret.matrix(); });
    m.def("cnoidPosition", [](ref_vec3 trans) -> Matrix4RM {
            Position ret; ret.setIdentity(); ret.translation() = trans; return ret.matrix(); });
    m.def("cnoidPosition", [](ref_mat3 rot) -> Matrix4RM {
            Position ret; ret.setIdentity(); ret.linear() = rot; return ret.matrix(); });
    m.def("cnoidPosition", [](ref_vec4 q) -> Matrix4RM {
            Position ret; ret.setIdentity(); ret.linear() = Matrix3(Quaternion(q)); return ret.matrix(); });

    m.def("normalizeVector", [](ref_vec3 v3) { return v3.normalized(); });
    m.def("normalizeVector", [](ref_vec4 v4) { return v4.normalized(); });

    m.def("angleAxisNormalized", [](double angle, ref_vec3 axis) {
            Vector3 ax_ = axis.normalized(); return Matrix3RM(AngleAxis(angle, ax_)); });
    m.def("Position_rotate_with_matrix", [](ref_mat4 cds, ref_mat3 mat, const std::string &wrt) -> Matrix4RM {
            Position coords(cds);
            Matrix3 mat_(mat);
            if (wrt == "local") {
                rotate_with_matrix(coords, mat_, coordinates::wrt::local);
            } else if (wrt == "world") {
                rotate_with_matrix(coords, mat_, coordinates::wrt::world);
            }
            return coords.matrix(); },
        py::arg("cds"), py::arg("mat"), py::arg("wrt") = "local");
    m.def("Position_rotate", [](ref_mat4 cds, const double theta, ref_vec3 axis, const std::string &wrt) -> Matrix4RM {
            Position coords(cds);
            if (wrt == "local") {
                rotate(coords, theta, axis, coordinates::wrt::local);
            } else if (wrt == "world") {
                rotate(coords, theta, axis, coordinates::wrt::world);
            }
            return coords.matrix(); },
        py::arg("cds"), py::arg("theta"), py::arg("axis"), py::arg("wrt") = "local");
    m.def("Position_transform", [](ref_mat4 cds, ref_mat4 c, const std::string &wrt) -> Matrix4RM {
            Position coords(cds);
            Position c_(c);
            if (wrt == "local") {
                transform(coords, c_, coordinates::wrt::local);
            } else if (wrt == "world") {
                transform(coords, c_, coordinates::wrt::world);
            }
            return coords.matrix(); },
        py::arg("cds"), py::arg("c"), py::arg("wrt") = "local");
    m.def("Position_transformation", [](ref_mat4 cds, ref_mat4 c, const std::string &wrt) -> Matrix4RM {
            Position coords(cds);
            Position c_(c);
            Position trans_c;
            if (wrt == "local") {
                transformation(coords, trans_c, c_, coordinates::wrt::local);
            } else if (wrt == "world") {
                transformation(coords, trans_c, c_, coordinates::wrt::world);
            }
            return trans_c.matrix(); },
        py::arg("cds"), py::arg("c"), py::arg("wrt") = "local");

    py::class_< coordinates, coordinatesPtr > coords_cls(m, "coordinates");

    py::enum_<coordinates::wrt>(coords_cls, "wrt")
    .value("local", coordinates::wrt::local)
    .value("world", coordinates::wrt::world)
    .value("parent", coordinates::wrt::parent)
    .export_values();

    coords_cls.def(py::init<>())
    .def(py::init([](ref_vec3 trs, ref_mat3 rot)
                  { return new coordinates(trs, rot); }))
    .def(py::init([](ref_vec3 trs)
                  { Vector3 v(trs); return new coordinates(v); }))
    .def(py::init([](ref_mat3 rot)
                  { Matrix3 m(rot); return new coordinates(m); }))
    .def(py::init([](ref_vec4 rot)
                  { Quaternion q(rot); return new coordinates(q); }))
    .def(py::init([](ref_vec3 trs, ref_vec4 rot)
                  { Quaternion q(rot); return new coordinates(trs, q); }))
    .def(py::init([](ref_mat4 position)
                  { Position p(position); return new coordinates(p); }))
    .def("__repr__", [](const coordinates &self) {
            std::stringstream ss;
            ss << "<coordinates ";
            Quaternion q(self.rot);
            ss << self.pos(0); ss << " ";
            ss << self.pos(1); ss << " ";
            ss << self.pos(2); ss << " / ";
            ss << q.x(); ss << " ";
            ss << q.y(); ss << " ";
            ss << q.z(); ss << " ";
            ss << q.w(); ss << " >";
            return ss.str();
        })
    .def_property("pos",
                  [](coordinates &self) { return self.pos; },
                  [](coordinates &self, ref_vec3 vec) { self.pos = vec; })
    .def_property("rot",
                  [](coordinates &self) { return self.rot; },
                  [](coordinates &self, ref_mat3 mat) { self.rot = mat; })
    .def("equal", &coordinates::equal, py::arg("cds"), py::arg("eps") = 0.00001)
    .def("toPosition",
         [](const coordinates &self) -> Matrix4RM { Position p; self.toPosition(p); return p.matrix(); })
    .def("rotate_with_matrix",
         [](coordinates &self, ref_mat3 mat, coordinates::wrt wrt)
         {  self.rotate_with_matrix(mat, wrt); },
         py::arg("mat"), py::arg("wrt") = coordinates::wrt::local )
    .def("rotate_with_matrix",
         [](coordinates &self, ref_mat3 mat, const coordinates &wrt)
         {  self.rotate_with_matrix(mat, wrt); } )
    .def("rotate_with_matrix",
         [](coordinates &self, ref_mat3 mat, ref_mat3 wrt)
         {  self.rotate_with_matrix(mat, wrt); } )
    .def("rotate",
         [](coordinates &self, double th_, ref_vec3 ax_, coordinates::wrt wrt)
         { self.rotate(th_, ax_, wrt); },
         py::arg("theta"), py::arg("axis"), py::arg("wrt") = coordinates::wrt::local )
    .def("rotate",
         [](coordinates &self, double th_, ref_vec3 ax_, const coordinates &wrt)
         { self.rotate(th_, ax_, wrt); } )
    .def("rotate",
         [](coordinates &self, double th_, ref_vec3 ax_, ref_mat3 wrt)
         { self.rotate(th_, ax_, wrt); } )
    .def("orient_with_matrix",
         [](coordinates &self, ref_mat3 mat, coordinates::wrt wrt)
         {  self.orient_with_matrix(mat, wrt); },
         py::arg("mat"), py::arg("wrt") = coordinates::wrt::local )
    .def("orient_with_matrix",
         [](coordinates &self, ref_mat3 mat, const coordinates &wrt)
         {  self.orient_with_matrix(mat, wrt); } )
    .def("orient_with_matrix",
         [](coordinates &self, ref_mat3 mat, ref_mat3 wrt)
         {  self.orient_with_matrix(mat, wrt); } )
    .def("orient",
         [](coordinates &self, double th_, ref_vec3 ax_, coordinates::wrt wrt)
         { self.orient(th_, ax_, wrt); },
         py::arg("theta"), py::arg("axis"), py::arg("wrt") = coordinates::wrt::local )
    .def("orient",
         [](coordinates &self, double th_, ref_vec3 ax_, const coordinates &wrt)
         { self.orient(th_, ax_, wrt); } )
    .def("orient",
         [](coordinates &self, double th_, ref_vec3 ax_, ref_mat3 wrt)
         { self.orient(th_, ax_, wrt); } )
    .def("difference_rotation",
         [](const coordinates &self, coordinates &c) {
             Vector3 ret; self.difference_rotation(ret, c); return ret;
         } )
    .def("difference_position",
         [](const coordinates &self, coordinates &c) {
             Vector3 ret; self.difference_position(ret, c); return ret;
         } )
    .def("inverse_transformation",
         [](const coordinates &self) {
             coordinates ret; self.inverse_transformation(ret); return ret;
         } )
    .def("transformation",
         [](const coordinates &self, const coordinates &c, coordinates::wrt wrt) {
             coordinates ret; self.transformation(ret, c, wrt); return ret;
         }, py::arg("coords"), py::arg("wrt") = coordinates::wrt::local )
    .def("transformation",
         [](const coordinates &self, const coordinates &c, const coordinates &wrt) {
             coordinates ret; self.transformation(ret, c, wrt); return ret;
         } )
    .def("transform",
         [](coordinates &self, const coordinates &c, coordinates::wrt wrt)
         { self.transform(c, wrt); }, py::arg("coords"), py::arg("wrt") = coordinates::wrt::local )
    .def("transform",
         [](coordinates &self, const coordinates &c, const coordinates &wrt)
         { self.transform(c, wrt); } )
    ////
    .def("rotate_vector",
         [](const coordinates &self, Eigen::Ref<Vector3> vec) { Vector3 v(vec);
             self.rotate_vector(v); vec = v; return vec; })
    .def("inverse_rotate_vector",
         [](const coordinates &self, Eigen::Ref<Vector3> vec) { Vector3 v(vec);
             self.inverse_rotate_vector(v); vec = v; return vec; })
    .def("transform_vector",
         [](const coordinates &self, Eigen::Ref<Vector3> vec) { Vector3 v(vec);
             self.transform_vector(v); vec = v; return vec; })
    .def("inverse_transform_vector",
         [](const coordinates &self, Eigen::Ref<Vector3> vec) { Vector3 v(vec);
             self.inverse_transform_vector(v); vec = v; return vec; })
    ///
    .def("translate",
         [](coordinates &self, ref_vec3 vec, coordinates::wrt wrt) {
             self.translate(vec, wrt); }, py::arg("vector"), py::arg("wrt") = coordinates::wrt::local )
    .def("translate",
         [](coordinates &self, ref_vec3 vec, const coordinates &wrt) {
             self.translate(vec, wrt); } )
    .def("locate",
         [](coordinates &self, ref_vec3 vec, coordinates::wrt wrt) {
             self.locate(vec, wrt); }, py::arg("vector"), py::arg("wrt") = coordinates::wrt::local )
    .def("locate",
         [](coordinates &self, ref_vec3 vec, const coordinates &wrt) {
             self.locate(vec, wrt); } )
    .def("move_to",
         [](coordinates &self, const coordinates &vec, coordinates::wrt wrt) {
             self.move_to(vec, wrt); }, py::arg("vector"), py::arg("wrt") = coordinates::wrt::local )
    .def("move_to",
         [](coordinates &self, const coordinates &vec, const coordinates &wrt) {
             self.move_to(vec, wrt); } )
    ;
}
