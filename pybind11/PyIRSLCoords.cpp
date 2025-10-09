/**
   @author YoheiKakiuchi
*/

#include <sstream>
#include <pybind11/pybind11.h>

#include "irsl_choreonoid/Coordinates.h"
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>
#include <cnoid/Body>
#include <cnoid/Link>
// for computeRotationScaling
#include <Eigen/SVD>

typedef Eigen::Ref<Matrix4RM> ref_noconst_mat4;
typedef Eigen::Ref<Matrix3RM> ref_noconst_mat3;
typedef Eigen::Ref<cnoid::Vector4>   ref_noconst_vec4;
typedef Eigen::Ref<cnoid::Vector3>   ref_noconst_vec3;

using namespace cnoid;
namespace py = pybind11;

Matrix4RM mid_coords_pos_(const double p, ref_mat4 c1, ref_mat4 c2, const double eps)
{
    cnoidPosition cnoid_pos;
    cnoidPosition p_c1(c1), p_c2(c2);
    cnoid_pos.setIdentity();
    mid_coords_pos(cnoid_pos, p, p_c1, p_c2, eps);
    return cnoid_pos.matrix();
}

PYBIND11_MODULE(IRSLCoords, m)
{
    m.doc() = R"__IRSL__(Functions for manipulating 4x4 matrix as homogeneous transformation matrix)__IRSL__";

    py::module::import("cnoid.Body");

#if 0 // deprecated
    m.def("mid_coords", [] (const double p, const coordinates &c1, const coordinates &c2, const double eps) {
            coordinatesPtr ret(new coordinates());
            mid_coords(*ret, p, c1, c2, eps);
            return ret; }, py::arg("p"), py::arg("c1"), py::arg("c2"), py::arg("eps") = 0.00001);
#endif
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

    m.def("computeRotationScaling", [] (ref_mat3 affine) {
        Eigen::Affine3d af;
        af.matrix()(0, 0) = affine(0, 0);
        af.matrix()(0, 1) = affine(0, 1);
        af.matrix()(0, 2) = affine(0, 2);
        af.matrix()(1, 0) = affine(1, 0);
        af.matrix()(1, 1) = affine(1, 1);
        af.matrix()(1, 2) = affine(1, 2);
        af.matrix()(2, 0) = affine(2, 0);
        af.matrix()(2, 1) = affine(2, 1);
        af.matrix()(2, 2) = affine(2, 2);
        //Eigen::Matrix3d rot, scl;
        std::vector<Matrix3RM> res(2);
        af.computeRotationScaling(&(res[0]), &(res[1]));
        return res;
    });
    m.def("computeScalingRotation", [] (ref_mat3 affine) {
        Eigen::Affine3d af;
        af.matrix()(0, 0) = affine(0, 0);
        af.matrix()(0, 1) = affine(0, 1);
        af.matrix()(0, 2) = affine(0, 2);
        af.matrix()(1, 0) = affine(1, 0);
        af.matrix()(1, 1) = affine(1, 1);
        af.matrix()(1, 2) = affine(1, 2);
        af.matrix()(2, 0) = affine(2, 0);
        af.matrix()(2, 1) = affine(2, 1);
        af.matrix()(2, 2) = affine(2, 2);
        //Eigen::Matrix3d rot, scl;
        std::vector<Matrix3RM> res(2);
        af.computeScalingRotation(&(res[0]), &(res[1]));
        return res;
    });

    /// for cnoid::Position
    m.def("PositionInverse", [](ref_mat4 _position) -> Matrix4RM { cnoidPosition p(_position); return p.inverse().matrix(); }, R"__IRSL__(
Generating inverse transformation matrix

Args:
    _position (numpy.array) : 4x4 homogeneous transformation matrix, using in Choreonoid

Returns:
    numpy.array : 4x4 matrix, inverse matrix of _position
          )__IRSL__");
    m.def("Position_translation", [](ref_mat4 _position) { cnoidPosition p(_position); return Vector3(p.translation()); }, R"__IRSL__(
Extracting translation part of transformation matrix

Args:
    _position (numpy.array) : 4x4 homogeneous transformation matrix, using in Choreonoid

Returns:
    numpy.array : 1x3 vector, translation part of _position
          )__IRSL__");
    m.def("Position_quaternion", [](ref_mat4 _position) { Quaternion q(cnoidPosition(_position).linear());
            return Vector4(q.x(), q.y(), q.z(), q.w()); }, R"__IRSL__(
Extracting rotation part of transformation matrix

Args:
    _position (numpy.array) : 4x4 homogeneous transformation matrix, using in Choreonoid

Returns:
    numpy.array : 1x4 vector, quaternion(x,y,z,w), rotation part of _position
          )__IRSL__");
    m.def("Position_rotation", [](ref_mat4 _position) { cnoidPosition p(_position); return Matrix3RM(p.linear()); }, R"__IRSL__(
Extracting rotation part of transformation matrix

Args:
    _position (numpy.array) : 4x4 homogeneous transformation matrix, using in Choreonoid

Returns:
    numpy.array : 3x3 matrix, rotation matrix, rotation part of _position
          )__IRSL__");
    m.def("rotationToQuaternion", [](ref_mat3 rot) { Quaternion q(rot);
            return Vector4(q.x(), q.y(), q.z(), q.w()); }, R"__IRSL__(
Converting rotation matrix to quaternion

Args:
    rot (numpy.array) : 3x3, rotation matrix

Returns:
    numpy.array : 1x4 vector, quaternion(x,y,z,w)
          )__IRSL__");
    m.def("quaternionToRotation", [](ref_vec4 q) { return Matrix3RM(Quaternion(q)); }, R"__IRSL__(
Converting quaternion to rotation matrix

Args:
    rot (numpy.array) : 1x4 vector, quaternion(x,y,z,w)

Returns:
    numpy.array : 3x3, rotation matrix
          )__IRSL__");

    m.def("cnoidPosition", [](ref_mat3 rot, ref_vec3 trans) -> Matrix4RM {
            cnoidPosition ret; ret.setIdentity(); ret.linear() = rot; ret.translation() = trans; return ret.matrix(); });
    m.def("cnoidPosition", [](ref_vec4 q, ref_vec3 trans) -> Matrix4RM {
            cnoidPosition ret; ret.setIdentity(); ret.linear() = Matrix3(Quaternion(q)); ret.translation() = trans; return ret.matrix(); });
    m.def("cnoidPosition", [](ref_vec3 trans) -> Matrix4RM {
            cnoidPosition ret; ret.setIdentity(); ret.translation() = trans; return ret.matrix(); });
    m.def("cnoidPosition", [](ref_mat3 rot) -> Matrix4RM {
            cnoidPosition ret; ret.setIdentity(); ret.linear() = rot; return ret.matrix(); });
    m.def("cnoidPosition", [](ref_vec4 q) -> Matrix4RM {
            cnoidPosition ret; ret.setIdentity(); ret.linear() = Matrix3(Quaternion(q)); return ret.matrix(); });

    m.def("normalizeVector", [](ref_vec3 v3) { return v3.normalized(); });
    m.def("normalizeVector", [](ref_vec4 v4) { return v4.normalized(); });

    m.def("angleAxisNormalized", [](double angle, ref_vec3 axis) {
          Vector3 ax_ = axis.normalized(); return Matrix3RM(AngleAxis(angle, ax_)); }, R"__IRSL__(
Converting AngleAxis to rotation matrix, axis is normalized

Args:
    angle (float) : rotation angle [radian]
    axis (numpy.array) : 1x3 vector, rotation axis

Returns:
    numpy.array : 3x3, rotation matrix
          )__IRSL__");
    m.def("Position_rotate_with_matrix", [](ref_mat4 cds, ref_mat3 mat, const std::string &wrt) -> Matrix4RM {
            cnoidPosition coords(cds);
            Matrix3 mat_(mat);
            if (wrt == "local") {
                rotate_with_matrix(coords, mat_, coordinates::wrt::local);
            } else if (wrt == "world") {
                rotate_with_matrix(coords, mat_, coordinates::wrt::world);
            }
            return coords.matrix(); },
        py::arg("cds"), py::arg("mat"), py::arg("wrt") = "local");
    m.def("Position_rotate", [](ref_mat4 cds, const double theta, ref_vec3 axis, const std::string &wrt) -> Matrix4RM {
            cnoidPosition coords(cds);
            if (wrt == "local") {
                rotate(coords, theta, axis, coordinates::wrt::local);
            } else if (wrt == "world") {
                rotate(coords, theta, axis, coordinates::wrt::world);
            }
            return coords.matrix(); },
        py::arg("cds"), py::arg("theta"), py::arg("axis"), py::arg("wrt") = "local");
    m.def("Position_transform", [](ref_mat4 cds, ref_mat4 c, const std::string &wrt) -> Matrix4RM {
            cnoidPosition coords(cds);
            cnoidPosition c_(c);
            if (wrt == "local") {
                transform(coords, c_, coordinates::wrt::local);
            } else if (wrt == "world") {
                transform(coords, c_, coordinates::wrt::world);
            }
            return coords.matrix(); },
        py::arg("cds"), py::arg("c"), py::arg("wrt") = "local");
    m.def("Position_transformation", [](ref_mat4 cds, ref_mat4 c, const std::string &wrt) -> Matrix4RM {
            cnoidPosition coords(cds);
            cnoidPosition c_(c);
            cnoidPosition trans_c;
            if (wrt == "local") {
                transformation(coords, trans_c, c_, coordinates::wrt::local);
            } else if (wrt == "world") {
                transformation(coords, trans_c, c_, coordinates::wrt::world);
            }
            return trans_c.matrix(); },
        py::arg("cds"), py::arg("c"), py::arg("wrt") = "local");

    m.def("Position_mid_coords", mid_coords_pos_, py::arg("p"), py::arg("c1"), py::arg("c2"), py::arg("eps") = 0.00001);

    //// for coordinates
    py::class_< coordinates, coordinatesPtr > coords_cls(m, "coordinates");

    py::enum_<coordinates::wrt>(coords_cls, "wrt")
    .value("local", coordinates::wrt::local)
    .value("world", coordinates::wrt::world)
    .value("parent", coordinates::wrt::parent)
    .export_values();

    coords_cls.def(py::init<>())
    .def(py::init([](ref_vec3 trs, ref_mat3 rot)
                  { Vector3 v(trs); Matrix3 m(rot);  return new coordinates(v, m); }))
    .def(py::init([](ref_vec3 trs)
                  { Vector3 v(trs); return new coordinates(v); }))
    .def(py::init([](ref_mat3 rot)
                  { Matrix3 m(rot); return new coordinates(m); }))
    .def(py::init([](ref_vec4 rot)
                  { Quaternion q(rot); return new coordinates(q); }))
    .def(py::init([](ref_vec3 trs, ref_vec3 rot)
                  { Vector3 v(trs); Vector3 rpy(rot); return new coordinates(v, rpy); }))
    .def(py::init([](ref_vec3 trs, ref_vec4 rot)
                  { Quaternion q(rot); return new coordinates(trs, q); }))
    .def(py::init([](ref_mat4 position)
                  { cnoidPosition p(position); return new coordinates(p); }))
    .def("__repr__", [](const coordinates &self) {
            std::stringstream ss;
            ss << "<coordinates[";
            ss << std::hex << reinterpret_cast<const void *>(&self);
            ss << "] ";
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
    .def("__str__", [](const coordinates &self) {
        std::stringstream ss;
        ss << "{'translation': [ ";
        ss << self.pos(0); ss << ", ";
        ss << self.pos(1); ss << ", ";
        ss << self.pos(2); ss << " ], ";
        ss << "'rotation': [ ";
        double an_; Vector3 ax_; self.rotationAngle(an_, ax_);
        ss << ax_(0); ss << ", ";
        ss << ax_(1); ss << ", ";
        ss << ax_(2); ss << ", ";
        ss << an_; ss << " ]}";
        return ss.str();
    })
    .def_property_readonly_static("X", [](py::object /* cls */) { return Vector3::UnitX(); })
    .def_property_readonly_static("Y", [](py::object /* cls */) { return Vector3::UnitY(); })
    .def_property_readonly_static("Z", [](py::object /* cls */) { return Vector3::UnitZ(); })
    .def_static("normalizeVector", [](ref_noconst_vec3 v3) { v3.normalize(); return v3; })
    .def_static("normalizeVector", [](ref_noconst_vec4 v4) { v4.normalize(); return v4; })
    .def_static("normalized", [](ref_vec3 v3) { return v3.normalized(); })
    .def_static("normalized", [](ref_vec4 v4) { return v4.normalized(); })
    .def_static("init2D", [](double x, double y, double theta) {
        Vector3 v; v(0) = x; v(1) = y; v(2) = 0.0;
        AngleAxis a; a.angle() = theta;
        a.axis()(0) = 0.0; a.axis()(1) = 0.0; a.axis()(2) = 1.0;
        return new coordinates(v, a);
    })
    .def_property("pos",
                  [](coordinates &self) { return self.pos; },
                  [](coordinates &self, ref_vec3 vec) { self.pos = vec; }, R"__IRSL__(
Translation part ( real vector with 3 elements, x, y, z ) of transformation matrix

Returns:
    numpy.array : 1x3 vector

                 )__IRSL__")
    .def_property("rot",
                  [](coordinates &self) { return self.rot; },
                  [](coordinates &self, ref_mat3 mat) { self.rot = mat; }, R"__IRSL__(
Rotation part of transformation ( Rotation matrix, 3x3 real matrix)

Returns:
    numpy.array : 3x3 matrix

                 )__IRSL__")
    .def_property("cnoidPosition",
                  [](coordinates &self) -> Matrix4RM { cnoidPosition p; self.toPosition(p); return p.matrix(); },
                  [](coordinates &self, ref_mat4 T) { cnoidPosition p(T); self.newcoords(p); }, R"__IRSL__(
Transformation matrix ( 4x4 homogeneous transformation matrix, using in Choreonoid )

Returns:
    numpy.array : 4x4 matrix

                 )__IRSL__")
    .def_property("angleAxis",
                  [](const coordinates &self) {
                      double an_; Vector3 ax_; self.rotationAngle(an_, ax_);
                      Vector4 ret; ret(0) = ax_(0); ret(1) = ax_(1); ret(2) = ax_(2); ret(3) = an_;
                      return ret; },
                  [](coordinates &self, ref_vec4 ret) {
                      double an_ = ret(3); Vector3 ax_(ret(0), ret(1), ret(2));
                      self.setRotationAngle(an_, ax_);
                      return &self; }, R"__IRSL__(
Rotation part of transformation ( Angle axis, real vector with 4 elements, ax, ay, az, rotation-angle [radian] )

Returns:
    numpy.array : 1x4 vector

                 )__IRSL__")
    .def_property("RPY",
                  [](const coordinates &self) { Vector3 ret; self.getRPY(ret); return ret; },
                  [](coordinates &self, ref_vec3 rpy) { self.setRPY(rpy); }, R"__IRSL__(
Rotation part of transformation ( RPY angle, real vector with 3 elements, roll [radian], pitch [radian], yaw [radian] )

Returns:
    numpy.array : 1x3 vector

                 )__IRSL__")
    .def_property("quaternion",
                  [](const coordinates &self) { Quaternion q(self.rot); return Vector4(q.x(), q.y(), q.z(), q.w()); },
                  [](coordinates &self, ref_vec4 q_in) { Quaternion q(q_in); self.set(q); }, R"__IRSL__(
Rotation part of transformation ( quaternion, real vector with 4 elements, x, y, z, w )

Returns:
    numpy.array : 1x4 vector

                 )__IRSL__")
    .def_property("quaternion_xyzw",
                  [](const coordinates &self) { Quaternion q(self.rot); return Vector4(q.x(), q.y(), q.z(), q.w()); },
                  [](coordinates &self, ref_vec4 q_in) { Quaternion q(q_in); self.set(q); }, R"__IRSL__(
Rotation part of transformation ( quaternion, real vector with 4 elements, x, y, z, w )

Returns:
    numpy.array : 1x4 vector

                 )__IRSL__")
    .def_property("quaternion_wxyz",
                  [](const coordinates &self) { Quaternion q(self.rot); return Vector4(q.w(), q.x(), q.y(), q.z()); },
                  [](coordinates &self, ref_vec4 q_in) { Quaternion q(q_in(0), q_in(1), q_in(2), q_in(3)); self.set(q); }, R"__IRSL__(
Rotation part of transformation ( quaternion, real vector with 4 elements, w, x, y, z )

Returns:
    numpy.array : 1x4 vector

                 )__IRSL__")
    .def("setCoordsToPosition",
         [](coordinates &self, ref_noconst_mat4 position_to_be_set) {
             Isometry3 _T(Isometry3::Identity());
             _T.translation() = self.pos; _T.linear() = self.rot;
             position_to_be_set = _T.matrix(); }, R"__IRSL__(
Set transformation matrix (changing value of argument)

Args:
    position_to_be_set(numpy.array) : 4x4 homogeneous transformation matrix, using in Choreonoid

                 )__IRSL__")
    .def("copy", [](const coordinates &self) { return new coordinates(self.pos, self.rot); }, R"__IRSL__(
Creating new instance with the same value of this instance

Returns:
    cnoid.IRSLCoords.coordinates : copy of this instance (created new instance)

                 )__IRSL__")
    .def("equal", &coordinates::equal, py::arg("cds"), py::arg("eps") = 0.00001)
    .def("toPosition",
         [](const coordinates &self) -> Matrix4RM { cnoidPosition p; self.toPosition(p); return p.matrix(); })
    .def("newcoords", [](coordinates &self, const coordinates &c) { self.newcoords(c); return &self; })
    .def("rotate_with_matrix",
         [](coordinates &self, ref_mat3 mat, coordinates::wrt wrt)
         {  self.rotate_with_matrix(mat, wrt); return &self; },
         py::arg("mat"), py::arg("wrt") = coordinates::wrt::local )
    .def("rotate_with_matrix",
         [](coordinates &self, ref_mat3 mat, const coordinates &wrt)
         {  self.rotate_with_matrix(mat, wrt); return &self; } )
    .def("rotate_with_matrix",
         [](coordinates &self, ref_mat3 mat, ref_mat3 wrt)
         {  self.rotate_with_matrix(mat, wrt); return &self; } )
    .def("rotate",
         [](coordinates &self, double th_, ref_vec3 ax_, coordinates::wrt wrt)
         { self.rotate(th_, ax_, wrt); return &self; },
         py::arg("theta"), py::arg("axis"), py::arg("wrt") = coordinates::wrt::local )
    .def("rotate",
         [](coordinates &self, double th_, ref_vec3 ax_, const coordinates &wrt)
         { self.rotate(th_, ax_, wrt); return &self; } )
    .def("rotate",
         [](coordinates &self, double th_, ref_vec3 ax_, ref_mat3 wrt)
         { self.rotate(th_, ax_, wrt); return &self; } )
    .def("orient_with_matrix",
         [](coordinates &self, ref_mat3 mat, coordinates::wrt wrt)
         {  self.orient_with_matrix(mat, wrt); return &self; },
         py::arg("mat"), py::arg("wrt") = coordinates::wrt::local )
    .def("orient_with_matrix",
         [](coordinates &self, ref_mat3 mat, const coordinates &wrt)
         {  self.orient_with_matrix(mat, wrt); return &self; } )
    .def("orient_with_matrix",
         [](coordinates &self, ref_mat3 mat, ref_mat3 wrt)
         {  self.orient_with_matrix(mat, wrt); return &self; } )
    .def("orient",
         [](coordinates &self, double th_, ref_vec3 ax_, coordinates::wrt wrt)
         { self.orient(th_, ax_, wrt); return &self; },
         py::arg("theta"), py::arg("axis"), py::arg("wrt") = coordinates::wrt::local )
    .def("orient",
         [](coordinates &self, double th_, ref_vec3 ax_, const coordinates &wrt)
         { self.orient(th_, ax_, wrt); return &self; } )
    .def("orient",
         [](coordinates &self, double th_, ref_vec3 ax_, ref_mat3 wrt)
         { self.orient(th_, ax_, wrt); return &self; } )
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
             coordinatesPtr ret(new coordinates()); self.inverse_transformation(*ret); return ret;
         } )
    .def("transformation",
         [](const coordinates &self, const coordinates &c, coordinates::wrt wrt) {
             coordinatesPtr ret(new coordinates()); self.transformation(*ret, c, wrt); return ret;
         }, py::arg("coords"), py::arg("wrt") = coordinates::wrt::local )
    .def("transformation",
         [](const coordinates &self, const coordinates &c, const coordinates &wrt) {
             coordinatesPtr ret(new coordinates()); self.transformation(*ret, c, wrt); return ret;
         } )
    .def("transform",
         [](coordinates &self, const coordinates &c, coordinates::wrt wrt)
         { self.transform(c, wrt); return &self; }, py::arg("coords"), py::arg("wrt") = coordinates::wrt::local )
    .def("transform",
         [](coordinates &self, const coordinates &c, const coordinates &wrt)
         { self.transform(c, wrt); return &self; } )
    .def("get_transformed",
         [](coordinates &self, const coordinates &c, coordinates::wrt wrt)
         { coordinates res; self.get_transformed(res, c, wrt); return res; }, py::arg("coords"), py::arg("wrt") = coordinates::wrt::local )
    .def("get_transformed",
         [](coordinates &self, const coordinates &c, const coordinates &wrt)
         { coordinates res; self.get_transformed(res, c, wrt); return res; } )
    //// in place version (modify input vec)
    .def("rotateVector",
         [](const coordinates &self, ref_noconst_vec3 vec) { Vector3 v(vec);
             self.rotate_vector(v); vec = v; return vec; })
    .def("inverseRotateVector",
         [](const coordinates &self, ref_noconst_vec3 vec) { Vector3 v(vec);
             self.inverse_rotate_vector(v); vec = v; return vec; })
    .def("transformVector",
         [](const coordinates &self, ref_noconst_vec3 vec) { Vector3 v(vec);
             self.transform_vector(v); vec = v; return vec; })
    .def("inverseTransformVector",
         [](const coordinates &self, ref_noconst_vec3 vec) { Vector3 v(vec);
             self.inverse_transform_vector(v); vec = v; return vec; })
    ////
    .def("rotate_vector",
         [](const coordinates &self, ref_vec3 vec) { Vector3 v(vec);
             self.rotate_vector(v); return v; })
    .def("inverse_rotate_vector",
         [](const coordinates &self, ref_vec3 vec) { Vector3 v(vec);
             self.inverse_rotate_vector(v); return v; })
    .def("transform_vector",
         [](const coordinates &self, ref_vec3 vec) { Vector3 v(vec);
             self.transform_vector(v); return v; })
    .def("inverse_transform_vector",
         [](const coordinates &self, ref_vec3 vec) { Vector3 v(vec);
             self.inverse_transform_vector(v); return v; })
    ///
    .def("translate",
         [](coordinates &self, ref_vec3 vec, coordinates::wrt wrt) {
             self.translate(vec, wrt); return &self; }, py::arg("vector"), py::arg("wrt") = coordinates::wrt::local )
    .def("translate",
         [](coordinates &self, ref_vec3 vec, const coordinates &wrt) {
             self.translate(vec, wrt); return &self; } )
    .def("locate",
         [](coordinates &self, ref_vec3 vec, coordinates::wrt wrt) {
             self.locate(vec, wrt); return &self; }, py::arg("vector"), py::arg("wrt") = coordinates::wrt::local )
    .def("locate",
         [](coordinates &self, ref_vec3 vec, const coordinates &wrt) {
             self.locate(vec, wrt); return &self; } )
    .def("move_to",
         [](coordinates &self, const coordinates &c, coordinates::wrt wrt) {
             self.move_to(c, wrt); return &self; }, py::arg("coords"), py::arg("wrt") = coordinates::wrt::local )
    .def("move_to",
         [](coordinates &self, const coordinates &c, const coordinates &wrt) {
             self.move_to(c, wrt); return &self; } )
    //
    .def("rotNormalize", &coordinates::rotNormalize)
    .def("getRotationAngle", [](const coordinates &self) {
            double an_; Vector3 ax_; self.rotationAngle(an_, ax_);
            Vector4 ret; ret(0) = ax_(0); ret(1) = ax_(1); ret(2) = ax_(2); ret(3) = an_;
            return ret; } )
    .def("setRotationAngle", [](coordinates &self, ref_vec4 ret) {
            double an_ = ret(3); Vector3 ax_(ret(0), ret(1), ret(2));
            self.setRotationAngle(an_, ax_);
            return &self; } )
    .def_property_readonly("x_axis", [](const coordinates &self) { Vector3 ret; self.x_axis(ret); return ret; }, R"__IRSL__(
Extracting x-axis of rotation matrix (1st column vector)

Returns:
    numpy.array : 1x3 vector, x-axis of rotation matrix (1st column vector)
                           )__IRSL__")
    .def_property_readonly("y_axis", [](const coordinates &self) { Vector3 ret; self.y_axis(ret); return ret; }, R"__IRSL__(
Extracting y-axis of rotation matrix (2nd column vector)

Returns:
    numpy.array : 1x3 vector, y-axis of rotation matrix (2nd column vector)
                           )__IRSL__")
    .def_property_readonly("z_axis", [](const coordinates &self) { Vector3 ret; self.z_axis(ret); return ret; }, R"__IRSL__(
Extracting z-axis of rotation matrix (3rd column vector)

Returns:
    numpy.array : 1x3 vector, z-axis of rotation matrix (3rd column vector)
                           )__IRSL__")
    .def("getRPY", [](const coordinates &self) { Vector3 ret; self.getRPY(ret); return ret; } )
    .def("setRPY", [](coordinates &self, ref_vec3 rpy) { self.setRPY(rpy); } )
    .def("setRPY", [](coordinates &self, double r, double p, double y) { self.setRPY(r,p,y); } )
    .def("inverse", [](coordinates &self) { self.inverse(); return &self; }, R"__IRSL__(
Updating self transformation as an inverse transformation

Returns:
    cnoid.IRSLCoords.coordinates : identical instance which was called with this method
         )__IRSL__")
    .def("mid_coords", [](const coordinates &self, const double p, const coordinates &c2, const double eps) {
            coordinatesPtr ret(new coordinates());
            self.mid_coords(*ret, p, c2, eps);
            return ret; }, py::arg("p"), py::arg("c2"), py::arg("eps") = 0.00001, R"__IRSL__(
Calculating interpolated coordinates

Args:
    p (float) : parameter 0.0 to 1.0, if p == 0.0, coords euqal to self is return. If p == 1.0, coords equal to c2 is return.
    c2 (cnoid.IRSLCoords.coordinates) : target coordinates
    eps (float, default = 0.00001) : precision

Returns:
    cnoid.IRSLCoords.coordinates : Interpolated coordinates
         )__IRSL__")
    ;

    // add method to ..
    m.def("angleVector",
          [](Body &self) {
              int n = self.numJoints();
              VectorX vec(n);
              for(int i = 0; i < n; i++) {
                  vec(i) = self.joint(i)->q();
              }
              return vec;
          });
    m.def("angleVector",
          [](Body &self, VectorX &vec) {
              int n = vec.size();
              if (self.numJoints() < n) n = self.numJoints();
              for(int i = 0; i < n; i++) {
                  self.joint(i)->q() = vec(i);
              }
          });
    m.def("getCoords",
          [](Link &self) {
              coordinatesPtr ret(new coordinates());
              *ret = self.position();
              return ret;
          });
    m.def("setCoords",
          [](Link &self, const coordinates &cds) {
              cds.toPosition(self.position());
          });
    m.def("getOffsetCoords",
          [](Link &self) {
              coordinatesPtr ret(new coordinates());
              *ret = self.offsetPosition();
              return ret;
          });
    m.def("setOffsetCoords",
          [](Link &self, const coordinates &cds) {
              cnoidPosition p;
              cds.toPosition(p);
              self.setOffsetPosition(p);
          });
}
