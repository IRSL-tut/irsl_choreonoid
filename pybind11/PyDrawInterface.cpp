/**
   @author Kunio Kojima
*/
// copied from jsk_choreonoid ( https://github.com/kindsenior/jsk_choreonoid )
#include <cnoid/PyUtil>
#include "DrawInterface.h"

#include <pybind11/pybind11.h>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(DrawInterface, m)
{
    m.doc() = "Interface for drawing";

    py::module::import("cnoid.Base");
    py::module::import("cnoid.Util");

    py::class_< DrawInterface, DrawInterfacePtr, Referenced >(m, "DrawInterface")
        .def(py::init<Eigen::Vector3f>())
        .def(py::init<>())
        .def("setLineWidth", &DrawInterface::setLineWidth)
        .def("setColor", &DrawInterface::setColor)
        .def("show", &DrawInterface::show)
        .def("hide", &DrawInterface::hide)
        .def("drawLine", &DrawInterface::drawLine)
        .def("drawArc", &DrawInterface::drawArc)
        .def("drawArrow", &DrawInterface::drawArrow)
        .def("drawArcArrow", &DrawInterface::drawArcArrow)
        .def("drawLineArcArrow", &DrawInterface::drawLineArcArrow)
        .def("drawArrowTip", &DrawInterface::drawArrowTip)
        //
        .def("addColor", &DrawInterface::addColor)
        .def("addDrawLine", &DrawInterface::addDrawLine)
        .def("drawAxis",
             [] (DrawInterface &self, coordinates &_cds, int _axis, double _length) {
                 self.drawAxis(_cds, _axis, _length); })
        .def("addAxis",
             [] (DrawInterface &self, coordinates &_cds, int _axis, double _length, int _color) {
                 self.addAxis(_cds, _axis, _length, _color); })
        .def("addAxis3",
             [] (DrawInterface &self, coordinates &_cds, double _length, int _x_color, int _y_color, int _z_color) {
                 self.addAxis3(_cds, _length, _x_color, _y_color, _z_color); })
        .def("addBDAxis",
             [] (DrawInterface &self, coordinates &_cds, int _axis, double _length, int _color) {
                 self.addBDAxis(_cds, _axis, _length, _color); })
        .def("addBDAxis3",
             [] (DrawInterface &self, coordinates &_cds, double _length, int _x_color, int _y_color, int _z_color) {
                 self.addBDAxis3(_cds, _length, _x_color, _y_color, _z_color); })
        //
        .def("hide_and_show", &DrawInterface::hide_and_show)
        .def("add_object", (void (DrawInterface::*)(SgNodePtr &, bool)) &DrawInterface::add_object,
             py::arg("object"), py::arg("update") = false)
        .def("add_object", (void (DrawInterface::*)(SgGroupPtr &, bool)) &DrawInterface::add_object,
             py::arg("object"), py::arg("update") = false)
        .def("add_object", (void (DrawInterface::*)(SgTransformPtr &, bool)) &DrawInterface::add_object,
             py::arg("object"), py::arg("update") = false)
        .def("add_object", (void (DrawInterface::*)(SgPosTransformPtr &, bool)) &DrawInterface::add_object,
             py::arg("object"), py::arg("update") = false)
        .def("add_object", (void (DrawInterface::*)(SgShapePtr &, bool)) &DrawInterface::add_object,
             py::arg("object"), py::arg("update") = false)
        //
        .def("remove_object", (void (DrawInterface::*)(SgNodePtr &, bool)) &DrawInterface::remove_object,
             py::arg("object"), py::arg("update") = false)
        .def("remove_object", (void (DrawInterface::*)(SgGroupPtr &, bool)) &DrawInterface::remove_object,
             py::arg("object"), py::arg("update") = false)
        .def("remove_object", (void (DrawInterface::*)(SgTransformPtr &, bool)) &DrawInterface::remove_object,
             py::arg("object"), py::arg("update") = false)
        .def("remove_object", (void (DrawInterface::*)(SgPosTransformPtr &, bool)) &DrawInterface::remove_object,
             py::arg("object"), py::arg("update") = false)
        .def("remove_object", (void (DrawInterface::*)(SgShapePtr &, bool)) &DrawInterface::remove_object,
             py::arg("object"), py::arg("update") = false)
        //
        .def("render", &DrawInterface::render)
        ;
    //SgPlot
    //SgPointSet
    //SgLineSet
    //SgOverlay

    m.def("flush", &DrawInterface::flush);
}
