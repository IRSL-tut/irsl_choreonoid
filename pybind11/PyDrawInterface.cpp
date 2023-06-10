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
        .def("drawAxis",
             [] (DrawInterface &self, coordinates &_cds, int _axis, double _length) {
                 self.drawAxis(_cds, _axis, _length); })
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
