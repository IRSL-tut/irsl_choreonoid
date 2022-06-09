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
        .def("draw", &DrawInterface::draw)
        .def("add_object", &DrawInterface::add_object)
        .def("remove_object", &DrawInterface::remove_object)
        ;
}
