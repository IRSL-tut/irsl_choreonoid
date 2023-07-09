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
        .def("setOrigin", &DrawInterface::setOrigin)
        .def("getOrigin", [] (GeneralDrawInterface &self) {
            coordinates _res;
            self.getOrigin(_res);
            return _res; })
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
        //
        .def("render", &DrawInterface::render)//should be class method
        .def("flush", [](DrawInterface &self) { DrawInterface::flush(); })//should be class method
        .def("viewAll", &DrawInterface::viewAll)// Can we implement viewThis?
        ;
    //SgPlot
    //SgPointSet
    //SgLineSet
    //SgOverlay
    py::class_< GeneralDrawInterface, GeneralDrawInterfacePtr, DrawInterface >(m, "GeneralDrawInterface")
        .def(py::init<>())
        .def("setOrigin", &DrawInterface::setOrigin)
        .def("getOrigin", [] (GeneralDrawInterface &self) {
            coordinates _res;
            self.getOrigin(_res);
            return _res; })
        //
        .def("addObject", (void (GeneralDrawInterface::*)(SgNodePtr &, bool)) &GeneralDrawInterface::add_object,
             py::arg("object"), py::arg("update") = false)
        .def("addObject", (void (GeneralDrawInterface::*)(SgGroupPtr &, bool)) &GeneralDrawInterface::add_object,
             py::arg("object"), py::arg("update") = false)
        .def("addObject", (void (GeneralDrawInterface::*)(SgTransformPtr &, bool)) &GeneralDrawInterface::add_object,
             py::arg("object"), py::arg("update") = false)
        .def("addObject", (void (GeneralDrawInterface::*)(SgPosTransformPtr &, bool)) &GeneralDrawInterface::add_object,
             py::arg("object"), py::arg("update") = false)
        .def("addObject", (void (GeneralDrawInterface::*)(SgShapePtr &, bool)) &GeneralDrawInterface::add_object,
             py::arg("object"), py::arg("update") = false)
        //
        .def("removeObject", (void (GeneralDrawInterface::*)(SgNodePtr &, bool)) &GeneralDrawInterface::remove_object,
             py::arg("object"), py::arg("update") = false)
        .def("removeObject", (void (GeneralDrawInterface::*)(SgGroupPtr &, bool)) &GeneralDrawInterface::remove_object,
             py::arg("object"), py::arg("update") = false)
        .def("removeObject", (void (GeneralDrawInterface::*)(SgTransformPtr &, bool)) &GeneralDrawInterface::remove_object,
             py::arg("object"), py::arg("update") = false)
        .def("removeObject", (void (GeneralDrawInterface::*)(SgPosTransformPtr &, bool)) &GeneralDrawInterface::remove_object,
             py::arg("object"), py::arg("update") = false)
        .def("removeObject", (void (GeneralDrawInterface::*)(SgShapePtr &, bool)) &GeneralDrawInterface::remove_object,
             py::arg("object"), py::arg("update") = false)
        //
        .def("render", &DrawInterface::render)//should be class method
        .def("flush", [](GeneralDrawInterface &self) { DrawInterface::flush(); })//should be class method
        .def("viewAll", &DrawInterface::viewAll)// Can we implement viewThis?
        ;

    m.def("flush", &DrawInterface::flush); //Deprecated??
}
