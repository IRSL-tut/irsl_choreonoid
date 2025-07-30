/**
   @author Kunio Kojima
*/
// copied from jsk_choreonoid ( https://github.com/kindsenior/jsk_choreonoid )
#include <cnoid/PyUtil>
#include "DrawInterface.h"
#include <cstring> // memcpy
#include <pybind11/pybind11.h>

using namespace cnoid;
namespace py = pybind11;

typedef unsigned char uchar;

py::array QImageToNumpy(const QImage &in)
{
    py::array_t<uchar> res;
    QImage::Format fm = in.format();
#if 0
    printf("format = %d, width = %d, height = %d, bytes = %d, p_line = %d\n",
           fm, in.width(), in.height(), in.sizeInBytes(), in.bytesPerLine());
#endif
    int pixelSize = 4;

    // set Format
    //QImage::Format_RGB888
    //QImage::Format_BGR888
    //QImage::Format_Grayscale8
    //QImage::Format_Grayscale16
    pixelSize = 3;
    QImage tmp = in.convertToFormat(QImage::Format_RGB888);
    //QImage tmp = in.convertToFormat(QImage::Format_RGB888).rgbSwapped();
#if 0
    printf("format = %d, width = %d, height = %d, bytes = %d, p_line = %d\n",
           tmp.format(), tmp.width(), tmp.height(), tmp.sizeInBytes(), tmp.bytesPerLine());
#endif
    res.resize({tmp.height(), tmp.width(), pixelSize});

    py::buffer_info b = res.request();
    uchar *ptr = static_cast<uchar *>(b.ptr);
    //
    if ( tmp.width() * pixelSize == tmp.bytesPerLine() ) {
        std::memcpy(ptr, tmp.bits(), tmp.sizeInBytes());
    } else {
        uchar *src = tmp.bits();
        for(int i = 0; i < tmp.height(); i++) {
            std::memcpy(ptr, src, tmp.width()*pixelSize);
            src += tmp.bytesPerLine();
            ptr += tmp.width()*pixelSize;
        }
    }
    return res;
}

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
        .def_property("T",
                      [](DrawInterface &self) -> Isometry3::MatrixType& { return self.T().matrix(); },
                      [](DrawInterface &self, Eigen::Ref<const Matrix4RM> T){ self.setPosition(T); })
        .def_property_readonly("SgPosTransform",
                               [](DrawInterface &self) { return self.posTrans; }, R"__IRSL__(
Generating instance of cnoid.Util.SgPosTransform, which represents the root position of this instance

Returns:
    cnoid.Util.SgPosTransform : Root coordinates of drawn objects in this instance at SceneGraph
          )__IRSL__")
        .def("hide_and_show", &DrawInterface::hide_and_show)
        //
        .def("render", &DrawInterface::render, py::arg("doImmediately") = false)//should be class method
        .def("flush", &DrawInterface::flush)
        .def("viewAll", &DrawInterface::viewAll)// Can we implement viewThis?
        ;
    //SgPlot
    //SgPointSet
    //SgLineSet
    //SgOverlay
    py::class_< GeneralDrawInterface, GeneralDrawInterfacePtr, DrawInterface >(m, "GeneralDrawInterface")
        .def(py::init<>())
        .def(py::init<bool>())
        .def("flush", &GeneralDrawInterface::flush)
        .def("cpp_clear", &GeneralDrawInterface::cpp_clear)
        .def_property_readonly("objects", [] (GeneralDrawInterface &self) {
            py::list _lst;
            for(auto it = self.posTrans->begin(); it != self.posTrans->end(); it++) {
                _lst.append( py::cast(*it) );
            }
            return _lst; }, R"__IRSL__(
Getting list of drawn objects in this instance

Returns:
    list [ cnoid.Util.SgNode ] : List of drawn objects in this instance
          )__IRSL__")
        .def("addObject", (void (GeneralDrawInterface::*)(SgNodePtr &, bool)) &GeneralDrawInterface::add_object,
             py::arg("obj"), py::arg("update") = false)
        .def("addObject", (void (GeneralDrawInterface::*)(SgGroupPtr &, bool)) &GeneralDrawInterface::add_object,
             py::arg("obj"), py::arg("update") = false)
        .def("addObject", (void (GeneralDrawInterface::*)(SgTransformPtr &, bool)) &GeneralDrawInterface::add_object,
             py::arg("obj"), py::arg("update") = false)
        .def("addObject", (void (GeneralDrawInterface::*)(SgPosTransformPtr &, bool)) &GeneralDrawInterface::add_object,
             py::arg("obj"), py::arg("update") = false)
        .def("addObject", (void (GeneralDrawInterface::*)(SgShapePtr &, bool)) &GeneralDrawInterface::add_object,
             py::arg("obj"), py::arg("update") = false)
        .def("addPyObject", [] (GeneralDrawInterface &self, py::object _o, bool _update) {
            SgNode *nd = _o.cast<SgNode *>();
            if (!!nd) {  SgNodePtr ptr(nd); self.add_object(ptr, _update); return true; }
            return false; }, py::arg("obj"), py::arg("update") = false)
        //
        .def("removeObject", (void (GeneralDrawInterface::*)(SgNodePtr &, bool)) &GeneralDrawInterface::remove_object,
             py::arg("obj"), py::arg("update") = false)
        .def("removeObject", (void (GeneralDrawInterface::*)(SgGroupPtr &, bool)) &GeneralDrawInterface::remove_object,
             py::arg("obj"), py::arg("update") = false)
        .def("removeObject", (void (GeneralDrawInterface::*)(SgTransformPtr &, bool)) &GeneralDrawInterface::remove_object,
             py::arg("obj"), py::arg("update") = false)
        .def("removeObject", (void (GeneralDrawInterface::*)(SgPosTransformPtr &, bool)) &GeneralDrawInterface::remove_object,
             py::arg("obj"), py::arg("update") = false)
        .def("removeObject", (void (GeneralDrawInterface::*)(SgShapePtr &, bool)) &GeneralDrawInterface::remove_object,
             py::arg("obj"), py::arg("update") = false)
        .def("removePyObject", [] (GeneralDrawInterface &self, py::object _o, bool _update) {
            SgNode *nd = _o.cast<SgNode *>();
            if (!!nd) {  SgNodePtr ptr(nd); self.remove_object(ptr, _update); return true; }
            return false; }, py::arg("obj"), py::arg("update") = false)
        .def("getImage", [] (GeneralDrawInterface &self) {
            SceneWidget *sw = self.sceneWidget();
            if (!!sw) {
                return QImageToNumpy(sw->getImage());
            }
        })
        ;

    m.def("flush", &DrawInterface::flushAll); //Deprecated??
}
