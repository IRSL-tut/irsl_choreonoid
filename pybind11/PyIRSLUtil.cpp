/**
   @author YoheiKakiuchi
*/

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/iostream.h> // add_ostream_redirect
#include <pybind11/numpy.h>
//
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>
#include <cnoid/ViewManager>

//
#include <cnoid/SceneGraph>
#include <cnoid/SceneDrawables>

//
#include <QCoreApplication>

//
#include <QMutex>
#include <QThread>

#include <vector>
#include <algorithm>
//#include <iostream>

using namespace cnoid;
namespace py = pybind11;

void get_viewtitles(std::vector<std::string> &res, bool mounted)
{
    auto viewClasses = ViewManager::viewClasses();
    for(auto& viewClass : viewClasses) {
        auto viewInstances = viewClass->instances();
        for(auto& view : viewInstances) {
            if (mounted && !(view->isMounted())) continue;
            res.push_back(view->windowTitle().toStdString());
            //view->isMounted();
            //view->mountOnMainWindow(true);
            //view->unmount();
        }
    }
}
void unmount_viewtitles(std::vector<std::string> &_in)
{
    auto viewClasses = ViewManager::viewClasses();
    for(auto& viewClass : viewClasses) {
        auto viewInstances = viewClass->instances();
        for(auto& view : viewInstances) {
            auto res = std::find(_in.begin(), _in.end(), view->windowTitle().toStdString());
            if (res != _in.end()) {
                view->unmount();
            }
        }
    }
}

void fixSgPosTransform(SgPosTransform &_trs)
{
    Matrix4 T = _trs.T().matrix();
    T(0, 2) = -T(0, 2);
    T(1, 2) = -T(1, 2);
    T(2, 2) = -T(2, 2);
    Isometry3 newT(T);
    _trs.setPosition(newT);
}

void fixMesh(SgMesh &_mesh)
{
    if(_mesh.hasNormals()) {
        SgNormalArray *nm = _mesh.normals();
        for(auto it = nm->begin(); it != nm->end(); it++) {
            (*it)(2) = - (*it)(2);
        }
    }
    if(_mesh.hasVertices()) {
        SgVertexArray *vt = _mesh.vertices();
        for(auto it = vt->begin(); it != vt->end(); it++) {
            (*it)(2) = - (*it)(2);
        }
    }
}

class MyMutex
{
public:
    QMutex mutex;

    void lock() {  mutex.lock(); }
    void unlock() {  mutex.unlock(); }
    bool try_lock(int wait) {
        if (wait >= 0) {
            return mutex.tryLock(wait);
        } else {
            return mutex.tryLock();
        }
    }
};

void setTextureImage(SgShape *shape, const py::array_t<unsigned char> &img, const std::string &name)
{
    SgTexture *tex = new SgTexture();
    SgImage *sgimg = tex->getOrCreateImage();
    sgimg->setUri(name, name);
    int dim    = img.ndim();
    int height = img.shape(0);
    int width  = img.shape(1);
    int comp   = 1;
    if (dim > 2) {
        comp   = img.shape(2);
    }
    //std::cout << "dim: " << dim << std::endl;
    //std::cout << "height: " << height << std::endl;
    //std::cout << "width: " << width << std::endl;
    //std::cout << "comp: " << comp << std::endl;
    sgimg->setSize(width, height, comp);
    const unsigned char *src = (const unsigned char *)(img.data(0));
    long len = width*height*comp - 1;
    for (long i = 0; i < width*height*comp; i++) {
        sgimg->pixels()[i] = src[i];
    }
    sgimg->image().applyVerticalFlip();
    shape->setTexture(tex);
}
#if 0
void debugTextureImage(SgShape *shape)
{
    SgTexture *tex = shape->getOrCreateTexture();
    SgImage *sgimg = tex->getOrCreateImage();
    int height = sgimg->height();
    int width = sgimg->width();
    int comp = sgimg->numComponents();
    std::cout << "height: " << height << std::endl;
    std::cout << "width: " << width << std::endl;
    std::cout << "comp: " << comp << std::endl;
    long cntr = 0;
    for (long i = 0; i < height; i++) {
        for (long j = 0; j < width; j++) {
            for (long k = 0; k < comp; k++) {
                std::cout << (int)sgimg->pixels()[cntr++] << " ";
            }
            std::cout << " ";
        }
        std::cout << std::endl;
    }
}
#endif
PYBIND11_MODULE(IRSLUtil, m)
{
    m.doc() = "utilitiy for choreonoid (IRSL)";

    py::module::import("cnoid.Base");
    // py::module::import("cnoid.Util");

    //
    m.def("getViewTitles", [] (bool mounted) {
        std::vector<std::string> res;
        get_viewtitles(res, mounted);
        return res;
    });
    m.def("unmountViewTitles", &unmount_viewtitles);

    // fix for left-hand coordinates
    m.def("fixSgPosTransform", &fixSgPosTransform);
    m.def("fixMesh", &fixMesh);

    m.def("processEvent", [](){
        QCoreApplication::processEvents(QEventLoop::AllEvents);
    });
    m.def("usleep", [](int usec){
        QThread::usleep(usec);
    });

    py::class_< MyMutex >(m, "Mutex")
    .def(py::init<>())
    .def("lock", &MyMutex::lock)
    .def("unlock", &MyMutex::unlock)
    .def("try_lock", &MyMutex::try_lock)
    .def("__enter__", [] (MyMutex &self) {
        self.lock();
        //return self;
    })
    .def("__exit__", [] (MyMutex &self, py::object &exc_type, py::object &exc_value, py::object &traceback) {
        self.unlock();
    })
    ;

    m.def("setTextureImage", &setTextureImage);
    //m.def("debugTextureImage", &debugTextureImage);

    // bind C++ output-stream
    py::add_ostream_redirect(m, "ostream_redirect");
}
