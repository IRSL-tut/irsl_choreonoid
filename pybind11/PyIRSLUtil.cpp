/**
   @author YoheiKakiuchi
*/

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/iostream.h> // add_ostream_redirect
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

#include <vector>
#include <algorithm>

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

    // bind C++ output-stream
    py::add_ostream_redirect(m, "ostream_redirect");
}
