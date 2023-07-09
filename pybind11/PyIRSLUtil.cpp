/**
   @author YoheiKakiuchi
*/

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
//
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>
#include <cnoid/ViewManager>

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

PYBIND11_MODULE(IRSLUtil, m)
{
    m.doc() = "utilitiy for choreonoid (IRSL)";

    py::module::import("cnoid.Base");
    // py::module::import("cnoid.Util");

    m.def("getViewTitles", [] (bool mounted) {
        std::vector<std::string> res;
        get_viewtitles(res, mounted);
        return res;
    });

    m.def("unmountViewTitles", &unmount_viewtitles);
}
