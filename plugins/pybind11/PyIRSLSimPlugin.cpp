/**
   @author YoheiKakiuchi
*/

#include <sstream>
#include <pybind11/pybind11.h>

#include "../SimStepControllerItem.h"

#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(IRSLSimPlugin, m)
{
    m.doc() = "IRSLSimPlugin";

    py::module::import("cnoid.Base");
    py::module::import("cnoid.Body");
    py::module::import("cnoid.BodyPlugin");

    py::class_< SimStepControllerItem, SimStepControllerItemPtr, ControllerItem > irsl_cont(m, "SimStepControllerItem");

    irsl_cont.def(py::init<>())
    .def(py::init<const SimStepControllerItem &>())
    .def("check_sim_step", &SimStepControllerItem::check_sim_step)
    .def("wait_next_step", &SimStepControllerItem::wait_next_step)
    .def("notify_sim_step", &SimStepControllerItem::notify_sim_step)
    .def("disable_stepping", &SimStepControllerItem::disable_stepping)
    .def("enable_stepping", &SimStepControllerItem::enable_stepping)
    .def("timeStep", &SimStepControllerItem::timeStep)
    .def("currentTime", &SimStepControllerItem::currentTime)
    .def("getSimBody", &SimStepControllerItem::getSimBody)
    .def("wait_steps", &SimStepControllerItem::wait_steps)
    .def("usleep", &SimStepControllerItem::usleep)
    .def("stop_on_output", &SimStepControllerItem::stop_on_output)
    .def("at_output", &SimStepControllerItem::at_output)
    ;
}
