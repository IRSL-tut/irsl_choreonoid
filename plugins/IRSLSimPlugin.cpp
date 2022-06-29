#include "IRSLSimPlugin.h"
#include "SimStepControllerItem.h"

#include <fmt/format.h>

using namespace cnoid;

namespace {

IRSLSimPlugin* instance_ = nullptr;

}


IRSLSimPlugin* IRSLSimPlugin::instance()
{
    return instance_;
}


IRSLSimPlugin::IRSLSimPlugin()
    : Plugin("IRSLSim")
{
    setActivationPriority(0);
    instance_ = this;
}


bool IRSLSimPlugin::initialize()
{
    // classes
    SimStepControllerItem::initializeClass(this);

    return true;
}


bool IRSLSimPlugin::finalize()
{
    instance_ = nullptr;
    return true;
}


const char* IRSLSimPlugin::description() const
{
    static std::string text =
        fmt::format("IRSLSim Plugin Version {}\n", CNOID_FULL_VERSION_STRING) +
        "\n" +
        "Copyrigh (c) 2022 IRSL-tut Development Team.\n"
        "\n" +
        MITLicenseText() +
        "\n"  ;

    return text.c_str();
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(IRSLSimPlugin);
