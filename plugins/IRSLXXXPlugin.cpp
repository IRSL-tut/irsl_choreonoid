#include "IRSLXXXPlugin.h"
#include "XXXBodyItem.h"
#include "XXXView.h"
#include "XXXBar.h"

#include <fmt/format.h>

using namespace cnoid;

namespace {
IRSLXXXPlugin* instance_ = nullptr;
}

IRSLXXXPlugin* IRSLXXXPlugin::instance()
{
    return instance_;
}

IRSLXXXPlugin::IRSLXXXPlugin()
    : Plugin("IRSLXXX")
{
    setActivationPriority(0);
    instance_ = this;
}

bool IRSLXXXPlugin::initialize()
{
    // classes
    XXXBodyItem::initializeClass(this);

    //View
    XXXView::initializeClass(this);

    //ToolBar
    addToolBar(XXXBar::instance());

    return true;
}

bool IRSLXXXPlugin::finalize()
{
    instance_ = nullptr;
    return true;
}

const char* IRSLXXXPlugin::description() const
{
    static std::string text =
        fmt::format("IRSLXXX Plugin Version {}\n", CNOID_FULL_VERSION_STRING) +
        "\n" +
        "Copyrigh (c) 2022 IRSL-tut Development Team.\n"
        "\n" +
        MITLicenseText() +
        "\n"  ;

    return text.c_str();
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(IRSLXXXPlugin);
