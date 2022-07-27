#include <cnoid/OptionManager> //??

#include "IRSLXXXPlugin.h"
#include "XXXBodyItem.h"
#include "XXXView.h"
#include "XXXBar.h"

#include <fmt/format.h>

#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;

namespace po = boost::program_options;

namespace {
IRSLXXXPlugin* instance_ = nullptr;
}

static void onSigOptionsParsed(po::variables_map& variables)
{
    if(variables.count("assembler")) {
        std::string fname_ = variables["assembler"].as<std::string>();
        DEBUG_STREAM("robot_assembler config file: " << fname_ << std::endl);

        XXXView *ptr = XXXView::instance();
        ptr->createButtons();
    }
}

IRSLXXXPlugin* IRSLXXXPlugin::instance()
{
    return instance_;
}

IRSLXXXPlugin::IRSLXXXPlugin()
    : Plugin("IRSLXXX")
{
    DEBUG_STREAM_INFO(IRSLXXXPlugin,IRSLXXXPlugin, std::endl);
    setActivationPriority(0);
    instance_ = this;
}

bool IRSLXXXPlugin::initialize()
{
    DEBUG_STREAM_INFO(IRSLXXXPlugin,initialize(), std::endl);
    OptionManager& om = this->optionManager();
    om.addOption("assembler", po::value<std::string>(), "load robot_assembler config file");
    om.sigOptionsParsed(1).connect(onSigOptionsParsed);

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
