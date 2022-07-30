#include <cnoid/OptionManager> //??

#include "RobotAssemblerPlugin.h"
#include "AssemblerBodyItem.h"
#include "AssemblerView.h"
#include "AssemblerBar.h"

#include <fmt/format.h>

#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;

namespace po = boost::program_options;

namespace {
RobotAssemblerPlugin* instance_ = nullptr;
}

static void onSigOptionsParsed(po::variables_map& variables)
{
    if(variables.count("assembler")) {
        std::string fname_ = variables["assembler"].as<std::string>();
        DEBUG_STREAM("robot_assembler config file: " << fname_ << std::endl);

        AssemblerView *ptr = AssemblerView::instance();
        ptr->createButtons();
    }
}

RobotAssemblerPlugin* RobotAssemblerPlugin::instance()
{
    return instance_;
}

RobotAssemblerPlugin::RobotAssemblerPlugin()
    : Plugin("RobotAssembler")
{
    DEBUG_STREAM_INFO(RobotAssemblerPlugin,RobotAssemblerPlugin, std::endl);
    setActivationPriority(0);
    instance_ = this;
}

bool RobotAssemblerPlugin::initialize()
{
    DEBUG_STREAM_INFO(RobotAssemblerPlugin,initialize(), std::endl);
    OptionManager& om = this->optionManager();
    om.addOption("assembler", po::value<std::string>(), "load robot_assembler config file");
    om.sigOptionsParsed(1).connect(onSigOptionsParsed);

    // classes
    AssemblerBodyItem::initializeClass(this);

    //View
    AssemblerView::initializeClass(this);

    //ToolBar
    addToolBar(AssemblerBar::instance());

    return true;
}

bool RobotAssemblerPlugin::finalize()
{
    instance_ = nullptr;
    return true;
}

const char* RobotAssemblerPlugin::description() const
{
    static std::string text =
        fmt::format("RobotAssembler Plugin Version {}\n", CNOID_FULL_VERSION_STRING) +
        "\n" +
        "Copyrigh (c) 2022 IRSL-tut Development Team.\n"
        "\n" +
        MITLicenseText() +
        "\n"  ;

    return text.c_str();
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(RobotAssemblerPlugin);
