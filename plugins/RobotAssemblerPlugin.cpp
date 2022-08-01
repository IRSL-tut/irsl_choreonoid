#include <cnoid/OptionManager> //??

#include "RobotAssemblerPlugin.h"
#include "AssemblerBodyItem.h"
#include "AssemblerView.h"
#include "AssemblerBar.h"

#include <fmt/format.h>

#include "RobotAssembler.h"

#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;

namespace po = boost::program_options;
namespace ra = cnoid::robot_assembler;

namespace {
RobotAssemblerPlugin* instance_ = nullptr;
}

namespace cnoid {

class RobotAssemblerPlugin::Impl
{
public:
    Impl(RobotAssemblerPlugin *_self) { self = _self; }

    RobotAssemblerPlugin *self;
    ra::SettingsPtr ra_settings;

    void onSigOptionsParsed(po::variables_map& variables);
};

}

void RobotAssemblerPlugin::Impl::onSigOptionsParsed(po::variables_map& variables)
{
    if(variables.count("assembler")) {
        std::string fname_ = variables["assembler"].as<std::string>();
        DEBUG_STREAM("robot_assembler config file: " << fname_ << std::endl);

        ra_settings = std::make_shared<ra::Settings> ();
        bool ret = ra_settings->parseYaml(fname_);

        if (ret) {
            AssemblerView *ptr = AssemblerView::instance();
            ptr->createButtons(ra_settings);
        }
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

    impl = new Impl(this);
}
RobotAssemblerPlugin::~RobotAssemblerPlugin()
{
    delete impl;
}

bool RobotAssemblerPlugin::initialize()
{
    DEBUG_STREAM_INFO(RobotAssemblerPlugin,initialize(), std::endl);
    OptionManager& om = this->optionManager();
    om.addOption("assembler", po::value<std::string>(), "load robot_assembler config file");
    //om.sigOptionsParsed(1).connect(onSigOptionsParsed);
    om.sigOptionsParsed(1).connect(
        [&](po::variables_map& _v) { impl->onSigOptionsParsed(_v); } );

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
    DEBUG_STREAM_INFO(RobotAssemblerPlugin,finalize, std::endl);
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
