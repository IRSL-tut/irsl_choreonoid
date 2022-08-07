#include <cnoid/OptionManager> //??

#include "RobotAssemblerPlugin.h"
#include "AssemblerItem.h"
#include "AssemblerView.h"
#include "AssemblerTreeView.h"
#include "AssemblerBar.h"
#include "AssemblerManager.h"

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

    void onSigOptionsParsed(po::variables_map& variables);
};

}
void RobotAssemblerPlugin::Impl::onSigOptionsParsed(po::variables_map& variables)
{
    // plugin created
    // initializeClass
    // creating project ( from .cnoid )
    // onSigOptionParsed
    DEBUG_PRINT();
    if(variables.count("assembler")) {
        std::string fname_ = variables["assembler"].as<std::string>();
        DEBUG_STREAM("robot_assembler config file: " << fname_);

        ra::SettingsPtr ra_settings = std::make_shared<ra::Settings> ();
        bool ret = ra_settings->parseYaml(fname_);

        if (ret) {
            DEBUG_STREAM(" settings created");
            AssemblerManager *manager = AssemblerManager::instance();
            if(!!manager) {
                DEBUG_STREAM(" manager find 0");
                manager->ra_settings = ra_settings;
                DEBUG_STREAM(" manager find 1");
                manager->roboasm = std::make_shared<ra::Roboasm>(ra_settings);
                DEBUG_STREAM(" manager find 2");
                AssemblerView *ptr = AssemblerView::instance();
                DEBUG_STREAM(" manager find 3");
                ptr->createButtons();
            }
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
    DEBUG_PRINT();
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
    DEBUG_PRINT();
    OptionManager& om = this->optionManager();
    om.addOption("assembler", po::value<std::string>(), "load robot_assembler config file");
    //om.sigOptionsParsed(1).connect(onSigOptionsParsed);
    om.sigOptionsParsed(1).connect(
        [&](po::variables_map& _v) { impl->onSigOptionsParsed(_v); } );

    // classes
    AssemblerItem::initializeClass(this);

    //View
    AssemblerView::initializeClass(this);
    AssemblerTreeView::initializeClass(this);

    //ToolBar
    addToolBar(AssemblerBar::instance());
    DEBUG_STREAM(" FINISH Plugin initialize");
    return true;
}
bool RobotAssemblerPlugin::finalize()
{
    DEBUG_PRINT();
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
