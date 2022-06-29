#include "SimStepControllerItem.h"
#include <cnoid/SimpleController>
#include <cnoid/BodyItem>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/UTF8>
#include <cnoid/ConnectionSet>
#include <cnoid/ProjectManager>
#include <cnoid/ItemManager>
#include <QLibrary>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <set>
#include <bitset>
#include <algorithm>

#include <iostream>

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;

void SimStepControllerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& itemManager = ext->itemManager();
    itemManager.registerClass<SimStepControllerItem, ControllerItem>("SimStepControllerItem");
    itemManager.addCreationPanel<SimStepControllerItem>();
}

SimStepControllerItem::SimStepControllerItem()
{
    setName("SimStepController");
    simulationBody = nullptr;
    io = nullptr;
    wait_counter = 0;
    stop_on_output_ = false;
    at_output_ = false;
    mu.lock();
}

SimStepControllerItem::SimStepControllerItem(const SimStepControllerItem& org)
    : ControllerItem(org)
{
    simulationBody = nullptr;
    io = nullptr;
    wait_counter = 0;
    stop_on_output_ = false;
    at_output_ = false;
    mu.lock();
}

SimStepControllerItem::~SimStepControllerItem()
{
}

bool SimStepControllerItem::checkIfSubController(ControllerItem* controllerItem) const
{
    return false;
}


bool SimStepControllerItem::initialize(ControllerIO* io)
{
#ifdef _IRSL_DEBUG_
    std::cerr << "ss::initialize" << std::endl;
#endif
    this->io = io;
    this->simulationBody = io->body();

    if ( !(this->simulationBody) ) {
        std::cerr << "io->body() is empty?" << std::endl;
    }

    return true;
}

double SimStepControllerItem::timeStep() const
{
    return io ? io->timeStep() : 0.0;
}

bool SimStepControllerItem::start()
{
#ifdef _IRSL_DEBUG_
    std::cerr << "ss:start" << std::endl;
#endif
    started = true;
    return true;
}

void SimStepControllerItem::input()
{
}

bool SimStepControllerItem::control()
{
    if (started) wait_control_notify();
    return true;
}

void SimStepControllerItem::output()
{
    if (started && stop_on_output_) {
        at_output_ = true;
        wait_control_notify();
        at_output_ = false;
    }
}

void SimStepControllerItem::stop()
{
#ifdef _IRSL_DEBUG_
    std::cerr << "ss:stop" << std::endl;
#endif
    started = false;
    notify_sim_step();
}

double SimStepControllerItem::currentTime() const
{
    return io ? io->currentTime() : 0.0;
}

// protected
Item* SimStepControllerItem::doDuplicate() const
{
    return new SimStepControllerItem(*this);
}

void SimStepControllerItem::onTreePathChanged()
{
#ifdef _IRSL_DEBUG_
    std::cerr << "ss:onTreePathChanged" << std::endl;
#endif
    bool isTargetBodyItemChanged = false;
    bool connected = isConnectedToRoot();
    BodyItem* bodyItem = nullptr;
    if(connected){
        bodyItem = findOwnerItem<BodyItem>();
    }
}

void SimStepControllerItem::onDisconnectedFromRoot()
{
#ifdef _IRSL_DEBUG_
    std::cerr << "ss:onDisconnectedFromRoot" << std::endl;
#endif
    if(!isActive()){
        // process if active
    }
    //
}

void SimStepControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
#ifdef _IRSL_DEBUG_
    std::cerr << "ss:doPutProperties" << std::endl;
#endif
    ControllerItem::doPutProperties(putProperty);
    // process properties
}

bool SimStepControllerItem::store(Archive& archive)
{
#ifdef _IRSL_DEBUG_
    std::cerr << "store" << std::endl;
#endif
    if(!ControllerItem::store(archive)){
        return false;
    }
    // store something
    return true;
}

bool SimStepControllerItem::restore(const Archive& archive)
{
#ifdef _IRSL_DEBUG_
    std::cerr << "restore" << std::endl;
#endif
    if(!ControllerItem::restore(archive)){
        return false;
    }
    // restore something
    return true;
}
