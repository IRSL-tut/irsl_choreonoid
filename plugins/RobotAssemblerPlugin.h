#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_H

#include <cnoid/Plugin>

namespace cnoid {

class RobotAssemblerPlugin : public Plugin
{
public:
    static RobotAssemblerPlugin* instance();
    RobotAssemblerPlugin();
    virtual bool initialize() override;
    virtual bool finalize() override;
    virtual const char* description() const override;
};

}

#endif
