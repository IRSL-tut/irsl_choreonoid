#ifndef CNOID_IRSL_SIM_PLUGIN_H
#define CNOID_IRSL_SIM_PLUGIN_H

#include <cnoid/Plugin>

namespace cnoid {

class IRSLSimPlugin : public Plugin
{
public:
    static IRSLSimPlugin* instance();
    IRSLSimPlugin();
    virtual bool initialize() override;
    virtual bool finalize() override;
    virtual const char* description() const override;
};

}

#endif
