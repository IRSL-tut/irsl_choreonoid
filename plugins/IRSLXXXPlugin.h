#ifndef CNOID_IRSL_XXX_PLUGIN_H
#define CNOID_IRSL_XXX_PLUGIN_H

#include <cnoid/Plugin>

namespace cnoid {

class IRSLXXXPlugin : public Plugin
{
public:
    static IRSLXXXPlugin* instance();
    IRSLXXXPlugin();
    virtual bool initialize() override;
    virtual bool finalize() override;
    virtual const char* description() const override;
};

}

#endif
