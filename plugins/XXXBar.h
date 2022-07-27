#ifndef CNOID_IRSL_XXX_PLUGIN_XXXBAR_H
#define CNOID_IRSL_XXX_PLUGIN_XXXBAR_H

#include <cnoid/ToolBar>

namespace cnoid {

class XXXBar : public ToolBar
{
public:
    static XXXBar* instance();
    virtual ~XXXBar();

private:
    XXXBar();

    class Impl;
    Impl* impl;
};

}
#endif
