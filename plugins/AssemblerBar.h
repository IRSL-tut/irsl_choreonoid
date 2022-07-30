#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_BAR_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_BAR_H

#include <cnoid/ToolBar>

namespace cnoid {

class AssemblerBar : public ToolBar
{
public:
    static AssemblerBar* instance();
    virtual ~AssemblerBar();

private:
    AssemblerBar();

    class Impl;
    Impl* impl;
};

}
#endif
