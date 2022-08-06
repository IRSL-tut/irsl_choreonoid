#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_BAR_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_BAR_H

#include <cnoid/ToolBar>

namespace cnoid {

class AssemblerBar : public ToolBar
{
public:
    static AssemblerBar* instance();
    virtual ~AssemblerBar();

    SignalProxy<void(int flags)> sigAlignDecl;
    SignalProxy<void(int flags)> sigAlignIncl;
    SignalProxy<void(int flags)> sigUnAlign;
    SignalProxy<void(int flags)> sigUnAttach;

    SignalProxy<void(int flags)> sigUndo;
    SignalProxy<void(int flags)> sigRedo;

    SignalProxy<void(int flags)> sigWrite;
    SignalProxy<void(int flags)> sigDeleteAll;

private:
    AssemblerBar();

    class Impl;
    Impl* impl;
};

}
#endif
