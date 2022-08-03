#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_VIEW_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_VIEW_H

#include <cnoid/View>
#include "RobotAssembler.h"
#include "exportdecl.h"

namespace ra = cnoid::robot_assembler;

namespace cnoid {

class CNOID_EXPORT AssemblerView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static AssemblerView* instance(); // instance??

    AssemblerView();
    virtual ~AssemblerView();

    void createButtons(ra::SettingsPtr &ra_settings);

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;
    virtual void onAttachedMenuRequest(MenuManager& menuManager) override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
