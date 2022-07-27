#ifndef CNOID_IRSL_XXX_PLUGIN_VIEW_H
#define CNOID_IRSL_XXX_PLUGIN_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT XXXView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static XXXView* instance(); // instance??

    XXXView();
    virtual ~XXXView();

    void createButtons();
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
