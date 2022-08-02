#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_ITEM_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_ITEM_H

#include <cnoid/Item>
// #include <cnoid/LocatableItem>
#include <cnoid/RenderableItem>
#include "exportdecl.h"

#include "RobotAssembler.h"

namespace cnoid {
class ItemManager;

class AssemblerItem;

typedef ref_ptr<AssemblerItem> AssemblerItemPtr;

//public LocatableItem
class CNOID_EXPORT AssemblerItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    AssemblerItem();
    AssemblerItem(const std::string& name);
    AssemblerItem(const AssemblerItem& org);
    virtual ~AssemblerItem();

    virtual bool setName(const std::string& name) override;

    // API for a composite body
    // The following body and link pair is basically determined by
    // the parent-child relationship in the item tree
    //AssemblerItem* parentAssemblerItem();

    SignalProxy<void(int flags)> sigModelUpdated();
    void notifyModelUpdate(int flags);

    // RenderableItem function
    virtual SgNode* getScene() override;

    float transparency() const;
    void setTransparency(float t);

protected:
    virtual Item* doDuplicate() const override;
    virtual bool doAssign(const Item* item) override;
    virtual void onTreePathChanged() override;
    virtual void onConnectedToRoot() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    class Impl;
    Impl* impl;
};


}

#endif
