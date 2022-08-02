#include "AssemblerItem.h"
#include "RobotAssemblerHelper.h"

#include <cnoid/RootItem>
#include <cnoid/LazySignal>
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>

#include <fmt/format.h>
#include <bitset>
#include <algorithm>
#include <iostream>

#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class AssemblerItem::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AssemblerItem* self;
    SgNodePtr scene;

    Impl(AssemblerItem* _self);
    Impl(AssemblerItem* _self, const Impl& org);

    ~Impl();

    void setTransparency(float t);

    Signal<void(int flags)> sigModelUpdated;

    float transparency;
};

}

void AssemblerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager* im = &ext->itemManager();
    im->registerClass<AssemblerItem>("AssemblerItem");
}

////
AssemblerItem::AssemblerItem()
{
    setAttributes(FileImmutable | Reloadable);//TODO
    impl = new Impl(this);
}

AssemblerItem::AssemblerItem(const std::string& name)
    : AssemblerItem()
{
    AssemblerItem::setName(name);
}

AssemblerItem::AssemblerItem(const AssemblerItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
    setChecked(org.isChecked());
}

AssemblerItem::~AssemblerItem()
{
    std::cerr << "delete AssemblerItem: " << name() << std::endl;
    delete impl;
}

//// Impl
AssemblerItem::Impl::Impl(AssemblerItem* _self)
    : self(_self)
{
}

AssemblerItem::Impl::Impl(AssemblerItem* _self, const Impl& org)
    : self(_self)
{
}

AssemblerItem::Impl::~Impl()
{
}

//// protected / override Item Class
Item* AssemblerItem::doDuplicate() const
{
    std::cerr << "doDuplicate(AssemblerItem)" << std::endl;
    return new AssemblerItem(*this);
}
bool AssemblerItem::doAssign(const Item* srcItem)
{
    std::cerr << "doAssign(AssemblerItem)" << std::endl;
    return false;
    //??
    //return impl->doAssign(srcItem);
}
void AssemblerItem::onTreePathChanged()
{
    std::cerr << "doTreePathChanged(AssemblerItem)" << std::endl;
}
void AssemblerItem::onConnectedToRoot()
{
    std::cerr << "doTreePathChanged(AssemblerItem)" << std::endl;
}
void AssemblerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    std::cerr << "doPutProperties(AssemblerItem)" << std::endl;
}
#if 0
void AssemblerItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Model name"), body->modelName());
    putProperty(_("Num links"), body->numLinks());
    putProperty(_("Num joints"), body->numJoints());
    putProperty(_("Num devices"), (int)body->devices().size());
    putProperty(_("Root link"), body->rootLink()->name());
    putProperty(_("Base link"), currentBaseLink ? currentBaseLink->name() : "none");
    putProperty.decimals(3)(_("Mass"), body->mass());
    putProperty(_("Static"), body->isStaticModel(),
                [&](bool on){ return on ? makeBodyStatic() : makeBodyDynamic(); });
    putProperty(_("Collision detection"), isCollisionDetectionEnabled,
                [&](bool on){ return setCollisionDetectionEnabled(on); });
    putProperty(_("Self-collision detection"), isSelfCollisionDetectionEnabled,
                [&](bool on){ return setSelfCollisionDetectionEnabled(on); });
    putProperty(_("Location editable"), self->isLocationEditable(),
                [&](bool on){ setLocationEditable(on, true); return true; });
    putProperty(_("Scene sensitive"), self->isSceneSensitive(),
                [&](bool on){ self->setSceneSensitive(on); return true; });
    putProperty.range(0.0, 0.9).decimals(1);
    putProperty(_("Transparency"), transparency,
                [&](float value){ setTransparency(value); return true; });
    putProperty(_("Visible link selection"), self->isVisibleLinkSelectionMode_,
                changeProperty(self->isVisibleLinkSelectionMode_));

    if(isAttachable()){
        putProperty(_("Enable attachment"), isAttachmentEnabled,
                    [&](bool on){ self->setAttachmentEnabled(on, false); return true; });
    }
}
#endif
//// protected

// override
bool AssemblerItem::setName(const std::string& name)
{
#if 0
    auto body = impl->body;
    if(body){
        body->setName(name);
        if(body->modelName().empty()){
            body->setModelName(name);
        }
    }
#endif
    return Item::setName(name);
}

//// signals
SignalProxy<void(int flags)> AssemblerItem::sigModelUpdated()
{
    return impl->sigModelUpdated;
}
void AssemblerItem::notifyModelUpdate(int flags)
{
    impl->sigModelUpdated(flags);
}

////
SgNode* AssemblerItem::getScene()
{
    return impl->scene;
}

//// transp
float AssemblerItem::transparency() const
{
    return impl->transparency;
}
void AssemblerItem::setTransparency(float t)
{
    impl->setTransparency(t);
}
void AssemblerItem::Impl::setTransparency(float t)
{
    //[todo]
}
