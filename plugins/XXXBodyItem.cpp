#include "XXXBodyItem.h"
#include "XXXSceneBody.h"

//#include <cnoid/SceneBody>
#include <cnoid/RootItem>
#include <cnoid/LazySignal>
#include <cnoid/ItemManager>
#include <cnoid/OptionManager> //??
#include <cnoid/PutPropertyFunction>

#include <fmt/format.h>
#include <bitset>
#include <algorithm>
#include <iostream>


using namespace std;
using namespace cnoid;
using fmt::format;

namespace {
#if 0
class BodyLocation : public LocationProxy
{
public:
    XXXBodyItem::Impl* impl;

    BodyLocation(XXXBodyItem::Impl* impl);
    void updateLocationType();
    virtual Isometry3 getLocation() const override;
    virtual bool isEditable() const override;
    virtual void setEditable(bool on) override;
    virtual bool setLocation(const Isometry3& T) override;
    virtual void finishLocationEditing() override;
    virtual Item* getCorrespondingItem() override;
    virtual LocationProxyPtr getParentLocationProxy() const override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

class LinkLocation : public LocationProxy
{
public:
    weak_ref_ptr<XXXBodyItem> refXXXBodyItem;
    weak_ref_ptr<Link> refLink;

    LinkLocation();
    LinkLocation(XXXBodyItem* bodyItem, Link* link);
    void setTarget(XXXBodyItem* bodyItem, Link* link);
    virtual std::string getName() const override;
    virtual Isometry3 getLocation() const override;
    virtual bool isEditable() const override;
    virtual Item* getCorrespondingItem() override;
    virtual LocationProxyPtr getParentLocationProxy() const override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};
#endif
}

namespace cnoid {

class XXXBodyItem::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    XXXBodyItem* self;
    BodyPtr body;

    XXXSceneBodyPtr sceneBody;

    Impl(XXXBodyItem* self);
    Impl(XXXBodyItem* self, const Impl& org);
    Impl(XXXBodyItem* self, Body* body);
    ~Impl();

    void setBody(Body* body);
    void setTransparency(float t);
    void createSceneBody();

    // location
    //ref_ptr<BodyLocation> bodyLocation;
    //ref_ptr<LinkLocation> parentLinkLocation;

    // signals
    LazySignal<Signal<void()>> sigKinematicStateChanged;
    Signal<void()> sigKinematicStateUpdated;
    Signal<void(int flags)> sigModelUpdated;

    float transparency;

    //notify
    void notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK, bool isDirect);
    void emitSigKinematicStateChanged();
};

}

static void onSigOptionsParsed(boost::program_options::variables_map& variables)
{
    if(variables.count("xxxbody")){
        vector<string> bodyFileNames = variables["xxxbody"].as<vector<string>>();
        for(size_t i=0; i < bodyFileNames.size(); ++i){
            XXXBodyItemPtr item(new XXXBodyItem);
            auto rootItem = RootItem::instance();
            if(item->load(bodyFileNames[i], rootItem, "CHOREONOID-BODY")){
                item->setChecked(true);
                rootItem->addChildItem(item);
            }
        }
    }
}

void XXXBodyItem::initializeClass(ExtensionManager* ext)
{
    ItemManager* im = &ext->itemManager();
    im->registerClass<XXXBodyItem>("XXXBodyItem");

    // Implemented in XXXBodyItemFileIO.cpp
    registerXXXBodyItemFileIoSet(im);

    //// option ???
    OptionManager& om = ext->optionManager();
    om.addOption("xxxbody", boost::program_options::value< vector<string> >(), "load a xxxbody file");
    om.sigOptionsParsed(1).connect(onSigOptionsParsed);
}

////
XXXBodyItem::XXXBodyItem()
{
    setAttributes(FileImmutable | Reloadable);//TODO
    impl = new Impl(this);
    // impl->init(false);
}

XXXBodyItem::XXXBodyItem(const std::string& name)
    : XXXBodyItem()
{
    XXXBodyItem::setName(name);
}

XXXBodyItem::XXXBodyItem(const XXXBodyItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);

    setChecked(org.isChecked());
}

XXXBodyItem::~XXXBodyItem()
{
    delete impl;
}

//// Impl
XXXBodyItem::Impl::Impl(XXXBodyItem* self)
    : Impl(self, new Body)
{
    body->rootLink()->setName("Root");
}

XXXBodyItem::Impl::Impl(XXXBodyItem* self, Body* body)
    : self(self),
      body(body)
{
}

XXXBodyItem::Impl::Impl(XXXBodyItem* self, const Impl& org)
    : Impl(self, org.body->clone())
{
}

XXXBodyItem::Impl::~Impl()
{
}

//// protected / override Item Class
Item* XXXBodyItem::doDuplicate() const
{
    std::cerr << "doDuplicate(XXXBodyItem)" << std::endl;
    return new XXXBodyItem(*this);
}
bool XXXBodyItem::doAssign(const Item* srcItem)
{
    std::cerr << "doAssign(XXXBodyItem)" << std::endl;
    return false;
    //??
    //return impl->doAssign(srcItem);
}
void XXXBodyItem::onTreePathChanged()
{
    std::cerr << "doTreePathChanged(XXXBodyItem)" << std::endl;
#if 0
    auto worldItem = findOwnerItem<WorldItem>();
    if(!worldItem){
        clearCollisions();
    }

    /*
      If the item is being restored in loading a project, the attachment update is
      processed when all the child items are restored so that an attachment device
      dynamically added by a DeviceOverwriteItem can work in the attachment update.
      In that case, the attachment update is processed by the callback function
      given to the Archive::addProcessOnSubTreeRestored in the restore function.
    */
    if(!impl->isBeingRestored){
        impl->updateAttachment(true, true);
    }
#endif
}
void XXXBodyItem::onConnectedToRoot()
{
    std::cerr << "doTreePathChanged(XXXBodyItem)" << std::endl;
    //storeKinematicState(impl->lastEditState);
}
void XXXBodyItem::doPutProperties(PutPropertyFunction& putProperty)
{
    std::cerr << "doPutProperties(XXXBodyItem)" << std::endl;
    //impl->doPutProperties(putProperty);
}
#if 0
void XXXBodyItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
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
bool XXXBodyItem::setName(const std::string& name)
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

Body* XXXBodyItem::body() const
{
    return impl->body;
}

void XXXBodyItem::setBody(Body* body)
{
    impl->setBody(body);
}

void XXXBodyItem::Impl::setBody(Body* body_)
{
    body = body_;
    auto rootLink = body->rootLink();
    if(rootLink->name().empty()){
        rootLink->setName("Root");
    }

    body->initializePosition();//

    auto& itemName = self->name();
    if(itemName.empty()){
        if(!body_->name().empty()){
            self->setName(body_->name());
        } else if(!body_->modelName().empty()){
            self->setName(body_->modelName());
        }
    } else {
        if(body_->name().empty()){
            body->setName(itemName);
        }
        if(body_->modelName().empty()){
            body->setModelName(itemName);
        }
    }
}

//// signals
SignalProxy<void()> XXXBodyItem::sigKinematicStateChanged()
{
    return impl->sigKinematicStateChanged.signal();
}
SignalProxy<void()> XXXBodyItem::sigKinematicStateUpdated()
{
    return impl->sigKinematicStateUpdated;
}
SignalProxy<void(int flags)> XXXBodyItem::sigModelUpdated()
{
    return impl->sigModelUpdated;
}
void XXXBodyItem::notifyModelUpdate(int flags)
{
    impl->sigModelUpdated(flags);
}
void XXXBodyItem::notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->notifyKinematicStateChange(requestFK, requestVelFK, requestAccFK, true);
}
void XXXBodyItem::notifyKinematicStateChange(Connection& connectionToBlock, bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->sigKinematicStateChanged.requestBlocking(connectionToBlock);
    impl->notifyKinematicStateChange(requestFK, requestVelFK, requestAccFK, true);
}
void XXXBodyItem::notifyKinematicStateChangeLater(bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->notifyKinematicStateChange(requestFK, requestVelFK,requestAccFK, false);
}
void XXXBodyItem::notifyKinematicStateChangeLater(Connection& connectionToBlock, bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->sigKinematicStateChanged.requestBlocking(connectionToBlock);
    impl->notifyKinematicStateChange(requestFK, requestVelFK, requestAccFK, false);
}
void XXXBodyItem::notifyKinematicStateUpdate(bool doNotifyStateChange)
{
    if(doNotifyStateChange){
        impl->notifyKinematicStateChange(false, false, false, true);
    }
    impl->sigKinematicStateUpdated();

    //if(isAttachedToParentBody_){
    //impl->parentXXXBodyItem->notifyKinematicStateUpdate(false);
    //}

    //auto record = new KinematicStateRecord(impl, impl->lastEditState);
    //UnifiedEditHistory::instance()->addRecord(record);
    //storeKinematicState(impl->lastEditState);
}

void XXXBodyItem::Impl::notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK, bool isDirect)
{
#if 0
    updateElements.reset();

    if(isProcessingInverseKinematicsIncludingParentBody){
        isProcessingInverseKinematicsIncludingParentBody = false;
        if(parentXXXBodyItem){
            parentXXXBodyItem->impl->notifyKinematicStateChange(
                requestFK, requestVelFK, requestAccFK, isDirect);
        }
    } else {
        if(requestFK){
            isFkRequested |= requestFK;
            isVelFkRequested |= requestVelFK;
            isAccFkRequested |= requestAccFK;
        }
        if(isDirect){
            sigKinematicStateChanged.emit();
        } else {
            sigKinematicStateChanged.request();
        }
    }
#endif
    sigKinematicStateChanged.emit();
    //sigKinematicStateChanged.request();
}

void XXXBodyItem::Impl::emitSigKinematicStateChanged()
{
#if 0
    if(isFkRequested){
        fkTraverse.calcForwardKinematics(isVelFkRequested, isAccFkRequested);
        isFkRequested = isVelFkRequested = isAccFkRequested = false;
    }
    if(isKinematicStateChangeNotifiedByParentXXXBodyItem){
        isKinematicStateChangeNotifiedByParentXXXBodyItem = false;
    } else {
        if(parentXXXBodyItem && !attachmentToParent){
            setRelativeOffsetPositionFromParentBody();
        }
    }
#endif
    sigKinematicStateChanged.signal()();
}
#if 0
LocationProxyPtr XXXBodyItem::getLocationProxy()
{
    if(!impl->bodyLocation){
        impl->bodyLocation = new BodyLocation(impl);
    }
    return impl->bodyLocation;
}
LocationProxyPtr XXXBodyItem::createLinkLocationProxy(Link* link)
{
    return new LinkLocation(this, link);
}
#endif

#if 0
namespace {
BodyLocation::BodyLocation(XXXBodyItem::Impl* impl)
    : LocationProxy(impl->attachmentToParent ? OffsetLocation : GlobalLocation),
      impl(impl)
{

}
void BodyLocation::updateLocationType()
{
    if(impl->attachmentToParent){
        setLocationType(OffsetLocation);
    } else {
        setLocationType(GlobalLocation);
    }
}
Isometry3 BodyLocation::getLocation() const
{
    auto rootLink = impl->body->rootLink();
    if(impl->attachmentToParent){
        // relative position from the parent link
        return rootLink->offsetPosition();
    } else {
        // global position
        return rootLink->T();
    }
}
bool BodyLocation::isEditable() const
{
    return impl->self->isLocationEditable();
}
void BodyLocation::setEditable(bool on)
{
    impl->setLocationEditable(on, true);
}
bool BodyLocation::setLocation(const Isometry3& T)
{
    auto rootLink = impl->body->rootLink();
    if(impl->attachmentToParent){
        rootLink->setOffsetPosition(T);
        impl->parentXXXBodyItem->notifyKinematicStateChange(true);
    } else {
        rootLink->setPosition(T);
        impl->body->calcForwardKinematics();
        impl->self->notifyKinematicStateChange();
    }
    return true;
}
void BodyLocation::finishLocationEditing()
{
    impl->self->notifyKinematicStateUpdate(false);
}
Item* BodyLocation::getCorrespondingItem()
{
    return impl->self;
}
LocationProxyPtr BodyLocation::getParentLocationProxy() const
{
    if(impl->parentXXXBodyItem){
        if(impl->attachmentToParent){
            if(!impl->parentLinkLocation){
                impl->parentLinkLocation = new LinkLocation;
            }
            auto parentLink = impl->body->parentBodyLink();
            impl->parentLinkLocation->setTarget(impl->parentXXXBodyItem, parentLink);
            return impl->parentLinkLocation;
        } else {
            return impl->parentXXXBodyItem->getLocationProxy();
        }
    }
    return nullptr;
}
SignalProxy<void()> BodyLocation::sigLocationChanged()
{
    return impl->sigKinematicStateChanged.signal();
}
LinkLocation::LinkLocation()
    : LocationProxy(GlobalLocation)
{

}
LinkLocation::LinkLocation(XXXBodyItem* bodyItem, Link* link)
    : LocationProxy(GlobalLocation),
      refXXXBodyItem(bodyItem),
      refLink(link)
{

}
void LinkLocation::setTarget(XXXBodyItem* bodyItem, Link* link)
{
    refXXXBodyItem = bodyItem;
    refLink = link;
}
std::string LinkLocation::getName() const
{
    if(auto link = refLink.lock()){
        return link->body()->name() + " - " + link->name();
    }
    return string();
}
Isometry3 LinkLocation::getLocation() const
{
    if(auto link = refLink.lock()){
        return link->T();
    }
    return Isometry3::Identity();
}
bool LinkLocation::isEditable() const
{
    return false;
}
Item* LinkLocation::getCorrespondingItem()
{
    return refXXXBodyItem.lock();
}
LocationProxyPtr LinkLocation::getParentLocationProxy() const
{
    if(auto body = refXXXBodyItem.lock()){
        body->getLocationProxy();
    }
    return nullptr;
}
SignalProxy<void()> LinkLocation::sigLocationChanged()
{
    if(auto bodyItem = refXXXBodyItem.lock()){
        return bodyItem->sigKinematicStateChanged();
    } else {
        static Signal<void()> dummySignal;
        return dummySignal;
    }
}
}
#endif

////
XXXSceneBody* XXXBodyItem::sceneBody()
{
    if(!impl->sceneBody){
        impl->createSceneBody();
    }
    return impl->sceneBody;
}
SgNode* XXXBodyItem::getScene()
{
    return sceneBody();
}
void XXXBodyItem::Impl::createSceneBody()
{
    sceneBody = new XXXSceneBody(self);
    sceneBody->setSceneDeviceUpdateConnection(true);
    if(transparency > 0.0f){
        sceneBody->setTransparency(transparency);
    }
}

//// transp
float XXXBodyItem::transparency() const
{
    return impl->transparency;
}
void XXXBodyItem::setTransparency(float t)
{
    impl->setTransparency(t);
}
void XXXBodyItem::Impl::setTransparency(float t)
{
    if(t != transparency){
        if(sceneBody){
            sceneBody->setTransparency(t);
        }
        transparency = t;
    }
}
