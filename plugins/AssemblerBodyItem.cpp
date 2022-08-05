#include "AssemblerBodyItem.h"
#include "AssemblerSceneBody.h"
#include "RobotAssemblerHelper.h"

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

#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {
#if 0
class BodyLocation : public LocationProxy
{
public:
    AssemblerBodyItem::Impl* impl;

    BodyLocation(AssemblerBodyItem::Impl* impl);
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
    weak_ref_ptr<AssemblerBodyItem> refAssemblerBodyItem;
    weak_ref_ptr<Link> refLink;

    LinkLocation();
    LinkLocation(AssemblerBodyItem* bodyItem, Link* link);
    void setTarget(AssemblerBodyItem* bodyItem, Link* link);
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

class AssemblerBodyItem::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AssemblerBodyItem* self;
    BodyPtr body;

    AssemblerSceneBodyPtr sceneBody;

    Impl(AssemblerBodyItem* self);
    Impl(AssemblerBodyItem* self, const Impl& org);
    Impl(AssemblerBodyItem* self, Body* body);
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
            AssemblerBodyItemPtr item(new AssemblerBodyItem);
            auto rootItem = RootItem::instance();
            if(item->load(bodyFileNames[i], rootItem, "CHOREONOID-BODY")){
                item->setChecked(true);
                rootItem->addChildItem(item);
            }
        }
    }
}

void AssemblerBodyItem::initializeClass(ExtensionManager* ext)
{
    ItemManager* im = &ext->itemManager();
    im->registerClass<AssemblerBodyItem>("AssemblerBodyItem");

    // Implemented in AssemblerBodyItemFileIO.cpp
    registerAssemblerBodyItemFileIoSet(im);

    //// option ???
    OptionManager& om = ext->optionManager();
    om.addOption("xxxbody", boost::program_options::value< vector<string> >(), "load a xxxbody file");
    om.sigOptionsParsed(1).connect(onSigOptionsParsed);
}

AssemblerBodyItemPtr AssemblerBodyItem::createItemFromAssemblerConf
(const std::string &name, cnoid::robot_assembler::Settings &ra_settings)
{
    AssemblerBodyItemPtr itm(new AssemblerBodyItem());

    auto it = ra_settings.mapParts.find(name);
    if (it == ra_settings.mapParts.end()) {
        std::cerr << "designated parts: " << name << " was not found!"<< std::endl;
    }

    SgNode *sg = cnoid::robot_assembler::createSceneFromGeomatry(it->second.visual);
    if (!!sg) {
        DEBUG_STREAM_NL_INFO(AssemblerBodyItem, createItemFromAssemblerConf, "sg created" << std::endl);
        BodyPtr newBody = new Body;
        newBody->rootLink()->addShapeNode(sg);
        newBody->setName("Loaded_Body");
        itm->setBody(newBody);
    }

    return itm;
}

////
AssemblerBodyItem::AssemblerBodyItem()
{
    setAttributes(FileImmutable | Reloadable);//TODO
    impl = new Impl(this);
    // impl->init(false);
}

AssemblerBodyItem::AssemblerBodyItem(const std::string& name)
    : AssemblerBodyItem()
{
    AssemblerBodyItem::setName(name);
}

AssemblerBodyItem::AssemblerBodyItem(const AssemblerBodyItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);

    setChecked(org.isChecked());
}

AssemblerBodyItem::~AssemblerBodyItem()
{
    std::cerr << "delete AssemblerBodyItem: " << name() << std::endl;
    delete impl;
}

//// Impl
AssemblerBodyItem::Impl::Impl(AssemblerBodyItem* self)
    : Impl(self, new Body)
{
    body->rootLink()->setName("Root");
}

AssemblerBodyItem::Impl::Impl(AssemblerBodyItem* self, Body* body)
    : self(self),
      body(body)
{
}

AssemblerBodyItem::Impl::Impl(AssemblerBodyItem* self, const Impl& org)
    : Impl(self, org.body->clone())
{
}

AssemblerBodyItem::Impl::~Impl()
{
}

//// protected / override Item Class
Item* AssemblerBodyItem::doDuplicate() const
{
    std::cerr << "doDuplicate(AssemblerBodyItem)" << std::endl;
    return new AssemblerBodyItem(*this);
}
bool AssemblerBodyItem::doAssign(const Item* srcItem)
{
    std::cerr << "doAssign(AssemblerBodyItem)" << std::endl;
    return false;
    //??
    //return impl->doAssign(srcItem);
}
void AssemblerBodyItem::onTreePathChanged()
{
    std::cerr << "doTreePathChanged(AssemblerBodyItem)" << std::endl;
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
void AssemblerBodyItem::onConnectedToRoot()
{
    std::cerr << "doTreePathChanged(AssemblerBodyItem)" << std::endl;
    //storeKinematicState(impl->lastEditState);
}
void AssemblerBodyItem::doPutProperties(PutPropertyFunction& putProperty)
{
    std::cerr << "doPutProperties(AssemblerBodyItem)" << std::endl;
    //impl->doPutProperties(putProperty);
}
#if 0
void AssemblerBodyItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
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
bool AssemblerBodyItem::setName(const std::string& name)
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

Body* AssemblerBodyItem::body() const
{
    return impl->body;
}

void AssemblerBodyItem::setBody(Body* body)
{
    impl->setBody(body);
}

void AssemblerBodyItem::Impl::setBody(Body* body_)
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
SignalProxy<void()> AssemblerBodyItem::sigKinematicStateChanged()
{
    return impl->sigKinematicStateChanged.signal();
}
SignalProxy<void()> AssemblerBodyItem::sigKinematicStateUpdated()
{
    return impl->sigKinematicStateUpdated;
}
SignalProxy<void(int flags)> AssemblerBodyItem::sigModelUpdated()
{
    return impl->sigModelUpdated;
}
void AssemblerBodyItem::notifyModelUpdate(int flags)
{
    impl->sigModelUpdated(flags);
}
void AssemblerBodyItem::notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->notifyKinematicStateChange(requestFK, requestVelFK, requestAccFK, true);
}
void AssemblerBodyItem::notifyKinematicStateChange(Connection& connectionToBlock, bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->sigKinematicStateChanged.requestBlocking(connectionToBlock);
    impl->notifyKinematicStateChange(requestFK, requestVelFK, requestAccFK, true);
}
void AssemblerBodyItem::notifyKinematicStateChangeLater(bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->notifyKinematicStateChange(requestFK, requestVelFK,requestAccFK, false);
}
void AssemblerBodyItem::notifyKinematicStateChangeLater(Connection& connectionToBlock, bool requestFK, bool requestVelFK, bool requestAccFK)
{
    impl->sigKinematicStateChanged.requestBlocking(connectionToBlock);
    impl->notifyKinematicStateChange(requestFK, requestVelFK, requestAccFK, false);
}
void AssemblerBodyItem::notifyKinematicStateUpdate(bool doNotifyStateChange)
{
    if(doNotifyStateChange){
        impl->notifyKinematicStateChange(false, false, false, true);
    }
    impl->sigKinematicStateUpdated();

    //if(isAttachedToParentBody_){
    //impl->parentAssemblerBodyItem->notifyKinematicStateUpdate(false);
    //}

    //auto record = new KinematicStateRecord(impl, impl->lastEditState);
    //UnifiedEditHistory::instance()->addRecord(record);
    //storeKinematicState(impl->lastEditState);
}

void AssemblerBodyItem::Impl::notifyKinematicStateChange(bool requestFK, bool requestVelFK, bool requestAccFK, bool isDirect)
{
#if 0
    updateElements.reset();

    if(isProcessingInverseKinematicsIncludingParentBody){
        isProcessingInverseKinematicsIncludingParentBody = false;
        if(parentAssemblerBodyItem){
            parentAssemblerBodyItem->impl->notifyKinematicStateChange(
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

void AssemblerBodyItem::Impl::emitSigKinematicStateChanged()
{
#if 0
    if(isFkRequested){
        fkTraverse.calcForwardKinematics(isVelFkRequested, isAccFkRequested);
        isFkRequested = isVelFkRequested = isAccFkRequested = false;
    }
    if(isKinematicStateChangeNotifiedByParentAssemblerBodyItem){
        isKinematicStateChangeNotifiedByParentAssemblerBodyItem = false;
    } else {
        if(parentAssemblerBodyItem && !attachmentToParent){
            setRelativeOffsetPositionFromParentBody();
        }
    }
#endif
    sigKinematicStateChanged.signal()();
}
#if 0
LocationProxyPtr AssemblerBodyItem::getLocationProxy()
{
    if(!impl->bodyLocation){
        impl->bodyLocation = new BodyLocation(impl);
    }
    return impl->bodyLocation;
}
LocationProxyPtr AssemblerBodyItem::createLinkLocationProxy(Link* link)
{
    return new LinkLocation(this, link);
}
#endif

#if 0
namespace {
BodyLocation::BodyLocation(AssemblerBodyItem::Impl* impl)
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
        impl->parentAssemblerBodyItem->notifyKinematicStateChange(true);
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
    if(impl->parentAssemblerBodyItem){
        if(impl->attachmentToParent){
            if(!impl->parentLinkLocation){
                impl->parentLinkLocation = new LinkLocation;
            }
            auto parentLink = impl->body->parentBodyLink();
            impl->parentLinkLocation->setTarget(impl->parentAssemblerBodyItem, parentLink);
            return impl->parentLinkLocation;
        } else {
            return impl->parentAssemblerBodyItem->getLocationProxy();
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
LinkLocation::LinkLocation(AssemblerBodyItem* bodyItem, Link* link)
    : LocationProxy(GlobalLocation),
      refAssemblerBodyItem(bodyItem),
      refLink(link)
{

}
void LinkLocation::setTarget(AssemblerBodyItem* bodyItem, Link* link)
{
    refAssemblerBodyItem = bodyItem;
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
    return refAssemblerBodyItem.lock();
}
LocationProxyPtr LinkLocation::getParentLocationProxy() const
{
    if(auto body = refAssemblerBodyItem.lock()){
        body->getLocationProxy();
    }
    return nullptr;
}
SignalProxy<void()> LinkLocation::sigLocationChanged()
{
    if(auto bodyItem = refAssemblerBodyItem.lock()){
        return bodyItem->sigKinematicStateChanged();
    } else {
        static Signal<void()> dummySignal;
        return dummySignal;
    }
}
}
#endif

////
AssemblerSceneBody* AssemblerBodyItem::sceneBody()
{
    if(!impl->sceneBody){
        impl->createSceneBody();
    }
    return impl->sceneBody;
}
SgNode* AssemblerBodyItem::getScene()
{
    return sceneBody();
}
void AssemblerBodyItem::Impl::createSceneBody()
{
    sceneBody = new AssemblerSceneBody(self);
    sceneBody->setSceneDeviceUpdateConnection(true);
    if(transparency > 0.0f){
        sceneBody->setTransparency(transparency);
    }
}

//// transp
float AssemblerBodyItem::transparency() const
{
    return impl->transparency;
}
void AssemblerBodyItem::setTransparency(float t)
{
    impl->setTransparency(t);
}
void AssemblerBodyItem::Impl::setTransparency(float t)
{
    if(t != transparency){
        if(sceneBody){
            sceneBody->setTransparency(t);
        }
        transparency = t;
    }
}
