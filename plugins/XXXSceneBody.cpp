#include "XXXSceneBody.h"
#include "XXXBodyItem.h"
#include <cnoid/ExtensionManager>
#include <cnoid/SceneMarkers>
#include <cnoid/MenuManager>

#include <iostream> // debug

using namespace cnoid;

class XXXSceneLink::Impl {
public:
    XXXSceneLink *self;
    SgUpdate& update; //???

    Impl(XXXSceneBody* sceneBody, XXXSceneLink* link);
    ~Impl();
};

class XXXSceneBody::Impl {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Impl(XXXSceneBody* self, XXXBodyItem* bodyItem);
    ~Impl();

    XXXSceneLink* xxxSceneLink(int index) {
        return static_cast<XXXSceneLink*>(self->sceneLink(index));
    }

    SgUpdate update;

    XXXSceneBody *self;
    XXXBodyItemPtr bodyItem;

    SgGroupPtr markerGroup;
    CrossMarkerPtr crossMarker;
    SphereMarkerPtr sphereMarker;
#if 0
    zmpMarker = new SphereMarker(radius, Vector3f(0.0f, 1.0f, 0.0f), 0.3);
    zmpMarker->addChild(new CrossMarker(radius * 2.5, Vector3f(0.0f, 1.0f, 0.0f), 2.0f));
    zmpMarker->setName("ZMP");

    markerGroup->addChildOnce(virtualElasticStringLine, update);
    markerGroup->removeChild(virtualElasticStringLine, update);
    virtualElasticStringLine->notifyUpdate(update.withAction(SgUpdate::Modified));
#endif

    void test(bool on) {
        std::cerr << "test: " << on << std::endl;
        if (on) {
            sphereMarker = new SphereMarker(0.05, Vector3f(0.0f, 1.0f, 0.0f), 0.3);
            sphereMarker->addChild(new CrossMarker(0.05 * 2.5, Vector3f(0.0f, 1.0f, 0.0f), 2.0f));
            sphereMarker->setName("HOGEEE");

            markerGroup->addChildOnce(sphereMarker, update);
        } else {
            //sphereMarker = new SphereMarker(0.05, Vector3f(0.0f, 1.0f, 0.0f), 0.3);
            //shpereMarker->addChild(new CrossMarker(0.05 * 2.5, Vector3f(0.0f, 1.0f, 0.0f), 2.0f));
            //sphereMarker->setName("HOGEEE");
            markerGroup->removeChild(sphereMarker, update);
            sphereMarker.reset();
        }
    }
    void move() {
        Position p = Position::Identity();
        p.translation() = Vector3(0, 0, 1.0);

        self->body()->rootLink()->setPosition(p);
        self->body()->calcForwardKinematics();
        //
        self->notifyUpdate(); // scenebody
        bodyItem->notifyUpdate(); // move?? 1 // bodyItem
        //
        bodyItem->notifyKinematicStateChange();
        self->updateSceneModel(); // move?? 2
        self->updateSceneDeviceModels(true);
    }
};

XXXSceneLink::XXXSceneLink(XXXSceneBody* sceneBody, Link* link)
    : SceneLink(sceneBody, link)
{
    impl = new Impl(sceneBody, this);
}
XXXSceneLink::~XXXSceneLink()
{
    delete impl;
}

XXXSceneLink::Impl::Impl(XXXSceneBody* sceneBody, XXXSceneLink* link)
    : self(link), update(sceneBody->impl->update)
{

}
XXXSceneLink::Impl::~Impl()
{

}

/// XXXSceneBody
void XXXSceneBody::initializeClass(ExtensionManager* ext) {
//    ext->setProjectArchiver(
//        "EditableSceneBody",
//        EditableSceneBody::Impl::storeProperties,
//        EditableSceneBody::Impl::restoreProperties);
}

XXXSceneBody::XXXSceneBody(XXXBodyItem* bodyItem)
{
    impl = new Impl(this, bodyItem);
    //impl->initialize();
}

XXXBodyItem* XXXSceneBody::bodyItem() {
    return impl->bodyItem;
}

//XXXSceneLink* XXXSceneBody::xxxSceneLink(int index) {
//    //return static_cast<XXXSceneLink*>(this->sceneLink(index));
//    return impl->XXXSceneLink(index);
//}

void XXXSceneBody::setLinkVisibilities(const std::vector<bool>& visibilities) {

}

void XXXSceneBody::updateSceneModel() {
    this->SceneBody::updateSceneModel();
}

XXXSceneBody::~XXXSceneBody()
{
    delete impl;
}

XXXSceneBody::Impl::Impl(XXXSceneBody* _self, XXXBodyItem* _bodyItem)
    : self(_self),
      bodyItem(_bodyItem)
{
    // initialize
    self->setBody(bodyItem->body(), [this](Link* link){ return new XXXSceneLink(self, link); });

    self->setName("XXX:" + self->name());
    markerGroup = new SgGroup;
    markerGroup->setName("Marker");
    self->addChild(markerGroup);

    //self->sigGraphConnection().connect([&](bool on){ onSceneGraphConnection(on); });
}

//// overrides
void XXXSceneBody::onSceneModeChanged(SceneWidgetEvent* event)
{
    std::cerr << "XXXSceneBody(onSceneModeChanged)" << std::endl;
}
bool XXXSceneBody::onButtonPressEvent(SceneWidgetEvent* event)
{
    std::cerr << "XXXSceneBody(onButtonPressEvent)" << std::endl;
    //return true;
    return false;
    //if return true, handling events after this function may not occurred
}
bool XXXSceneBody::onDoubleClickEvent(SceneWidgetEvent* event)
{
    std::cerr << "XXXSceneBody(onDoubleClickEvent)" << std::endl;
    return false;
}
bool XXXSceneBody::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    std::cerr << "XXXSceneBody(onButtonReleaseEvent)" << std::endl;
    return false;
}
bool XXXSceneBody::onPointerMoveEvent(SceneWidgetEvent* event)
{
    std::cerr << "XXXSceneBody(onPointerMoveEvent)" << std::endl;
    return false;
}
void XXXSceneBody::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    std::cerr << "XXXSceneBody(onPointerLeaveEvent)" << std::endl;
}
bool XXXSceneBody::onKeyPressEvent(SceneWidgetEvent* event)
{
    std::cerr << "XXXSceneBody(onKeyPressEvent)" << std::endl;
    return false;
}
bool XXXSceneBody::onKeyReleaseEvent(SceneWidgetEvent* event)
{
    std::cerr << "XXXSceneBody(onKeyReleaseEvent)" << std::endl;
    return false;
}
bool XXXSceneBody::onScrollEvent(SceneWidgetEvent* event)
{
    std::cerr << "XXXSceneBody(onScrollEvent)" << std::endl;
    return false;
}
void XXXSceneBody::onFocusChanged(SceneWidgetEvent* event, bool on)
{
    std::cerr << "XXXSceneBody(onFocusChanged)" << std::endl;
}
bool XXXSceneBody::onContextMenuRequest(SceneWidgetEvent* event)
{
    std::cerr << "XXXSceneBody(onContextMenuRequest)" << std::endl;
    auto menu = event->contextMenu();

    menu->addItem("Clear")->sigTriggered().connect(
        [&](){ impl->test(false); });

    menu->addSeparator();

    menu->addItem("move")->sigTriggered().connect(
        [&](){ impl->move(); });

    menu->setPath("HOGE");

    menu->addItem("ONNNNNN")->sigTriggered().connect(
        [&](){ impl->test(true); });

    menu->setPath("/");
    menu->addSeparator();

    return false; //??
}
