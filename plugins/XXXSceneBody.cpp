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
    SceneMarkerPtr sMarker;

    void test(bool on) {
        std::cerr << "test: " << on << std::endl;
        if (on) {
            //sphereMarker = new SphereMarker(0.05, Vector3f(0.0f, 1.0f, 0.0f), 0.3);
            //sphereMarker->addChild(new CrossMarker(0.05 * 2.5, Vector3f(0.0f, 1.0f, 0.0f), 2.0f));
            //sphereMarker->setName("HOGEEE");
            //markerGroup->addChildOnce(sphereMarker, update);
            sMarker = new SceneMarker();
            sMarker->setMarkerType(SceneMarker::AXES_MARKER);
            sMarker->setMarkerSize(0.5);
            sMarker->updateMarker(false);
            markerGroup->addChildOnce(sMarker, update);
        } else {
            //markerGroup->removeChild(sphereMarker, update);
            //sphereMarker.reset();
            markerGroup->removeChild(sMarker, update);
            sMarker.reset();
        }
    }
    void move() {
        Position p = Position::Identity();
        p.translation() = Vector3(0, 0, 1.0);

        self->body()->rootLink()->setPosition(p);
        self->body()->calcForwardKinematics();

        self->updateSceneModel(); // required for applying the movement in Display
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
    SceneWidgetEvent::EventType tp = event->type();
    std::cerr << "Type: " << tp << std::endl;
    switch(tp) {
    case SceneWidgetEvent::ButtonPress:
        int bt = event->button();
        switch(bt) {
        case Qt::LeftButton:
            std::cerr << "Left" << std::endl;
            break;
        case Qt::RightButton:
            std::cerr << "Right" << std::endl;
            break;
        case Qt::MiddleButton:
            std::cerr << "Middle" << std::endl;
            break;
        }
        break;
    }
    SgNodePath pt = event->nodePath();
    std::cerr << "event->nodePath() : " << pt.size() << std::endl;
    for (int i = 0 ; i < pt.size(); i++) {
        SgNode *ptr = pt[i];
        std::cerr << "---" << std::endl;
        std::cerr << static_cast<void *> (ptr) << std::endl;
        std::cerr << "name: " << ptr->name() << std::endl;
        std::cerr << "class: " << ptr->className() << std::endl;
        std::cerr << "attr: " << ptr->attributes() << std::endl;
        if (ptr->hasUri()) {
            std::cerr << "uri: " << ptr->uri() << std::endl;
        }
        if (ptr->hasAbsoluteUri()) {
            std::cerr << "abs_uri: " << ptr->absoluteUri() << std::endl;
        }
        if (ptr->hasParents()) {
            int j = 0;
            for(auto it = ptr->parentBegin(); it != ptr->parentEnd(); it++, j++) {
                std::cerr << "p" << j << " : " << static_cast<void *>(*it) << std::endl;
            }
        }
    }
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
