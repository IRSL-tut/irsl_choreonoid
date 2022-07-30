#include "AssemblerSceneBody.h"
#include "AssemblerBodyItem.h"
#include <cnoid/ExtensionManager>
#include <cnoid/SceneMarkers>
#include <cnoid/MenuManager>

#include <iostream> // debug

using namespace cnoid;

class AssemblerSceneLink::Impl {
public:
    AssemblerSceneLink *self;
    SgUpdate& update; //???

    Impl(AssemblerSceneBody* sceneBody, AssemblerSceneLink* link);
    ~Impl();
};

class AssemblerSceneBody::Impl {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Impl(AssemblerSceneBody* self, AssemblerBodyItem* bodyItem);
    ~Impl();

    AssemblerSceneLink* xxxSceneLink(int index) {
        return static_cast<AssemblerSceneLink*>(self->sceneLink(index));
    }

    SgUpdate update;

    AssemblerSceneBody *self;
    AssemblerBodyItemPtr bodyItem;

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

AssemblerSceneLink::AssemblerSceneLink(AssemblerSceneBody* sceneBody, Link* link)
    : SceneLink(sceneBody, link)
{
    impl = new Impl(sceneBody, this);
}
AssemblerSceneLink::~AssemblerSceneLink()
{
    delete impl;
}

AssemblerSceneLink::Impl::Impl(AssemblerSceneBody* sceneBody, AssemblerSceneLink* link)
    : self(link), update(sceneBody->impl->update)
{

}
AssemblerSceneLink::Impl::~Impl()
{

}

/// AssemblerSceneBody
void AssemblerSceneBody::initializeClass(ExtensionManager* ext) {
//    ext->setProjectArchiver(
//        "EditableSceneBody",
//        EditableSceneBody::Impl::storeProperties,
//        EditableSceneBody::Impl::restoreProperties);
}

AssemblerSceneBody::AssemblerSceneBody(AssemblerBodyItem* bodyItem)
{
    impl = new Impl(this, bodyItem);
    //impl->initialize();
}

AssemblerBodyItem* AssemblerSceneBody::bodyItem() {
    return impl->bodyItem;
}

//AssemblerSceneLink* AssemblerSceneBody::xxxSceneLink(int index) {
//    //return static_cast<AssemblerSceneLink*>(this->sceneLink(index));
//    return impl->AssemblerSceneLink(index);
//}

void AssemblerSceneBody::setLinkVisibilities(const std::vector<bool>& visibilities) {

}

void AssemblerSceneBody::updateSceneModel() {
    this->SceneBody::updateSceneModel();
}

AssemblerSceneBody::~AssemblerSceneBody()
{
    delete impl;
}

AssemblerSceneBody::Impl::Impl(AssemblerSceneBody* _self, AssemblerBodyItem* _bodyItem)
    : self(_self),
      bodyItem(_bodyItem)
{
    // initialize
    self->setBody(bodyItem->body(), [this](Link* link){ return new AssemblerSceneLink(self, link); });

    self->setName("Assembler:" + self->name());
    markerGroup = new SgGroup;
    markerGroup->setName("Marker");
    self->addChild(markerGroup);

    //self->sigGraphConnection().connect([&](bool on){ onSceneGraphConnection(on); });
}

//// overrides
void AssemblerSceneBody::onSceneModeChanged(SceneWidgetEvent* event)
{
    std::cerr << "AssemblerSceneBody(onSceneModeChanged)" << std::endl;
}
bool AssemblerSceneBody::onButtonPressEvent(SceneWidgetEvent* event)
{
    std::cerr << "AssemblerSceneBody(onButtonPressEvent)" << std::endl;
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
bool AssemblerSceneBody::onDoubleClickEvent(SceneWidgetEvent* event)
{
    std::cerr << "AssemblerSceneBody(onDoubleClickEvent)" << std::endl;
    return false;
}
bool AssemblerSceneBody::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    std::cerr << "AssemblerSceneBody(onButtonReleaseEvent)" << std::endl;
    return false;
}
bool AssemblerSceneBody::onPointerMoveEvent(SceneWidgetEvent* event)
{
    std::cerr << "AssemblerSceneBody(onPointerMoveEvent)" << std::endl;
    return false;
}
void AssemblerSceneBody::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    std::cerr << "AssemblerSceneBody(onPointerLeaveEvent)" << std::endl;
}
bool AssemblerSceneBody::onKeyPressEvent(SceneWidgetEvent* event)
{
    std::cerr << "AssemblerSceneBody(onKeyPressEvent)" << std::endl;
    return false;
}
bool AssemblerSceneBody::onKeyReleaseEvent(SceneWidgetEvent* event)
{
    std::cerr << "AssemblerSceneBody(onKeyReleaseEvent)" << std::endl;
    return false;
}
bool AssemblerSceneBody::onScrollEvent(SceneWidgetEvent* event)
{
    std::cerr << "AssemblerSceneBody(onScrollEvent)" << std::endl;
    return false;
}
void AssemblerSceneBody::onFocusChanged(SceneWidgetEvent* event, bool on)
{
    std::cerr << "AssemblerSceneBody(onFocusChanged)" << std::endl;
}
bool AssemblerSceneBody::onContextMenuRequest(SceneWidgetEvent* event)
{
    std::cerr << "AssemblerSceneBody(onContextMenuRequest)" << std::endl;
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
