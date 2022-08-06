#include "RobotAssemblerHelper.h"
#include "RobotAssemblerBody.h"
// shape
#include <cnoid/SceneLoader>
#include <cnoid/MeshGenerator>
// context menu
#include <cnoid/MenuManager>

#include <iostream>

#define IRSL_DEBUG
#include "irsl_debug.h"

namespace cnoid {
namespace robot_assembler {

static const Vector3f color_default(0.3f, 0.3f, 0.6f);
static const Vector3f color_good0(0.0f, 1.0f, 0.0f);
static const Vector3f color_good1(0.0f, 1.0f, 0.0f);
static const Vector3f color_bad0(1.0f, 0.0f, 0.0f);
static const Vector3f color_bad1(1.0f, 0.0f, 0.0f);
static const Vector3f color_can_connect0(0.0f, 1.0f, 1.0f);
static const Vector3f color_can_connect1(0.0f, 1.0f, 1.0f);
static const Vector3f color_selected(0.5f, 0.0f, 0.5f);

static void createShapeConnectingPoint(SgPosTransform *_root, SgMaterialPtr &_res_material,
                                       SgSwitchableGroupPtr &_res_switch)
{
    // create shape
    const std::string &name_ = _root->name();
    // material ???
    SgMaterialPtr material(new SgMaterial());
    material->setName(name_ + "/common_material");

    material->setDiffuseColor(color_default);
    material->setEmissiveColor(Vector3f(0.0f, 0.0f, 0.0f));
    material->setSpecularColor(Vector3f(0.0f, 0.0f, 0.0f));
    material->setAmbientIntensity(0.7f);

    SgSwitchableGroupPtr sw_g(new SgSwitchableGroup());
    sw_g->setName(name_ + "/switch");
    MeshGenerator mg;
    {
        SgShapePtr shape(new SgShape());
        shape->setName(name_ + "/x");
        SgMeshPtr mesh = mg.generateBox(Vector3(0.02, 0.005, 0.005));
        shape->setMesh(mesh);
        shape->setMaterial(material);
        sw_g->addChild(shape);
    }
    {
        SgShapePtr shape(new SgShape());
        shape->setName(name_ + "/y");
        SgMeshPtr mesh = mg.generateBox(Vector3(0.005, 0.02, 0.005));
        shape->setMesh(mesh);
        shape->setMaterial(material);
        sw_g->addChild(shape);
    }
    {
        SgShapePtr shape(new SgShape());
        shape->setName(name_ + "/z");
        SgMeshPtr mesh = mg.generateBox(Vector3(0.005, 0.005, 0.02));
        shape->setMesh(mesh);
        shape->setMaterial(material);
        sw_g->addChild(shape);
    }
    _root->addChild(sw_g);

    _res_material = material;
    _res_switch = sw_g;
}

RASceneConnectingPoint::RASceneConnectingPoint(RoboasmConnectingPointPtr _c)
    : SgPosTransform(), self(_c), current_state(DEFAULT)
{
    setName("CP:" + self->name());
    SgMaterialPtr mat_;//todo
    SgSwitchableGroupPtr sw_;//todo
    createShapeConnectingPoint(this, mat_, sw_);

    coordinates *cds = static_cast<coordinates *>(self.get());
    Position p;
    cds->toPosition(p);
    position() = p;

    material = mat_;
    switch_node = sw_;
}
RASceneConnectingPoint::~RASceneConnectingPoint()
{
    DEBUG_STREAM_NL(self->name() << std::endl);
}
void RASceneConnectingPoint::changeState(RASceneConnectingPoint::Clicked _clk)
{
    if (current_state == _clk) return;
    current_state = _clk;
    switch(_clk) {
    case DEFAULT:
    {
        material->setDiffuseColor(color_default);
    }
    break;
    case SELECT_GOOD0:
    {
        material->setDiffuseColor(color_good0);
    }
    break;
    case SELECT_GOOD1:
    {
        material->setDiffuseColor(color_good1);
    }
    break;
    case SELECT_BAD0:
    {
        material->setDiffuseColor(color_bad0);
    }
    break;
    case SELECT_BAD1:
    {
        material->setDiffuseColor(color_bad1);
    }
    break;
    case CAN_CONNECT0:
    {
        material->setDiffuseColor(color_can_connect0);
    }
    break;
    case CAN_CONNECT1:
    {
        material->setDiffuseColor(color_can_connect1);
    }
    break;
    }
}
RASceneParts::RASceneParts(RoboasmPartsPtr _p)
    : SgPosTransform(), self(_p), partsScene(nullptr)
{
    setName("PT:" + self->name());
    createSceneFromGeometry(this, self->info->visual);
    //partsScene = node;

    coordsPtrList lst;
    _p->directDescendants(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        RoboasmConnectingPointPtr ptr = dynamic_pointer_cast<RoboasmConnectingPoint>(*it);
        if(!!ptr) {
            RASceneConnectingPoint *cp = new RASceneConnectingPoint(ptr);
            this->addChild(cp);
            spoint_list.push_back(cp);
        }
    }
}
RASceneParts::~RASceneParts()
{
    DEBUG_STREAM_NL(self->name() << std::endl);
}
RASceneRobot::RASceneRobot(RoboasmRobotPtr _r)
    : SgPosTransform(), self(_r)
{
    setName("RB:" + self->name());
    partsPtrList lst;
    self->allParts(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        RASceneParts *pt = new RASceneParts(*it);
        pt->robot_ptr = this;
        sparts_set.insert(pt);
        this->addChild(pt);
        for(int i = 0; i < pt->spoint_list.size(); i++) {
            pt->spoint_list[i]->robot_ptr = this;
            spoint_set.insert(pt->spoint_list[i]);
        }
    }
}
RASceneRobot::~RASceneRobot()
{
    DEBUG_STREAM_NL(self->name() << std::endl);
}
//// overrides : SceneWidgetEventHandler
void RASceneRobot::onSceneModeChanged(SceneWidgetEvent* event)
{
    // SgNode should have Operable
    DEBUG_STREAM_NL(std::endl);
}
bool RASceneRobot::onButtonPressEvent(SceneWidgetEvent* event)
{
    SceneWidgetEvent::EventType tp = event->type();
    DEBUG_STREAM_NL(" Type: " << tp << std::endl);
#if 0
    switch(tp) {
    case SceneWidgetEvent::ButtonPress:
        int bt = event->button();
        switch(bt) {
        case Qt::LeftButton:
            DEBUG_STREAM_NL(" Left" << std::endl);
            break;
        case Qt::RightButton:
            DEBUG_STREAM_NL(" Right" << std::endl);
            break;
        case Qt::MiddleButton:
            DEBUG_STREAM_NL(" Middle" << std::endl);
            break;
        }
        break;
    }
#endif
    SgNodePath pt = event->nodePath();
#if 0
    DEBUG_STREAM_NL(" event->nodePath() : " << pt.size() << std::endl);
    for (int i = 0 ; i < pt.size(); i++) {
        SgNode *ptr = pt[i];
        DEBUG_STREAM_NL(" ---" << std::endl);
        DEBUG_STREAM_NL(" " << static_cast<void *> (ptr) << std::endl);
        DEBUG_STREAM_NL(" name: " << ptr->name() << std::endl);
        DEBUG_STREAM_NL(" class: " << ptr->className() << std::endl);
        DEBUG_STREAM_NL(" attr: " << ptr->attributes() << std::endl);
        if (ptr->hasUri()) {
            DEBUG_STREAM_NL( " uri: " << ptr->uri() << std::endl);
        }
        if (ptr->hasAbsoluteUri()) {
            DEBUG_STREAM_NL( " abs_uri: " << ptr->absoluteUri() << std::endl);
        }
        if (ptr->hasParents()) {
            int j = 0;
            for(auto it = ptr->parentBegin(); it != ptr->parentEnd(); it++, j++) {
                DEBUG_STREAM_NL(" p" << j << " : " << static_cast<void *>(*it) << std::endl);
            }
        }
    }
#endif
    RASceneParts *pt_ = nullptr;
    RASceneConnectingPoint *cp_ = nullptr;
    for (int i = 0 ; i < pt.size(); i++) {
        SgNode *ptr = pt[i];
        if(!pt_) pt_ = dynamic_cast<RASceneParts *>(ptr);
        if(!cp_) cp_ = dynamic_cast<RASceneConnectingPoint *>(ptr);
        if(!!pt_ && !!cp_) break;
    }
    if(!!cp_) {
        DEBUG_STREAM_NL( " ConnectingPointClicked : " << cp_->name() << std::endl);
        sigPointClickedFunc(cp_);
    } else if (!!pt_) {
        DEBUG_STREAM_NL( " PartsClicked : " << pt_->name() << std::endl);
        sigPartsClickedFunc(pt_);
    } else {
        DEBUG_STREAM_NL( " ---unknown state---" << std::endl);
    }
    return false;
    //if return true, handling events after this function may not occurred
}
bool RASceneRobot::onDoubleClickEvent(SceneWidgetEvent* event)
{
    DEBUG_STREAM_NL(std::endl);
    // disable default behavior / default: double click -> toggle edit-mode
    return true;
}
#if 0
bool RASceneRobot::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    DEBUG_STREAM_NL(std::endl);
    return false;
}
bool RASceneRobot::onPointerMoveEvent(SceneWidgetEvent* event)
{
    DEBUG_STREAM_NL(std::endl);
    return false;
}
void RASceneRobot::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    DEBUG_STREAM_NL(std::endl);
}
bool RASceneRobot::onKeyPressEvent(SceneWidgetEvent* event)
{
    DEBUG_STREAM_NL(std::endl);
    return false;
}
bool RASceneRobot::onKeyReleaseEvent(SceneWidgetEvent* event)
{
    DEBUG_STREAM_NL(std::endl);
    return false;
}
bool RASceneRobot::onScrollEvent(SceneWidgetEvent* event)
{
    DEBUG_STREAM_NL(std::endl);
    return false;
}
#endif
void RASceneRobot::onFocusChanged(SceneWidgetEvent* event, bool on)
{
    // may call when mode was changed
    DEBUG_STREAM_NL(std::endl);
}
bool RASceneRobot::onContextMenuRequest(SceneWidgetEvent* event)
{
    DEBUG_STREAM_NL(std::endl);

    auto menu = event->contextMenu();

    menu->addItem("Move")->sigTriggered().connect(
        [this](){ } );

    menu->addSeparator();

    std::string label_ = "Delete This: ";
    label_ += this->name();
    menu->addItem(label_)->sigTriggered().connect(
        [this](){ sigDeleteRobotFunc(this); });

    menu->addSeparator();

    menu->addItem("Attach")->sigTriggered().connect(
        [this](){ sigAttachFunc(); } );

    return false; //??
}

} }
