#include "RobotAssembler.h"

#include <cnoid/SceneGraph>
#include <cnoid/SceneWidgetEventHandler>

// #include <set>

#pragma once

namespace cnoid {
namespace robot_assembler {

class RASceneParts : public SgPosTransform
{
public:
    RASceneParts() = delete;
    RASceneParts(RoboasmPartsPtr _p);
    //~RASceneParts();
protected:
    RoboasmPartsPtr self;
    SgNodePtr partsScene;

    class RASceneConnectingPoint : public SgPosTransform
    {
    public:
        RASceneConnectingPoint() = delete;
        RASceneConnectingPoint(RoboasmConnectingPointPtr _c);
        //~RASceneConnectingPoint();
        RoboasmConnectingPointPtr self;
        SgMaterialPtr material;
        SgSwitchableGroupPtr switch_node;
    };
    std::vector<RASceneConnectingPoint*> point_list;
};
//typedef std::shared_ptr<RASceneParts> RAScenePartsPtr;
typedef ref_ptr<RASceneParts> RAScenePartsPtr;

class RASceneRobot : public SgPosTransform, public SceneWidgetEventHandler
{
public:
    RASceneRobot() = delete;
    RASceneRobot(RoboasmRobotPtr _r);

    RoboasmRobotPtr self;

    // addParts
    // ---
    // update(selected_point, can_match);
    //
    // test
    //void initialCreate(RoboasmRobotPtr _r);
#if 0
    //// overrides
    virtual void onSceneModeChanged(SceneWidgetEvent* event) override;
    virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onDoubleClickEvent(SceneWidgetEvent* event) override;
    virtual bool onButtonReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    virtual void onPointerLeaveEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyPressEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onScrollEvent(SceneWidgetEvent* event) override;
    virtual void onFocusChanged(SceneWidgetEvent* event, bool on) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event) override;
#endif
protected:
    std::vector<RASceneParts*> parts_list;
    // create SgRAParts and add as child
};
// SgPosTransform -> SgPosTransform(partsScene) -> SgShape (parts) -> material(???)
//                -> SgPosTransform -> SgShepe (connecting-point) -> SgMaterial (from list???
//                                                                -> SgSwitch
// notifyUpdate(SgUpdate::REMOVED | SgUpdate::ADDED | SgUpdate::MODIFIED); / on scene graph

} }
