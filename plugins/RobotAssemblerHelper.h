#include "RobotAssembler.h"

#include <cnoid/SceneGraph>
#include <cnoid/SceneWidgetEventHandler>
#include <cnoid/Body>
#include <set>

#pragma once

// temp
#define IRSL_DEBUG
#include "irsl_debug.h"

namespace cnoid {
namespace robot_assembler {

class RASceneConnectingPoint;
class RASceneParts;
class RASceneRobot;

class RASceneConnectingPoint : public SgPosTransform
{
public:
    enum Clicked {
        DEFAULT,
        SELECT_GOOD0,
        SELECT_GOOD1,
        SELECT_BAD0,
        SELECT_BAD1,
        CAN_CONNECT0,
        CAN_CONNECT1,
        CAN_CONNECT,
        SELECTED,
        NOT_CONNECT
    };
    RASceneConnectingPoint() = delete;
    RASceneConnectingPoint(RoboasmConnectingPointPtr _c);
    ~RASceneConnectingPoint();
    //~RASceneConnectingPoint();
    RoboasmConnectingPointPtr point() { return self; }
    void switchOn(bool on) {  if (!!switch_node) switch_node->setTurnedOn(on); }
    void changeState(Clicked _clk = DEFAULT);
    RASceneRobot *scene_robot() { return robot_ptr; }
protected:
    RASceneRobot *robot_ptr;
    RoboasmConnectingPointPtr self;
    SgMaterialPtr material;
    SgSwitchableGroupPtr switch_node;
    Clicked current_state;

    friend RASceneParts;
    friend RASceneRobot;
};
typedef ref_ptr<RASceneConnectingPoint> RASceneConnectingPointPtr;

class RASceneParts : public SgPosTransform
{
public:
    RASceneParts() = delete;
    RASceneParts(RoboasmPartsPtr _p);
    ~RASceneParts();
    //~RASceneParts();
    RoboasmPartsPtr parts() { return self; }
    RASceneRobot *scene_robot() { return robot_ptr; }

protected:
    RASceneRobot *robot_ptr;
    RoboasmPartsPtr self;
    SgNodePtr partsScene;

    std::vector<RASceneConnectingPoint*> spoint_list;

    friend RASceneConnectingPoint;
    friend RASceneRobot;
};
typedef ref_ptr<RASceneParts> RAScenePartsPtr;

class RASceneRobot : public SgPosTransform, public SceneWidgetEventHandler
{
public:
    RASceneRobot() = delete;
    RASceneRobot(RoboasmRobotPtr _r);
    ~RASceneRobot();

    RoboasmRobotPtr robot() { return self; }
    //bool addParts(RASceneParts *pt) {};
    bool mergeRobot(RASceneRobot *_rb) {
        coordinates base_coords = self->worldcoords();
        coordinates trans;
        Position p;
        _rb->clearChildren();
        for(auto pt_it = _rb->sparts_set.begin(); pt_it != _rb->sparts_set.end(); pt_it++) {
            this->addChild(*pt_it);
            base_coords.transformation(trans, (*pt_it)->parts()->worldcoords());
            trans.toPosition(p);
            (*pt_it)->position() = p;
            sparts_set.insert(*pt_it);
            (*pt_it)->robot_ptr = this;
            for(auto pit = (*pt_it)->spoint_list.begin(); pit != (*pt_it)->spoint_list.end(); pit++) {
                (*pit)->robot_ptr = this;
                spoint_set.insert(*pit);
            }
        }
    }

    void setCoords(coordinates &_coords) {
        self->newcoords(_coords);
        self->updateDescendants();
        Position p; _coords.toPosition(p);
        this->position() = p;
    }
    void updateFromSelf() {
        Position p; self->toPosition(p);
        this->position() = p;
    }

    void debug() {
        partsPtrList plst;
        self->allParts(plst);
        if(plst.size() != sparts_set.size()) {
            DEBUG_STREAM_NL(" Roboasm: " << plst.size() << " != Scene: " << sparts_set.size() << std::endl);
        }
        for(auto it = plst.begin(); it != plst.end(); it++) {
            bool exist = false;
            coordinates scoords;
            for(auto sit = sparts_set.begin(); sit != sparts_set.end(); sit++) {
                if((*it) == (*sit)->parts()) {
                    scoords = (*sit)->position();
                    exist = true;
                    break;
                }
            }
            if(!exist) {
                DEBUG_STREAM_NL(" Roboasm: " << (*it)->name() << " not in scene" << std::endl);
            } else {
                coordinates rcoords;
                self->worldcoords().transformation(rcoords, (*it)->worldcoords());
                if(!rcoords.equal(scoords)) {
                    DEBUG_STREAM_NL(" coords not equal r: " << (*it)->name() << std::endl);
                    //std::cout << "r: " << rcoords << ", s: " << scoords << std::endl;
                }
            }
        }
        for(auto sit = sparts_set.begin(); sit != sparts_set.end(); sit++) {
            bool exist = false;
            for(auto it = plst.begin(); it != plst.end(); it++) {
                if((*it) == (*sit)->parts()) {
                    exist = true;
                    break;
                }
            }
            if(!exist) {
                DEBUG_STREAM_NL(" Scene: " << (*sit)->name() << " not in roboasm" << std::endl);
            }
        }
    }
    // removeParts [TODO]

    //// Signals
    SignalProxy<int(RASceneConnectingPoint *_cp)> sigPointClicked() { return sigPointClickedFunc; }
    //int notifyPointClicked(RoboasmConnectingPoint *_cp);
    SignalProxy<int(RASceneParts *_pt)> sigPartsClicked() { return sigPartsClickedFunc; }
    //int notifyPartsClicked(RASceneParts *_pt);

    SignalProxy<void(RASceneRobot *_rb)> sigDeleteRobot() { return sigDeleteRobotFunc; }
    SignalProxy<void()> sigAttach() { return sigAttachFunc; }
    // SignalProxy<void(RASceneParts *_pt)> sigMoveRobot() { return sigPartsClickedFunc; }

    //// overrides : SceneWidgetEventHandler
    virtual void onSceneModeChanged(SceneWidgetEvent* event) override;
    virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onDoubleClickEvent(SceneWidgetEvent* event) override;
#if 0
    virtual bool onButtonReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    virtual void onPointerLeaveEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyPressEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onScrollEvent(SceneWidgetEvent* event) override;
#endif
    virtual void onFocusChanged(SceneWidgetEvent* event, bool on) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event) override;
    void attachHistory(AttachHistory &_hist,
                       const std::string &_parent,
                       const std::string &_robot_point,
                       const std::string &_parts_name,
                       const std::string &_parts_type,
                       const std::string &_parts_point,
                       coordinates &_coords)
    {
        _attachHistory(_hist, _parent, _robot_point, _parts_name, _parts_type, _parts_point);
        _hist[0].configuration.clear();
        _hist[0].config_coords = _coords;
        for(auto it = _hist.begin(); it != _hist.end(); it++) {
            history.push_back(*it);
        }
    }
    void attachHistory(AttachHistory &_hist,
                       const std::string &_parent,
                       const std::string &_robot_point,
                       const std::string &_parts_name,
                       const std::string &_parts_type,
                       const std::string &_parts_point,
                       const std::string &_config)
    {
        _attachHistory(_hist, _parent, _robot_point, _parts_name, _parts_type, _parts_point);
        _hist[0].configuration = _config;
        coordinates cds;
        _hist[0].config_coords = cds;
        for(auto it = _hist.begin(); it != _hist.end(); it++) {
            history.push_back(*it);
        }
    }
    // [todo]
    std::set<RASceneParts*> sparts_set;
    std::set<RASceneConnectingPoint*> spoint_set;
    AttachHistory history;

protected:
    bool _attachHistory(AttachHistory &_hist,
                        const std::string &_parent,
                        const std::string &_robot_point,
                        const std::string &_parts_name,
                        const std::string &_parts_type,
                        const std::string &_parts_point)
    {
        if(_hist.size() < 1) return false;
        // check _hist[0].parts_name == _parts_name
        _hist[0].parts_name = _parts_name;
        _hist[0].parts_type = _parts_type;
        _hist[0].parts_point = _parts_point;
        //_hist[0].configuration = _config;
        _hist[0].robot_parts_point = _robot_point;
        _hist[0].parent = _parent;
        _hist[0].initial_parts = false;
        _hist[0].inverse = false;

        return true;
    }
    RoboasmRobotPtr self;
    // create SgRAParts and add as child
    Signal<int(RASceneConnectingPoint *_cp)> sigPointClickedFunc;
    Signal<int(RASceneParts *_pt)> sigPartsClickedFunc;
    Signal<void(RASceneRobot *_rb)> sigDeleteRobotFunc;
    Signal<void()> sigAttachFunc;
    friend RASceneConnectingPoint;
    friend RASceneParts;
};
typedef ref_ptr<RASceneRobot> RASceneRobotPtr;

// SgPosTransform -> SgPosTransform(partsScene) -> SgShape (parts) -> material(???)
//                -> SgPosTransform -> SgShepe (connecting-point) -> SgMaterial (from list???
//                                                                -> SgSwitch
// notifyUpdate(SgUpdate::REMOVED | SgUpdate::ADDED | SgUpdate::MODIFIED); / on scene graph
 
} }
