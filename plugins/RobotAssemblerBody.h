#ifndef CNOID_ROBOT_ASSEMBLER_BODY_H
#define CNOID_ROBOT_ASSEMBLER_BODY_H

#include "RobotAssembler.h"
#include <cnoid/Body>

namespace cnoid {
namespace robot_assembler {

extern const Vector3f default_body_color;

void createSceneFromGeometry(SgPosTransform *sg_main, std::vector<Geometry> &geom_list);

class RoboasmBodyCreator
{
public:
    RoboasmBodyCreator() : joint_counter(0), merge_fixed_joint(false) {};
    RoboasmBodyCreator(const std::string &_name);

    void setMergeFixedJoint(bool on = true) { merge_fixed_joint = on; };
    BodyPtr createBody(RoboasmRobotPtr _rb, const std::string &_name = std::string());
protected:
    BodyPtr body;
    int joint_counter;
    std::string name;
    bool merge_fixed_joint;

    std::map<std::string, std::string> map_link_cnoid_roboasm;  // key: cnoid_name, value: roboasm_name
    std::map<std::string, std::string> map_joint_cnoid_roboasm; // key: cnoid_name, value: roboasm_name
    //std::vector<std::string> joint_list; // cnoid_name

    BodyPtr _createBody(RoboasmRobotPtr _rb);
    Link *createLink(RoboasmPartsPtr _pt, bool _is_root = false);
    bool appendChildLink(BodyPtr _bd, Link *_lk, RoboasmPartsPtr _pt);
    bool mergeFixedJoint(BodyPtr _bd);
};

} }
#endif
