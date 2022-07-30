#include "RobotAssembler.h"

#include <cnoid/SceneGraph>


#pragma once

namespace cnoid {
namespace robot_assembler {

SgNode *createSceneFromGeomatry(std::vector<Geometry> &geom_list);

} }
