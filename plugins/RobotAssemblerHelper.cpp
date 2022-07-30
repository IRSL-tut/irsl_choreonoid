
#include "RobotAssemblerHelper.h"

#include <cnoid/SceneLoader>
#include <cnoid/MeshGenerator>

#include <iostream>

#define IRSL_DEBUG
#include "irsl_debug.h"

namespace cnoid {
namespace robot_assembler {

SgNode *createSceneFromGeomatry(std::vector<Geometry> &geom_list)
{
    DEBUG_STREAM("createSceneFromGeomatry" << std::endl);
    if (geom_list.size() <= 0) {
        return nullptr;
    }

    SgPosTransform *sg_main = new SgPosTransform();
    sg_main->setName("sg_main");
    for(int i = 0; i < geom_list.size(); i++) {
        Geometry &geom = geom_list[i];

        if (geom.type == Geometry::Mesh) {
            SceneLoader sceneLoader;
            sceneLoader.setMessageSink(std::cerr);
            DEBUG_STREAM("createSceneFromGeomatry: mesh load: " << geom.url << std::endl);
            SgNode* shape = sceneLoader.load(geom.url);
            if (!!shape) {
                DEBUG_STREAM("createSceneFromGeomatry: mesh loaded" << std::endl);
                Position p; geom.coords.toPosition(p);
                SgPosTransform *trs = new SgPosTransform(p);
                trs->setName("sg_trs");
                trs->addChild(shape);
                sg_main->addChild(trs);
            }
        } else if (geom.type == Geometry::Box) {
            // parameter
            MeshGenerator mg;
            SgMeshPtr mesh = mg.generateBox(Vector3(geom.parameter[0], geom.parameter[1], geom.parameter[2]));

            SgShape* shape = new SgShape;
            shape->setMesh(mesh);
            //shape->setName("box");
            // material
            SgMaterial* material = new SgMaterial;
            material->setName("material");
            //Vector3f color(0.1f, 0.1f, 0.7f);
            material->setDiffuseColor(Vector3f(0.05f, 0.05f, 0.3f));
            material->setEmissiveColor(Vector3f(0.05f, 0.05f, 0.4f));
            material->setAmbientIntensity(0.0f);

            shape->setMaterial(material);

            if (!!shape) {
                Position p; geom.coords.toPosition(p);
                SgPosTransform *trs = new SgPosTransform(p);
                trs->setName("sg_trs");
                trs->addChild(shape);
                sg_main->addChild(trs);
            }
        }
    }

    return sg_main;
}

} }
