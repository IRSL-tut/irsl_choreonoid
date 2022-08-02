
#include "RobotAssemblerHelper.h"

#include <cnoid/SceneLoader>
#include <cnoid/MeshGenerator>

#include <iostream>

#define IRSL_DEBUG
#include "irsl_debug.h"

namespace cnoid {
namespace robot_assembler {

static void createShapeConnectingPoint(SgPosTransform *_root, SgMaterialPtr &_res_material,
                                       SgSwitchableGroupPtr &_res_switch)
{
    // create shape
    SgShapePtr shape(new SgShape());
    MeshGenerator mg;
    {
        SgMeshPtr mesh = mg.generateBox(Vector3(0.02, 0.005, 0.005));
        shape->setMesh(mesh);
    }
    {
        SgMeshPtr mesh = mg.generateBox(Vector3(0.005, 0.02, 0.005));
        shape->setMesh(mesh);
    }
    {
        SgMeshPtr mesh = mg.generateBox(Vector3(0.005, 0.005, 0.02));
        shape->setMesh(mesh);
    }
    // material ???
    SgMaterialPtr material(new SgMaterial());
    material->setName("material");
    //Vector3f color(0.1f, 0.1f, 0.7f);
    material->setDiffuseColor(Vector3f(0.05f, 0.05f, 0.3f));
    material->setEmissiveColor(Vector3f(0.05f, 0.05f, 0.4f));
    material->setAmbientIntensity(0.0f);
    shape->setMaterial(material);

    // switch
    SgSwitchableGroupPtr sw_g(new SgSwitchableGroup());
    //shape->addChild(sw_node);//
    //_root->addChild(sw_node);//
    sw_g->addChild(shape);
    _root->addChild(sw_g);

    _res_material = material;
    _res_switch = sw_g;
}
static void createSceneFromGeometry(SgPosTransform *sg_main, std::vector<Geometry> &geom_list)
{
    DEBUG_STREAM("createSceneFromGeomatry" << std::endl);
    if (geom_list.size() <= 0) {
        //
        return;
    }
    for(int i = 0; i < geom_list.size(); i++) {
        Geometry &geom = geom_list[i];

        if (geom.type == Geometry::Mesh) {
            SceneLoader sceneLoader;
            sceneLoader.setMessageSink(std::cerr);
            DEBUG_STREAM("createSceneFromGeomatry: mesh load: " << geom.url << std::endl);
            SgNodePtr shape = sceneLoader.load(geom.url);
            if (!!shape) {
                DEBUG_STREAM("createSceneFromGeomatry: mesh loaded" << std::endl);
                Position p; geom.coords.toPosition(p);
                SgPosTransformPtr trs(new SgPosTransform(p));
                trs->setName("sg_trs");
                trs->addChild(shape);
                sg_main->addChild(trs);
            }
        } else if (geom.type == Geometry::Box) {
            // parameter
            MeshGenerator mg;
            SgMeshPtr mesh = mg.generateBox(Vector3(geom.parameter[0], geom.parameter[1], geom.parameter[2]));

            SgShapePtr shape(new SgShape());
            shape->setMesh(mesh);
            //shape->setName("box");
            // material
            if (!!shape) {
                SgMaterialPtr material(new SgMaterial());
                material->setName("material");
                //Vector3f color(0.1f, 0.1f, 0.7f);
                material->setDiffuseColor(Vector3f(0.05f, 0.05f, 0.3f));
                material->setEmissiveColor(Vector3f(0.05f, 0.05f, 0.4f));
                material->setAmbientIntensity(0.0f);
                shape->setMaterial(material);

                Position p; geom.coords.toPosition(p);
                SgPosTransformPtr trs(new SgPosTransform(p));
                trs->setName("sg_trs");
                trs->addChild(shape);
                sg_main->addChild(trs);
            }
        }
    }
}
RASceneParts::RASceneParts(RoboasmPartsPtr _p)
    : SgPosTransform(), self(_p), partsScene(nullptr)
{
    createSceneFromGeometry(this, self->info->visual);
    //partsScene = node;

    coordsPtrList lst;
    _p->directDescendants(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        RoboasmConnectingPointPtr ptr = dynamic_pointer_cast<RoboasmConnectingPoint>(*it);
        if(!!ptr) {
            RASceneConnectingPoint *cp = new RASceneConnectingPoint(ptr);
            this->addChild(cp);
            point_list.push_back(cp);
        }
    }
}
RASceneParts::RASceneConnectingPoint::RASceneConnectingPoint(RoboasmConnectingPointPtr _c)
    : SgPosTransform(), self(_c)
{
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
RASceneRobot::RASceneRobot(RoboasmRobotPtr _r)
    : SgPosTransform(), self(_r)
{
    partsPtrList lst;
    self->allParts(lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        RASceneParts *pt = new RASceneParts(*it);
        parts_list.push_back(pt);
        this->addChild(pt);
    }
}

} }
