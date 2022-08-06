#include "RobotAssemblerBody.h"

// shape
#include <cnoid/SceneLoader>
#include <cnoid/MeshGenerator>
#include <cnoid/CloneMap>

#define IRSL_DEBUG
#include "irsl_debug.h"

namespace cnoid {
namespace robot_assembler {

const Vector3f default_body_color(0.0f, 0.0f, 0.8f);

void createSceneFromGeometry(SgPosTransform *sg_main, std::vector<Geometry> &geom_list)
{
    DEBUG_STREAM_NL(std::endl);
    if (geom_list.size() <= 0) {
        //
        return;
    }
    const std::string &name_ = sg_main->name();
    for(int i = 0; i < geom_list.size(); i++) {
        Geometry &geom = geom_list[i];

        if (geom.type == Geometry::Mesh) {
            SceneLoader sceneLoader;
            sceneLoader.setMessageSink(std::cerr);
            DEBUG_STREAM_NL(" mesh load: " << geom.url << std::endl);
            SgNodePtr shape = sceneLoader.load(geom.url);
            if (!!shape) {
                shape->setName(name_ + "/geom");
                DEBUG_STREAM_NL(" mesh loaded!" << std::endl);
                Position p; geom.coords.toPosition(p);
                SgPosTransformPtr trs(new SgPosTransform(p));
                trs->setName(name_ + "/geom_postrans");
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
                shape->setName(name_ + "/box");
                SgMaterialPtr material(new SgMaterial());
                material->setName(name_ + "material");
                //Vector3f color(0.1f, 0.1f, 0.7f);
                material->setDiffuseColor(default_body_color);
                material->setEmissiveColor(Vector3f(0.0f, 0.0f, 0.0f));
                material->setAmbientIntensity(0.7f);
                shape->setMaterial(material);

                Position p; geom.coords.toPosition(p);
                SgPosTransformPtr trs(new SgPosTransform(p));
                trs->setName(name_ + "/geom_postrans");
                trs->addChild(shape);
                sg_main->addChild(trs);
            }
        }
    }
}

RoboasmBodyCreator::RoboasmBodyCreator(const std::string &_name) : RoboasmBodyCreator ()
{
    name = _name;
    joint_counter = 0;
}
Link *RoboasmBodyCreator::createLink(RoboasmPartsPtr _pt, bool _is_root)
{
    Link *lk = new Link();
    coordinates link_origin_to_self_;

    if (_is_root) { // for root link
        lk->setName("Root");
        //lk->setJointName();
        lk->setJointType(Link::FreeJoint);
        lk->setJointId(-1);
        // temp
        Position p;
        _pt->worldcoords().toPosition(p);
        lk->setOffsetPosition(p);
        //
        map_link_cnoid_roboasm.insert(std::pair<std::string, std::string>("Root", _pt->name()));
    } else { // usual (has parent) link
        RoboasmPartsPtr p_pt_;
        RoboasmConnectingPointPtr p_cp_;
        RoboasmConnectingPointPtr s_cp_;
        if(!_pt->parentParts(p_pt_, p_cp_, s_cp_)) {
            std::cerr << "miss fetch" << std::endl;
            delete lk;
            return nullptr;
        }
        s_cp_->worldcoords().transformation(link_origin_to_self_,
                                            _pt->worldcoords());
        if(!p_pt_->parent()) {
            // invalid
            delete lk;
            return nullptr;
        }
        coordinates cds;
        p_pt_->parent()->worldcoords().transformation(cds, s_cp_->worldcoords());
        Position p; cds.toPosition(p);
        lk->setOffsetPosition(p);
        if( p_cp_->isActuator() ) {
            std::ostringstream ss_;
            ss_ << "LINK_" << joint_counter;
            lk->setName(ss_.str());
            lk->setJointId(joint_counter++);
            //lk->setJointName();
            map_link_cnoid_roboasm.insert(std::pair<std::string, std::string>(ss_.str(), _pt->name()));
            Actuator *ainfo_ = dynamic_cast<Actuator*>(p_cp_->info);
            if(!!ainfo_) {
                lk->setJointAxis(ainfo_->axis);
                switch (ainfo_->getType()) {
                case ConnectingPoint::Rotational:
                    lk->setJointType(Link::RevoluteJoint);
                    break;
                case ConnectingPoint::Linear:
                    lk->setJointType(Link::PrismaticJoint);
                    break;
                case ConnectingPoint::Fixed: // ??
                    lk->setJointType(Link::RevoluteJoint);
                    lk->setJointRange(0, 0);
                    break;
                case ConnectingPoint::Free: // ??
                    lk->setJointType(Link::FreeJoint);
                    break;
                }
                if(ainfo_->limit[0] != 0.0 || ainfo_->limit[1] != 0.0) {
                    lk->setJointRange(ainfo_->limit[0], ainfo_->limit[1]);
                }
                if(ainfo_->vlimit[0] != 0.0 || ainfo_->vlimit[1] != 0.0) {
                    lk->setJointVelocityRange(ainfo_->vlimit[0], ainfo_->vlimit[1]);
                }
                if(ainfo_->tqlimit[0] != 0.0 || ainfo_->tqlimit[1] != 0.0) {
                    lk->setJointEffortRange(ainfo_->tqlimit[0], ainfo_->tqlimit[1]);
                }
            } else {
                // no ainfo
            }
        } else { // not actuator(fixed connect)
            lk->setName(_pt->name());
            lk->setJointType(Link::FixedJoint);
            lk->setJointId(-1);
            map_link_cnoid_roboasm.insert(std::pair<std::string, std::string>(_pt->name(), _pt->name()));
        }
    } // root or not root
    if (!!_pt->info && _pt->info->hasMassParam) {
        lk->setCenterOfMass(_pt->info->COM);
        lk->setMass(_pt->info->mass);
        lk->setInertia(_pt->info->inertia_tensor);
    }
    // shape
    MeshGenerator mg;
    // com / next point
    { // red parts(origin)
        SgMeshPtr mesh = mg.generateBox(Vector3(0.004, 0.004, 0.004));
        SgShapePtr shape(new SgShape());
        shape->setMesh(mesh);
        SgMaterialPtr material(new SgMaterial());
        material->setDiffuseColor(Vector3f(1.0f, 0.0f, 0.0f));
        material->setEmissiveColor(Vector3f(0.0f, 0.0f, 0.0f));
        material->setAmbientIntensity(0.7f);
        shape->setMaterial(material);
        Position p; link_origin_to_self_.toPosition(p);
        SgPosTransformPtr trs(new SgPosTransform(p));
        trs->addChild(shape);
        lk->addShapeNode(trs); //
    }
    if(!_is_root)
    { // green self_cp
        SgMeshPtr mesh = mg.generateBox(Vector3(0.004, 0.004, 0.004));
        SgShapePtr shape(new SgShape());
        shape->setMesh(mesh);
        SgMaterialPtr material(new SgMaterial());
        material->setDiffuseColor(Vector3f(0.0f, 1.0f, 0.0f));
        material->setEmissiveColor(Vector3f(0.0f, 0.0f, 0.0f));
        material->setAmbientIntensity(0.7f);
        shape->setMaterial(material);
        // link_origin
        SgPosTransformPtr trs(new SgPosTransform());
        trs->addChild(shape);
        lk->addShapeNode(trs); //
    }
    if(!!_pt->info) {
        SgPosTransformPtr trs_vis;
        SgPosTransformPtr trs_col;
        if(_pt->info->visual.size() > 0) {
            trs_vis = (new SgPosTransform());
            trs_vis->setName(_pt->name() + "/visual");
            createSceneFromGeometry(trs_vis, _pt->info->visual);
        }
        if(_pt->info->collision.size() > 0) {
            trs_col = (new SgPosTransform());
            trs_col->setName(_pt->name() + "/collision");
            createSceneFromGeometry(trs_col, _pt->info->collision);
        }
        if(!!trs_vis) {
            Position p; link_origin_to_self_.toPosition(p);
            trs_vis->position() = p;
        }
        if(!!trs_col) {
            Position p; link_origin_to_self_.toPosition(p);
            trs_col->position() = p;
        }
        if (!!trs_vis) {
            lk->addVisualShapeNode(trs_vis);
        } else if (!!trs_col) {
            lk->addVisualShapeNode(trs_col);
        }
        if (!!trs_col) {
            lk->addCollisionShapeNode(trs_col);
        } else if (!!trs_vis) {
            lk->addCollisionShapeNode(trs_vis);
        }
    }
    return lk;
}
bool RoboasmBodyCreator::appendChildLink(BodyPtr _bd, Link *_lk, RoboasmPartsPtr _pt)
{
    partsPtrList plst;
    _pt->childParts(plst);
    for (auto it = plst.begin(); it != plst.end(); it++) {
        Link *clk = createLink(*it);
        if(!!clk) {
            _lk->appendChild(clk);
            _bd->updateLinkTree();
            appendChildLink(_bd, clk, (*it));
        }
    }
}
BodyPtr RoboasmBodyCreator::_createBody(RoboasmRobotPtr _rb)
{
    _rb->updateDescendants();
    RoboasmPartsPtr root_ = _rb->rootParts();
    if(!root_) {
        ERROR_STREAM(" root not found");
        return nullptr;
    }

    body = new Body();
    if(name.size() > 0) {
        body->setName(name);
        body->setModelName(_rb->name());
    } else {
        body->setName(_rb->name());
        body->setModelName(_rb->name());
    }

    Link *lk = createLink(root_, true);
    body->setRootLink(lk);
    body->updateLinkTree();

    appendChildLink(body, lk, root_);

    return body;
}
BodyPtr RoboasmBodyCreator::createBody(RoboasmRobotPtr _rb, const std::string &_name)
{
    if(_name.size() > 0) name = _name;
    body = new Body();

    _createBody(_rb);

    if (merge_fixed_joint) {
        mergeFixedJoint(body);
    }

    return body;
}
static bool mergeLink(Link *plink, Link *clink)
{
    if(!plink) {
        std::cerr << "plink does not exist" << std::endl;
        return false;
    }
    if(!clink) {
        std::cerr << "clink does not exist" << std::endl;
        return false;
    }
    std::cerr << "p:" << plink->name() << " / " << "c:" << clink->name() << std::endl;
    LinkPtr link_protect(clink);
    std::vector<LinkPtr> all_child;
    {
        Link *cur = clink->child();
        while(!!cur) {
            all_child.push_back(cur);
            cur = cur->sibling();
        }
    }
    //std::cout << "child : " << all_child.size() << std::endl;
    // remove child
    if(!plink->removeChild(clink)) {
        std::cerr << "remove failed : " << clink->name() << std::endl;
        return false;
    }
    // update mass paramter of plink
    {
        double new_mass = plink->mass() + clink->mass();
        coordinates cds_Tb(clink->Tb());
        Vector3 p_c_c = clink->c();
        cds_Tb.transform_vector(p_c_c);
        Vector3 new_c = ((clink->mass() * p_c_c) + (plink->mass() * plink->c()))/new_mass;

        Matrix3 pIc = cds_Tb.rot * clink->I() * cds_Tb.rot.transpose();
        Matrix3 h_c = hat(new_c - p_c_c);
        Matrix3 h_p = hat(new_c - plink->c());

        Matrix3 newI = (pIc - clink->mass() * (h_c * h_c)) +
                       (plink->I() - plink->mass() * (h_p * h_p));
        plink->setInertia(newI);
        plink->setMass(new_mass);
        plink->setCenterOfMass(new_c);
    }
    const Position cTb = clink->Tb();
    {// update visual shape of plink
        SgGroup *vsp = clink->visualShape();
        if(!!vsp) {
            CloneMap cmp;
            vsp = new SgGroup(*vsp, &cmp);
        }
        SgPosTransform *posT = new SgPosTransform();
        posT->addChild(vsp);
        posT->setPosition(cTb);
        plink->addVisualShapeNode(posT);
    }
    {// update collision shape of plink
        SgGroup *csp = clink->collisionShape();
        if(!!csp) {
            CloneMap cmp;
            csp = new SgGroup(*csp, &cmp);
        }
        SgPosTransform *posT = new SgPosTransform();
        posT->addChild(csp);
        posT->setPosition(cTb);
        plink->addCollisionShapeNode(posT);
    }
    // append children of clink
    for(int i = 0; i < all_child.size(); i++) {
        // update offset of children of clink
        Position newTb = cTb * all_child[i]->Tb();
        all_child[i]->setOffsetPosition(newTb);
        // append children of clink to plink
        plink->appendChild(all_child[i]);
    }
    return true;
}
bool RoboasmBodyCreator::mergeFixedJoint(BodyPtr _bd)
{
    std::cerr << "merge" << std::endl;
    Link *lk = _bd->rootLink();
    bool do_loop = true;
    while(do_loop) {
        Link *clink = nullptr;
        Link *cur = lk->child();
        while(!!cur) {
            std::cerr << "nm: " << cur->name() << " : " << cur->isFixedJoint() << std::endl;
            if(cur->isFixedJoint()) {
                clink = cur;
                break;
            }
            if(!!cur->sibling()) {
                cur = cur->sibling();
            } else {
                if(!!cur->parent()->child()) {
                    cur = cur->parent()->child()->child();
                } else {
                    cur = nullptr;
                }
            }
        }
        if(!clink) {
            break;
        }
        std::cout << "clink : " << clink->name() << std::endl;
        if(!mergeLink(clink->parent(), clink)) {
            return false;
        }
        _bd->updateLinkTree();
    }
    return true;
}

} }
