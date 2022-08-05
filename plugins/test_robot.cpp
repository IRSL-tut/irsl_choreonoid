#include <cnoid/Body>
#include <cnoid/BodyLoader>
//#include "RobotAssembler.h"
#include <irsl_choreonoid/Coordinates.h>
#include <cnoid/YAMLReader> // ValueNode
#include <cnoid/SceneGraph>
#include <cnoid/CloneMap>
#include <cnoid/StdBodyWriter>

using namespace cnoid;

void print(coordinates &cds)
{
    std::cout << "((" << cds.pos(0) << " "
              << cds.pos(1) << " " << cds.pos(2);
    Vector3 rpy; cds.getRPY(rpy);
    std::cout << ") (" << rpy(0)  << " " << rpy(1)  << " "
              << rpy(2) << "))";
}
void print(const Vector3 &vec)
{
    std::cout << "(" << vec(0) << " "
              << vec(1) << " " << vec(2);
    std::cout << ")";
}
void traverseSG(SgObject *sgo, int indent = 0)
{
    for(int i=0;i<indent;i++) std::cout << "  ";
    std::cout << "  sg(name) : " << sgo->name() << std::endl;
    if(sgo->isNode()) {
        for(int i=0;i<indent;i++) std::cout << "  ";
        std::cout << "  sg(class) : " << sgo->toNode()->className() << std::endl;
    }
    SgPosTransform *pt = dynamic_cast<SgPosTransform*>(sgo);
    if (!!pt) {
        coordinates cds(pt->position());
        for(int i=0;i<indent;i++) std::cout << "  ";
        std::cout << "  sg(trans) : "; print(cds); std::cout << std::endl;
    }
    //sgo->isGroupNode();
    //sgo->isTransformNode();
    int num = sgo->numChildObjects();
    for(int i = 0; i < num; i++) {
        traverseSG(sgo->childObject(i), indent + 1);
    }
}
void traverseLink(Link *lk)
{
    std::cout << "lk:name: " << lk->name() << std::endl;
    if(!!lk->parent())  std::cout << "   parent: " << lk->parent()->name() << std::endl;
    if(!!lk->child())   std::cout << "    child: " << lk->child()->name() << std::endl;
    if(!!lk->sibling()) std::cout << "  sibling: " << lk->sibling()->name() << std::endl;

    coordinates Tb(lk->Tb());
    std::cout << " Tb: "; print(Tb); std::cout << std::endl;
    std::cout << " mass: " << lk->mass() << std::endl;
    std::cout << " COM: "; print(lk->centerOfMass()); std::cout << std::endl;
    std::cout << " axis: "; print(lk->jointAxis()); std::cout << std::endl;

    Matrix3 im = lk->I();
    std::cout << "   I:"; print(im.row(0)); std::cout << std::endl;
    std::cout << "     "; print(im.row(1)); std::cout << std::endl;
    std::cout << "     "; print(im.row(2)); std::cout << std::endl;

    traverseSG(lk->visualShape());
    Mapping *mp = lk->info();
    std::cout << "  info : " << mp->size() << std::endl;
    for(auto it = mp->begin(); it != mp->end(); it++) {
        std::cout << "  " << it->first << ":" << std::endl;
        //printNode(out, it->second, indent + 1);
    }
    if(!!lk->child()) traverseLink(lk->child());
    if(!!lk->sibling()) traverseLink(lk->sibling());
    //cur_lk->isFixedJoint();
    //cur_lk->visualShape();
}
void traverseBody(Body *bd)
{
    std::cout << "bd : " << bd->name() << std::endl;
    std::cout << "numLInks() : " << bd->numLinks() << std::endl;
    Mapping *mp = bd->info();
    std::cout << "info : " << mp->size() << std::endl;
    for(auto it = mp->begin(); it != mp->end(); it++) {
        std::cout << it->first << ":" << std::endl;
        //printNode(out, it->second, indent + 1);
    }
    if(!!bd->parentBody()) {
        std::cout << "has parent body" << std::endl;
    }
    if(!!bd->parentBodyLink()) {
        std::cout << "has parent body link" << std::endl;
    }
    traverseLink(bd->rootLink());
}
bool mergeLink(Link *plink, Link *clink)
{
    if(!plink) {
        std::cerr << "plink does not exist" << std::endl;
        return false;
    }
    if(!clink) {
        std::cerr << "clink does not exist" << std::endl;
        return false;
    }
    LinkPtr link_protect(clink);
    std::vector<LinkPtr> all_child;
    {
        Link *cur = clink->child();
        while(!!cur) {
            all_child.push_back(cur);
            cur = cur->sibling();
        }
    }
    std::cout << "child : " << all_child.size() << std::endl;
    // remove child
    if(!plink->removeChild(clink)) {
        std::cerr << "not a child2" << std::endl;
        return false;
    }
    std::cout << "removed" << std::endl;
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
int main(int argc, char **argv)
{
    Body *body = nullptr;
    if (argc > 1) {
        std::cout << "argc > 1" << std::endl;
        BodyLoader bl;
        body = bl.load(argv[1]);
    }

    if(!body) {
        std::cout << "body not loaded" << std::endl;
        return -1;
    }

    std::cout << "Original" << std::endl;
    traverseBody(body);
    {
        StdBodyWriter writer;
        writer.writeBody(body, "/tmp/sr.org.body");
    }
    //
    //Link *plink = body->link("WAIST");
    //Link *clink = body->link("WAIST_P");
    //Link *plink = body->link("LLEG_ANKLE_P");
    //Link *clink = body->link("LLEG_ANKLE_R");
    Link *plink = body->link("LLEG_KNEE");
    Link *clink = body->link("LLEG_ANKLE_P");
    if(!mergeLink(plink, clink)) {
        std::cout << "merge failed" << std::endl;
        return -1;
    }
    std::cout << "Merged0" << std::endl;
    body->updateLinkTree();
    std::cout << "Merged1" << std::endl;
    //traverseBody(body);
    {
        StdBodyWriter writer;
        writer.writeBody(body, "/tmp/sr.new.body");
    }
    return 0;
}
//
// link->appendChild(child_link);
