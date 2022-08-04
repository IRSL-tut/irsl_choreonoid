#include <cnoid/Body>
#include <cnoid/BodyLoader>
//#include "RobotAssembler.h"
#include <irsl_choreonoid/Coordinates.h>
#include <cnoid/YAMLReader> // ValueNode
#include <cnoid/SceneGraph>

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
        return 0;
    }

    traverseBody(body);
}
//
// link->appendChild(child_link);
