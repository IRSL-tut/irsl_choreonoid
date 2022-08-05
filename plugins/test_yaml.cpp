// g++ -c cnoid_yaml_test.cpp -I/home/irsl/sandbox/choreonoid_ws/devel/include/choreonoid-1.8
// g++ -o cnoid_yaml_test cnoid_yaml_test.o -L/home/irsl/sandbox/choreonoid_ws/devel/lib -lCnoidUtil

#include "RobotAssembler.h"
#include <iostream>

using namespace cnoid;
namespace ra = cnoid::robot_assembler;
using namespace ra;
#if 0
#include <cnoid/YAMLReader>

void parseNode(ValueNode *val, int indent = 0);

void parseNode(ValueNode *val, int indent)
{
    switch (val->nodeType()) {
    case ValueNode::SCALAR :
        for(int i = 0; i < indent; i++) std::cout << "  ";
        std::cout << "node[scalar]: " << val->toString() << std::endl;
        break;

    case ValueNode::MAPPING:
    {
        for(int i = 0; i < indent; i++) std::cout << "  ";
        //std::cout << "node[map]: " << val->toString() << std::endl;
        std::cout << "node[map]:" << std::endl;
        Mapping *cmapping = val->toMapping();
        for(auto it = cmapping->begin(); it != cmapping->end(); it++) {
            for(int i = 0; i < indent; i++) std::cout << "  ";
            std::cout << " key: " << it->first << std::endl;
            parseNode(it->second, indent + 1);
        }
    }
        break;

    case ValueNode::LISTING:
    {
        for(int i = 0; i < indent; i++) std::cout << "  ";
        //std::cout << "node[list]: " << val->toString() << std::endl;
        std::cout << "node[list]:" << std::endl;
        Listing *clist = val->toListing();
        for(int i = 0; i < clist->size(); i++) {
            parseNode(clist->at(i), indent + 1);
        }
    }
        break;

    }
}

int main(int argc, char **argv) {

    YAMLReader yaml_reader;

    bool ret = false;
    if (argc > 1) {
        std::cerr << "argc > 1" << std::endl;
        ret = yaml_reader.load(argv[1]);
    }
    if (ret) {
        std::cerr << "num: " << yaml_reader.numDocuments() << std::endl;
        for(int i = 0; i < yaml_reader.numDocuments(); i++) {
            ValueNode *val = yaml_reader.document(i);

            if ( val->nodeType() == ValueNode::MAPPING ) {
                std::string key = "actuators";
                ValueNode *target = val->toMapping()->find(key);
                if(!!target) {
                    std::cout << "--- target found ---" << std::endl;
                    parseNode(target);
                }
            }

            //std::cout << "document [" << i << "]"<< std::endl;
            //parseNode(val);
        }
    }
}
#endif
void print(coordinates &cds)
{
    std::cout << "((" << cds.pos(0) << " "
              << cds.pos(1) << " " << cds.pos(2);
    Vector3 rpy; cds.getRPY(rpy);
    std::cout << ") (" << rpy(0)  << " " << rpy(1)  << " "
              << rpy(2) << "))";
}
void print(ra::ConnectingConfiguration &in, int i)
{
    std::cout << "[ConnectingConfiguration] " << i;
    std::cout << " / name: " << in.name;
    std::cout << " / desc: " << in.description;
    std::cout << " / coords: ";
    print(in.coords); std::cout << std::endl;
}

void print(ra::ConnectingTypeMatch &in)
{
    std::cout << "[ConnectingTypeMatch] ";
    std::cout << "(" << in.pair[0] << "," << in.pair[1] << ") / [";
    for(int i = 0; i < in.allowed_configuration.size(); i++) {
        if (i == 0) {
            std::cout << in.allowed_configuration[i];
        } else {
            std::cout << ", " << in.allowed_configuration[i];
        }
    }
    std::cout << "]" << std::endl;
}

void print(ra::ConnectingPoint &in)
{
    ra::ConnectingPoint::PartsType tp_ = in.getType();
    if (tp_ == ra::ConnectingPoint::Parts) {
        std::cout << "  [ConnectingPoint]" << std::endl;
    } else {
        std::cout << "  [Actuator]" << std::endl;
    }
    std::cout << "    name: " << in.name << std::endl;
    std::cout << "    coords: ";
    print(in.coords); std::cout << std::endl;
    std::cout << "    type_list: ";
    for(int i = 0; i < in.type_list.size(); i++) {
        if(i == 0) {
            std::cout << in.type_list[i];
        } else {
            std::cout << ", " << in.type_list[i];
        }
    }
    std::cout << std::endl;
}

void print(ra::Actuator &in)
{
    print(*static_cast<ra::ConnectingPoint *> (&in));
    ra::ConnectingPoint::PartsType tp_ = in.getType();
    std::cout << "    actuator_type: ";
    switch (tp_) {
    case ra::ConnectingPoint::Rotational:
        std::cout << "rotational";
        break;
    default:
        std::cout << "unknown";
        break;
    }
    std::cout << std::endl;

    std::cout << "    axis: " << in.axis(0) << ", " << in.axis(1) << ", " << in.axis(2) << std::endl;
}

void print(ra::Geometry &in)
{
    std::cout << "  [Geometry]" << std::endl;
    std::cout << "    type: " << in.type << std::endl;
    std::cout << "    scale: " << in.scale << std::endl;
    std::cout << "    coords: "; print(in.coords); std::cout << std::endl;
    if (in.url.size() > 0) {
        std::cout << "    url: " << in.url << std::endl;
    }
    if (in.parameter.size() > 0) {
        std::cout << "    param: [";
        for(int i = 0; i < in.parameter.size(); i++) {
            if (i == 0) {
                std::cout << in.parameter[i];
            } else {
                std::cout << ", " << in.parameter[i];
            }
        }
        std::cout << "]" << std::endl;
    }
}

void print(ra::Parts &in)
{
    std::cout << "[Parts]" << std::endl;;
    std::cout << "  name: " << in.type << std::endl;
    std::cout << "  class: " << in.class_name << std::endl;

    if (in.hasMassParam) {
        std::cout << "  mass: " << in.mass << std::endl;
        std::cout << "   COM: " << in.COM(0) << ", " << in.COM(1) << ", " << in.COM(2) << std::endl;
        std::cout << "  inertia: " << in.inertia_tensor << std::endl;
    } else {
        std::cout << "  ## no mass-param" << std::endl;
    }

    std::cout << "  visual: " << in.visual.size() << std::endl;
    for(int i = 0; i < in.visual.size(); i++) {
        print(in.visual[i]);
    }
    std::cout << "  collision: " << in.collision.size() << std::endl;
    for(int i = 0; i < in.collision.size(); i++) {
        print(in.collision[i]);
    }
    std::cout << "  connecting-points: " << in.connecting_points.size() << std::endl;
    for(int i = 0; i < in.connecting_points.size(); i++) {
        print(in.connecting_points[i]);
    }
    std::cout << "  actuators: " << in.actuators.size() << std::endl;
    for(int i = 0; i < in.actuators.size(); i++) {
        print(in.actuators[i]);
    }
}

void print_lst(coordsList &lst)
{
    for(int i = 0; i < lst.size(); i++) {
        std::cout << i << " : [" << lst[i] << "] " << lst[i]->name() << " : ";
        print(lst[i]->worldcoords());
        std::cout << std::endl;
    }
}
void print_lst(coordsPtrList &lst)
{
    for(int i = 0; i < lst.size(); i++) {
        std::cout << i << " : [" << lst[i].get() << "] " << lst[i]->name() << " : ";
        print(lst[i]->worldcoords());
        std::cout << std::endl;
    }
}

int main(int argc, char **argv) {

    //ra::SettingsPtr ra_settings;
    std::shared_ptr<Roboasm> roboasm;
    bool ret = false;
    if (argc > 1) {
        std::cerr << "argc > 1" << std::endl;
        //ret = ra_settings->parseYaml(argv[1]);
        roboasm = std::make_shared<Roboasm>(argv[1]);
        ret = roboasm->isReady();
    }
    if (ret) {
        std::cerr << "success" << std::endl;
#if 0
        {
            std::vector<std::string> &lst = ra_settings->listConnectingTypeNames;
            std::cout << "listConnectingTypeNames: " << lst.size() << std::endl;
            for(int i = 0; i < lst.size(); i++) {
                std::cout << "  " << i << " " << lst[i] << std::endl;
            }
        }
        {
            std::vector<ra::ConnectingConfiguration> &lst = ra_settings->listConnectingConfiguration;
            std::cout << "listConnectingConfiguration: " << lst.size() << std::endl;
            for(int i = 0; i < lst.size(); i++) {
                print(lst[i], i);
            }
        }
        {
            std::vector<ra::ConnectingTypeMatch> &lst = ra_settings->listConnectingTypeMatch;
            std::cout << "listConnectingTypeMatch: " << lst.size() << std::endl;
            for(int i = 0; i < lst.size(); i++) {
                print(lst[i]);
            }
        }
        {
            std::cout << "mapParts: " << ra_settings->mapParts.size() << std::endl;
            for (auto it = ra_settings->mapParts.begin(); it != ra_settings->mapParts.end(); it++) {
                // it->first;
                //ra::Parts &pt = it->second;
                //print(pt);
                print(it->second);
            }
        }
#endif

#if 0
        std::cout << "a0" << std::endl;
        ra::RoboasmPartsPtr p = roboasm->makeParts("s3301");
        std::cout << "a1" << std::endl;
        ra::RoboasmRobotPtr r = roboasm->makeRobot("R(s3301)", p);
        std::cout << "a2" << std::endl;
        coordsList lst;
        std::cout << "a3" << std::endl;
        r->allDescendants(lst);
        std::cout << "a4" << std::endl;
        r->updateDescendants();
        std::cout << "lst : " << lst.size() << std::endl;
        print_lst(lst);
        coordsList lst2;
        for(int i = 0; i < lst.size(); i++) {
            RoboasmConnectingPoint * p = dynamic_cast<RoboasmConnectingPoint *>(lst[i]);
            if (!!p) {
                lst2.push_back(p);
            }
        }
        std::cout << "lst2 : " << lst2.size() << std::endl;
        print_lst(lst2);
        coordsList lst3;
        for(int i = 0; i < lst.size(); i++) {
            RoboasmParts * p = dynamic_cast<RoboasmParts *>(lst[i]);
            if (!!p) {
                lst3.push_back(p);
            }
        }
        std::cout << "lst3 : " << lst3.size() << std::endl;
        print_lst(lst3);
        coordsList lst4;
        for(int i = 0; i < lst.size(); i++) {
            RoboasmRobot * p = dynamic_cast<RoboasmRobot *>(lst[i]);
            if (!!p) {
                lst4.push_back(p);
            }
        }
        std::cout << "lst4 : " << lst4.size() << std::endl;
        print_lst(lst4);

        coordsPtrList lstp;
        r->allDescendants(lstp);
        std::cout << "lstp : " << lstp.size() << std::endl;
        print_lst(lstp);
        coordsPtrList lstp2;
        for(int i = 0; i < lstp.size(); i++) {
            RoboasmConnectingPointPtr p = std::dynamic_pointer_cast<RoboasmConnectingPoint>(lstp[i]);
            if (!!p) {
                lstp2.push_back(p);
            }
        }
        std::cout << "lstp2 : " << lstp2.size() << std::endl;
        print_lst(lstp2);

        coordsPtrList lstp3;
        r->allDescendants<RoboasmParts>(lstp3);
        std::cout << "lstp3 : " << lstp3.size() << std::endl;
        print_lst(lstp3);
#if 0
        ra::RoboasmCoordsPtr p0 = std::make_shared<ra::RoboasmCoords>("p0");
        ra::RoboasmCoordsPtr p1 = std::make_shared<ra::RoboasmCoords>("p1");
        coordinates c0(Vector3(1.0, 0, 0));
        coordinates c1(Vector3(0, 0, 1.0));
        p0->set(c0);
        p1->set(c1);
        p0->update();
        p1->update();
        std::cout << "p0 " ;
        print(p0->worldcoords()); std::cout << std::endl;
        std::cout << "p1 " ;
        print(p1->worldcoords()); std::cout << std::endl;

        p0->assoc(p1);
        p0->updateDescendants();

        std::cout << "p0 " << p0.get() << std::endl;
        std::cout << "p0 w:" ;
        print(p0->worldcoords()); std::cout << std::endl;
        std::cout << "p0 l:" ;
        print(*dynamic_cast<coordinates *>(p0.get()));  std::cout << std::endl;
        std::cout << "p1 " << p1.get() << std::endl;
        std::cout << "p1 p:" << p1->parent() << std::endl;
        std::cout << "p1 w:" ;
        print(p1->worldcoords()); std::cout << std::endl;
        std::cout << "p1 l:" ;
        print(*dynamic_cast<coordinates *>(p1.get()));  std::cout << std::endl;

        p1->update();

        std::cout << "p1 " << p1.get() << std::endl;
        std::cout << "p1 p:" << p1->parent() << std::endl;
        std::cout << "p1 w:" ;
        print(p1->worldcoords()); std::cout << std::endl;
        std::cout << "p1 l:" ;
        print(*dynamic_cast<coordinates *>(p1.get()));  std::cout << std::endl;

        p0->translate(Vector3(0, 1, 0));
        p0->update();
        p0->updateDescendants();
        std::cout << "p0 w:" ;
        print(p0->worldcoords()); std::cout << std::endl;
        std::cout << "p1 w:" ;
        print(p1->worldcoords()); std::cout << std::endl;
#endif
#endif
        ra::RoboasmPartsPtr p0 = roboasm->makeParts("s3301", "s3301");
        if(!p0) {
            std::cout << "fail make p0" << std::endl;
            return 0;
        }
        ra::RoboasmRobotPtr r = roboasm->makeRobot("R(s3301)", p0);

        coordsPtrList lst;
        std::cout << "A0" << std::endl;
        r->allDescendants(lst);

        coordsPtrList lst2;
        std::cout << "A1" << std::endl;
        r->allDescendants<RoboasmConnectingPoint>(lst2);

        std::cout << "AA" << std::endl;
        ra::RoboasmPartsPtr p1 = roboasm->makeParts("upper-arm20", "upper-arm20");
        if(!p1) {
            std::cout << "fail make p1" << std::endl;
            return 0;
        }
        ra::RoboasmPartsPtr p2 = roboasm->makeParts("joint-base", "joint-base");
        if(!p2) {
            std::cout << "fail make p2" << std::endl;
            return 0;
        }
        ra::RoboasmPartsPtr p3 = roboasm->makeParts("bottom-arm20", "bottom-arm20");
        if(!p3) {
            std::cout << "fail make p3" << std::endl;
            return 0;
        }
        {
            RoboasmConnectingPointPtr parts_point;
            RoboasmConnectingPointPtr robot_point;
            ConnectingTypeMatch *_res_match;
            ConnectingConfiguration *_res_config;
            bool ret;
            std::cerr << "p1" << std::endl;
            ret = r->checkAttachByName(p1,
                                       "upper-arm20/horn-hole",
                                       "s3301/horn",
                                       "fixed",
                                       parts_point,
                                       robot_point,
                                       _res_config,
                                       _res_match);
            std::cout << "ret = " << ret << std::endl;
            if (ret) {
                std::cout << "attach" << std::endl;
                coordinates conf_coords;
                r->attach(p1, parts_point, robot_point, _res_config);
            }
        }
        {
            RoboasmConnectingPointPtr parts_point;
            RoboasmConnectingPointPtr robot_point;
            ConnectingTypeMatch *_res_match;
            ConnectingConfiguration *_res_config;
            bool ret;
            std::cerr << "p2" << std::endl;
            ret = r->checkAttachByName(p2,
                                       "joint-base/side-hole0",
                                       "upper-arm20/single-hole",
                                       "fixed",
                                       parts_point,
                                       robot_point,
                                       _res_config,
                                       _res_match);
            std::cout << "ret = " << ret << std::endl;
            if (ret) {
                std::cout << "attach" << std::endl;
                coordinates conf_coords;
                r->attach(p2, parts_point, robot_point, _res_config);
            }
        }
        {
            RoboasmConnectingPointPtr parts_point;
            RoboasmConnectingPointPtr robot_point;
            ConnectingTypeMatch *_res_match;
            ConnectingConfiguration *_res_config;
            bool ret;
            std::cerr << "p3" << std::endl;
            ret = r->checkAttachByName(p3,
                                       "bottom-arm20/single-hole",
                                       "joint-base/side-hole1",
                                       "fixed",
                                       parts_point,
                                       robot_point,
                                       _res_config,
                                       _res_match);
            std::cout << "ret = " << ret << std::endl;
            if (ret) {
                std::cout << "attach" << std::endl;
                coordinates conf_coords;
                r->attach(p3, parts_point, robot_point, _res_config);
            }
        }

        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << ";;; orig ;;;" << std::endl;
        {
            coordsPtrList lstp;
            r->allDescendants<RoboasmParts>(lstp);
            std::cout << "parts : " << lstp.size() << std::endl;
            print_lst(lstp);
        }
        {
            coordsPtrList lstp;
            r->allDescendants<RoboasmConnectingPoint>(lstp);
            std::cout << "connecting-points : " << lstp.size() << std::endl;
            print_lst(lstp);
        }

        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << ";;; moved ;;;" << std::endl;
        coordinates base(Vector3(0.1, 0.2, 1.0), 0.0, 0.8, 0.0);
        r->newcoords(base);
        r->updateDescendants();
        {
            coordsPtrList lstp;
            r->allDescendants<RoboasmParts>(lstp);
            std::cout << "parts : " << lstp.size() << std::endl;
            print_lst(lstp);
        }
        {
            coordsPtrList lstp;
            r->allDescendants<RoboasmConnectingPoint>(lstp);
            std::cout << "connecting-points : " << lstp.size() << std::endl;
            print_lst(lstp);
        }

        {
            RoboasmCoordsPtr pp = r->find("bottom-arm20/single-hole-dummy");
            if(!!pp) {
                coordsList lst;
                pp->toRootList(lst);
                std::cout << "lst : " << lst.size() << std::endl;
                print_lst(lst);
            }
            if(!!pp) {
                coordsPtrList lst;
                pp->toRootList(lst);
                std::cout << "plst : " << lst.size() << std::endl;
                print_lst(lst);
            }
            RoboasmConnectingPointPtr rcp = std::dynamic_pointer_cast<RoboasmConnectingPoint>(pp);
            if(!!rcp) {
                r->changeRoot(rcp);
                coordsPtrList lst;
                rcp->updateDescendants();
                rcp->allDescendants(lst);
                std::cout << "lst : " << lst.size() << std::endl;
                print_lst(lst);
            } else {
                std::cout << "failed" << std::endl;
            }
        }
#if 0
        coordinates cds(Vector3 (123, 55, 321), 0, M_PI/2, 0);
        coordinates inv;
        std::cout << "cds: "; print(cds); std::cout << std::endl;
        std::cout << "mat: " << cds.rot << std::endl;
        cds.inverse_transformation(inv);
        std::cout << "inv: "; print(inv); std::cout << std::endl;
        std::cout << "mat: " << inv.rot << std::endl;
        cds.inverse();
        std::cout << "(inv)cds: "; print(cds); std::cout << std::endl;
        std::cout << "mat: " << cds.rot << std::endl;
#endif
        std::cout << "SCOPE END" << std::endl;
    }
    std::cout << "SCOPE ENDED" << std::endl;
    //

    {
        RoboasmFile roboasm("assembled_robot.roboasm");
        if(roboasm.isValid()) {
            std::cout << "valid file" << std::endl;
            if(!roboasm.dumpRoboasm("/tmp/hoge.roboasm")) {
                std::cout << "dump fail" << std::endl;
            }
        }
    }
    RoboasmRobotPtr roboasm_rb = roboasm->makeRobotFromFile("assembled_robot.roboasm");
    if(!!roboasm_rb) {
        coordsPtrList lstp;
        roboasm_rb->allDescendants(lstp);
        std::cout << "roboasm_rb : " << lstp.size() << std::endl;
        print_lst(lstp);

        if(roboasm_rb->checkValidity()) {
            std::cout << "VALID" << std::endl;
        }
        RoboasmFile rb_file;
        roboasm_rb->createRoboasm(rb_file);
        if(!rb_file.dumpRoboasm("/tmp/fuga.roboasm")) {
            std::cout << "dump fail fuga" << std::endl;
        }
    }

    std::cout << "SCOPE FINISH" << std::endl;
}
