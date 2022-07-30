// g++ -c cnoid_yaml_test.cpp -I/home/irsl/sandbox/choreonoid_ws/devel/include/choreonoid-1.8
// g++ -o cnoid_yaml_test cnoid_yaml_test.o -L/home/irsl/sandbox/choreonoid_ws/devel/lib -lCnoidUtil

#include "RobotAssembler.h"
#include <iostream>

using namespace cnoid;
namespace ra = cnoid::robot_assembler;

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
    std::cout << "((" << cds.pos(0) << ", "
              << cds.pos(1) << ", " << cds.pos(2);
    Vector3 rpy; cds.getRPY(rpy);
    std::cout << ") (" << rpy(0)  << ", " << rpy(1)  << ", "
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

int main(int argc, char **argv) {

    ra::RobotAssemblerConfiguration conf;

    bool ret = false;
    if (argc > 1) {
        std::cerr << "argc > 1" << std::endl;
        ret = conf.parseYaml(argv[1]);
    }
    if (ret) {
        std::cerr << "success" << std::endl;

        {
            std::vector<std::string> &lst = conf.listConnectingTypeNames;
            std::cout << "listConnectingTypeNames: " << lst.size() << std::endl;
            for(int i = 0; i < lst.size(); i++) {
                std::cout << "  " << i << " " << lst[i] << std::endl;
            }
        }
        {
            std::vector<ra::ConnectingConfiguration> &lst = conf.listConnectingConfiguration;
            std::cout << "listConnectingConfiguration: " << lst.size() << std::endl;
            for(int i = 0; i < lst.size(); i++) {
                print(lst[i], i);
            }
        }
        {
            std::vector<ra::ConnectingTypeMatch> &lst = conf.listConnectingTypeMatch;
            std::cout << "listConnectingTypeMatch: " << lst.size() << std::endl;
            for(int i = 0; i < lst.size(); i++) {
                print(lst[i]);
            }
        }
        {
            std::cout << "mapParts: " << conf.mapParts.size() << std::endl;
            for (auto it = conf.mapParts.begin(); it != conf.mapParts.end(); it++) {
                // it->first;
                //ra::Parts &pt = it->second;
                //print(pt);
                print(it->second);
            }
        }
    }
}
