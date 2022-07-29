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

void print(ra::ConnectingConfiguration &in, int i)
{
    std::cout << "[ConnectingConfiguration] " << i;
    std::cout << " / name: " << in.name;
    std::cout << " / desc: " << in.description;
    std::cout << " / coords: " << in.coords.pos;
    Vector3 rpy; in.coords.getRPY(rpy);
    std::cout << " " << rpy << std::endl;
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
    }
}




