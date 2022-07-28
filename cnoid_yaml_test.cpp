// g++ -c cnoid_yaml_test.cpp -I/home/irsl/sandbox/choreonoid_ws/devel/include/choreonoid-1.8
// g++ -o cnoid_yaml_test cnoid_yaml_test.o -L/home/irsl/sandbox/choreonoid_ws/devel/lib -lCnoidUtil

#include <cnoid/YAMLReader>
#include <iostream>

typedef long ConnectingType;
typedef long ConnectingConfigurationType;
class ConnectingConfiguration;
class ConnectingTypeMatch;
class ConnectingPoint;
class Actuator;
class ExtraInfo;
class Geometry;
class Parts;

std::vector<std::string> listConnectingTypeNames;
std::vector<ConnectingConfiguration> listConnectingConfiguration;

class ConnectingConfiguration
{
    std::string name;
    std::string description;
    coordinates coords;
};

class ConnectingTypeMatch
{
    ConnectingType[2] pair;
    std::vector<ConnectingConfigurationType> allowed_configuration;
};

class ConnectingPoint
{
    std::string name;
    std::vector<ConnectingType> type_list;
    coordinate coords;
};

class Actuator : public ConnectingPoint
{
    enum ActuatorType {
        None = 0,
        Revolute = 1 << 0,
        Linear = 1 << 1,
        Free = 1 << 2,
        Fixed = 1 << 3
    }; // sphere
    ActuatorType type;
    cnoid::Vector3 axis;
};

class ExtraInfo
{
    enum Type {
        None,
        IMU,
        Touch,
        Force,
    };
    std::string name;
    Type type;
    coordinates coords;
    // parameters Mapping
};

class Geometry
{
    enum Type {
        None,
        Mesh,
        Box,
        Cylinder,
        Sphere,
        Cone,
        Capsule,
        Ellipsoid
    };

    coordinates coords;
    std::string uri;
    double scale;
    Type type;
    std::vector<double> parameter;
};

class Parts
{
public:
    std::string type;
    std::string class_name;

    std::vector<Geometry> visual;
    std::vector<Geometry> collision;

    bool hasMassParam;
    double mass;
    cnoid::Vector3 COM; // center of mass
    cnoid::Matrix3 inertia_tensor;

    std::vector<ConnectingPoint> connecting_points;
    std::vector<Actuator> actuators;
    std::vector<ExtraInfo> extra_data;
};

using namespace cnoid;

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
