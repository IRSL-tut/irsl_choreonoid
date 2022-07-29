#include <irsl_choreonoid/Coordinates.h>
#include <string>
#include <vector>
#include <map>

#pragma once

namespace cnoid {
namespace robot_assembler {

typedef long ConnectingType;
typedef long ConnectingConfigurationType;
struct ConnectingConfiguration;
struct ConnectingTypeMatch;
struct ConnectingPoint;
struct Actuator;
struct ExtraInfo;
struct Geometry;

struct ConnectingConfiguration
{
    std::string name;
    std::string description;
    coordinates coords;
};

struct ConnectingTypeMatch
{
    ConnectingType pair[2];
    std::vector<ConnectingConfigurationType> allowed_configuration;
};

struct ConnectingPoint
{
    std::string name;
    std::vector<ConnectingType> type_list;
    coordinates coords;
};

struct Actuator : public ConnectingPoint
{
    enum ActuatorType {
        None = 0,
        Revolute = 1 << 0,
        Linear = 1 << 1,
        Free = 1 << 2,
        Fixed = 1 << 3
    }; // sphere
    ActuatorType type;
    Vector3 axis;
};

struct ExtraInfo
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

struct Geometry
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
    Vector3 COM; // center of mass
    Matrix3 inertia_tensor;

    std::vector<ConnectingPoint> connecting_points;
    std::vector<Actuator> actuators;
    std::vector<ExtraInfo> extra_data;
};

////
class RobotAssemblerConfiguration
{
public:
    RobotAssemblerConfiguration();
    ~RobotAssemblerConfiguration();

    std::vector<std::string> listConnectingTypeNames;
    std::vector<ConnectingConfiguration> listConnectingConfiguration;
    std::vector<ConnectingTypeMatch> listConnectingTypeMatch;

    std::map<std::string, Parts> mapParts;

    bool parseYaml(const std::string &filename);

private:
    class Impl;
    Impl *impl;
};

}; };
