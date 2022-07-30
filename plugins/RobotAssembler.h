#include <irsl_choreonoid/Coordinates.h>
#include <string>
#include <vector>
#include <map>
#include <memory>

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

class PointBase
{
public:
    std::string name;
    coordinates coords;
};

class ConnectingPoint : public PointBase
{
public:
    //using PointBase::name;
    //using PointBase::coords;

    enum PartsType {
        Parts = 0, // Parts
        Rotational = 1 << 0, // Actuator
        Linear     = 1 << 1,
        Fixed      = 1 << 2,
        Sphere     = 1 << 3,
        Plane      = 1 << 4,
        Spherical  = 1 << 5,
        Free       = 1 << 6,
        UNDEFINED  = 1 << 7
    };

    ConnectingPoint() : type(Parts) {}
    virtual PartsType getType() { return type; }

    std::vector<ConnectingType> type_list;
protected:
    PartsType type;
};

class Actuator : public ConnectingPoint
{
public:
    //using ConnectingPoint::name;
    //using ConnectingPoint::coords;
    //using ConnectingPoint::type_list;

    Actuator() { type = UNDEFINED; }
    //Actuator(const ConnectingPoint &cpt, PartsType _tp) { *this = cpt; type = _tp; }
    Actuator(PartsType _tp) { type = _tp; }
    //ActuatorType type;
    Vector3 axis;
    double limit[2];
    double vlimit[2];
    double tqlimit[2];
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
        Ellipsoid,
        Dummy // can not detect collision
    };

    coordinates coords;
    std::string url;
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

typedef std::shared_ptr<RobotAssemblerConfiguration> RobotAssemblerConfigurationPtr;

class RoboasmCoords;
typedef std::shared_ptr<RoboasmCoords> RoboasmCoordsPtr;
class RoboasmCoords : coordinates
{
    std::string name;
    RoboasmCoordsPtr parent;
    std::vector<RoboasmCoordsPtr> descendants;

    // virtual
    void update();
    void updateDescendants();

    void assoc(RoboasmCoordsPtr c);
    void dissocParent();

    void toRootList();
    void allDescendants();
    void toNextLink();

    void connectingPoints();
    void activeConnectingPoints();
    void actuators();
    void activeActuators();
    void find();
};

class RoboasmConnectingPoint : public RoboasmCoords
{
};
typedef std::shared_ptr<RoboasmConnectingPoint> RoboasmConnectingPointPtr;

class RoboasmParts : public RoboasmCoords
{
};
typedef std::shared_ptr<RoboasmParts> RoboasmPartsPtr;

class RoboasmRobot : public RoboasmCoords
{
};
typedef std::shared_ptr<RoboasmRobot> RoboasmRobotPtr;
}; };
