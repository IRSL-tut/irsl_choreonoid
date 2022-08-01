#include <irsl_choreonoid/Coordinates.h>

#include <string>
#include <vector>
#include <memory>
#include <map>
//#include <set>

#pragma once

namespace cnoid {
namespace robot_assembler {

typedef long ConnectingTypeID;
typedef long ConnectingConfigurationID;
struct ConnectingConfiguration;
struct ConnectingTypeMatch;
class PointBase;
class ConnectingPoint;
class Actuator;
struct ExtraInfo;
struct Geometry;
class Parts;
typedef std::shared_ptr<Parts> PartsPtr;

#if 0
struct ConnectingConstraint
{
    enum Type {
        None,
        Rotational_X = 1 << 0, // param(0), limit(2), dof(1)
        Rotational_Y = 1 << 1, // param(0), limit(2), dof(1)
        Rotational_Z = 1 << 2, // param(0), limit(2), dof(1)
        Rotational   = 1 << 3, // param(3), limit(2), dof(1) ? angle_axis
        Rotational_XY = 1 << 4, // param(0), limit(4), dof(2)
        Rotational_YZ = 1 << 5, // param(0), limit(4), dof(2)
        Rotational_ZX = 1 << 6, // param(0), limit(4), dof(2)
        Rotational_2D = 1 << 7, // param(6), limit(4), dof(2) ? angle_axis * angle_axis ??
        Free_Rotate   = 1 << 8, // param(0), limit(6), dof(3)
        Linear_X = 1 << 9,  // param(0), limit(2), dof(1)
        Linear_Y = 1 << 10, // param(0), limit(2), dof(1)
        Linear_Z = 1 << 11, // param(0), limit(2), dof(1)
        Linear   = 1 << 12, // param(6), limit(2), dof(1)
        Plane_XY = 1 << 13, // param(0), limit(4), dof(2)
        Plane_YZ = 1 << 14, // param(0), limit(4), dof(2)
        Plane_ZX = 1 << 15, // param(0), limit(4), dof(2)
        Plane    = 1 << 16, // param(6), limit(4), dof(2)
        Free_Translate = 1 << 17, // param(0), limit(6), dof(3)
        ParemetricLine,
        ParametricSurface,
        ParametricVolume,
    };
  std::string name;
  std::string description
  Type type;
  //type // plane
  std::vector<double> parameter;
};
struct ConstraintTypeMatch
{
    ConnectingTypeID pair[2];
    ConnectingConstraint constraint;
};
struct ConstraintConfiguration
{
    coordinates coords;
    std::vector<double> parameter;
};
#endif
struct ConnectingType
{
    std::string name;
    //
    ConnectingTypeID index;
};
struct ConnectingConfiguration
{
    std::string name;
    std::string description;
    coordinates coords;
    //
    ConnectingConfigurationID index;
};

struct ConnectingTypeMatch
{
    ConnectingTypeID pair[2];
    std::vector<ConnectingConfigurationID> allowed_configuration;
    //
    int index;
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
    //[TODO]
    // reverse configuration
    // reverse-search coords->configuration
    std::vector<ConnectingTypeID> type_list;
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
    Parts *parent_parts;
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
class Settings
{
public:
    Settings();
    ~Settings();

    std::vector<std::string> listConnectingTypeNames;
    std::vector<ConnectingConfiguration> listConnectingConfiguration;
    std::vector<ConnectingTypeMatch> listConnectingTypeMatch;

    std::map<std::string, Parts> mapParts;

    bool parseYaml(const std::string &filename);
    //RoboasmPartsPtr makeParts(const std::string &parts_key);
    //RoboasmPartsPtr makeParts(const std::string &parts_key, const std::string &_name);
    //RoboasmRobotPtr makeRobot(const std::string &_name, RoboasmPartsPtr parts);

    ConnectingTypeMatch *searchMatch(ConnectingTypeID _a, ConnectingTypeID _b);
    ConnectingTypeMatch *searchConnection(ConnectingTypeID _a, ConnectingTypeID _b,
                                          ConnectingConfigurationID _tp);
    ConnectingTypeMatch *searchConnection(ConnectingTypeID _a, ConnectingTypeID _b,
                                          const std::string &config_name);
    ConnectingTypeMatch *searchConnection(ConnectingTypeID _a, ConnectingTypeID _b,
                                          ConnectingConfigurationID _tp,
                                          ConnectingConfiguration *_res);
    ConnectingTypeMatch *searchConnection(ConnectingTypeID _a, ConnectingTypeID _b,
                                          const std::string &config_name,
                                          ConnectingConfiguration *_res);
    //int searchMatch(ConnectingTypeID a, ConnectingTypeID b);
    // match / invert?
    // A, A => parent/child <-
    // B, C => parent/child <-
private:
    //RoboasmRobotPtr current_robot;
    class Impl;
    Impl *impl;
    // speedup search-match
};
typedef std::shared_ptr<Settings> SettingsPtr;

} }
