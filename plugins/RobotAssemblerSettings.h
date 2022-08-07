#ifndef CNOID_ROBOT_ASSEMBLER_SETTINGS_H
#define CNOID_ROBOT_ASSEMBLER_SETTINGS_H

#include <irsl_choreonoid/Coordinates.h>

#include <string>
#include <vector>
#include <memory>
#include <map>
//#include <set>

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
    long index;
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
    Actuator() {
        axis.Zero();
        limit[0] = 0.0;
        limit[1] = 0.0;
        vlimit[0] = 0.0;
        vlimit[1] = 0.0;
        tqlimit[0] = 0.0;
        tqlimit[1] = 0.0;
        type = UNDEFINED;
    }
    Actuator(PartsType _tp) : Actuator() { type = _tp; }

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
    Parts() : hasMassParam(false), mass(0.0)
    {
        COM.setZero();
        inertia_tensor.setZero();
    }

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

    std::vector<ConnectingType> listConnectingType;
    std::vector<ConnectingConfiguration> listConnectingConfiguration;
    std::vector<ConnectingTypeMatch> listConnectingTypeMatch;

    std::map<std::string, Parts> mapParts;

    bool parseYaml(const std::string &filename);

    ConnectingTypeMatch *searchMatch(ConnectingTypeID _a, ConnectingTypeID _b);
    ConnectingTypeMatch *searchConnection(ConnectingTypeID _a, ConnectingTypeID _b,
                                          ConnectingConfigurationID _tp);
    ConnectingType *searchConnectingType(const std::string &_name);
    ConnectingConfiguration *searchConnectingConfiguration(const std::string &_name);
    //int searchMatch(ConnectingTypeID a, ConnectingTypeID b);
    // match / invert?
    // A, A => parent/child <-
    // B, C => parent/child <-
private:
    class Impl;
    Impl *impl;
};
typedef std::shared_ptr<Settings> SettingsPtr;

class AttachHistoryItem
{
public:
    AttachHistoryItem() : inverse(false), initial_parts(false) {}
    std::string robot_parts_point;
    std::string parts_name;
    std::string parts_type;
    std::string parts_point;
    std::string configuration; // configuration and config_coords are exclusive
    coordinates config_coords; //
    std::string parent;
    bool inverse;
    bool initial_parts;

    std::ostream &print(std::ostream &ostr)
    {
        ostr << "(";
        if (parent.size() > 0) {
            ostr << "(:parent " << parent << ")";
        }
        if (parts_name.size() > 0) {
            ostr << "(:parts-name " << parts_name << ")";
        }
        if (parts_type.size() > 0) {
            ostr << "(:parts-type " << parts_type << ")";
        }
        if (parts_point.size() > 0) {
            ostr << "(:parts-point " << parts_point << ")";
        }
        if (robot_parts_point.size() > 0) {
            ostr << "(:robot-parts-point " << robot_parts_point << ")";
        }
        if (configuration.size() > 0) {
            ostr << "(:configuration " << configuration << ")";
        } else {
            ostr << "(:configuration ((";
            Vector3 rpy; config_coords.getRPY(rpy);
            ostr << config_coords.pos(0) << " ";
            ostr << config_coords.pos(1) << " ";
            ostr << config_coords.pos(2) << ")(";
            ostr << rpy(2) << " ";
            ostr << rpy(1) << " ";
            ostr << rpy(0) << "))";
        }
        if(initial_parts) {
            ostr << "(:initial-parts t)";
        }
        if(inverse) {
            ostr << "(:inverse t)";
        }
        ostr << ")";
        return ostr;
    }
};
typedef std::vector<AttachHistoryItem> AttachHistory;
typedef std::map<std::string, AttachHistoryItem*> StringMap;
typedef std::pair<std::string, AttachHistoryItem*> StringPair;

//void mergeAttachHistory(AttachHistory &_parent, const AttachHistory &_child);

struct AssembleConfig
{
    std::string robot_name;
    coordinates initial_coords;
    std::map<std::string, std::string> actuator_name;
    std::map<std::string, std::string> actuator_axis_name;
    std::map<std::string, Vector3> actuator_axis_vector;

    bool isValid() {
        if(robot_name.size() > 0) return true;
        if(!initial_coords.isInitial()) return true;
        if(actuator_name.size() > 0) return true;
        if(actuator_axis_name.size() > 0) return true;
        if(actuator_axis_vector.size() > 0) return true;
        return false;
    }
};
class RoboasmFile
{
public:
    RoboasmFile() {}
    RoboasmFile(const std::string &_filename)
    {
        valid_ = this->parseRoboasm(_filename);
    }

    bool isValid() { return valid_; }
    AttachHistory history;
    AssembleConfig config;

    bool parseRoboasm(const std::string &_filename);
    bool dumpRoboasm(const std::string &_filename);
private:
    bool valid_;
};

} }

#endif
