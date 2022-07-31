#include <irsl_choreonoid/Coordinates.h>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <memory>
#include <algorithm>

#pragma once

namespace cnoid {
namespace robot_assembler {

typedef long ConnectingType;
typedef long ConnectingConfigurationType;
struct ConnectingConfiguration;
struct ConnectingTypeMatch;
class PointBase;
class ConnectingPoint;
class Actuator;
struct ExtraInfo;
struct Geometry;
class Parts;
typedef std::shared_ptr<Parts> PartsPtr;

// roboasm classes
class RoboasmCoords;
class RoboasmConnectingPoint;
class RoboasmParts;
class RoboasmRobot;
typedef std::shared_ptr<RoboasmCoords> RoboasmCoordsPtr;
typedef std::shared_ptr<RoboasmParts> RoboasmPartsPtr;
typedef std::shared_ptr<RoboasmConnectingPoint> RoboasmConnectingPointPtr;
typedef std::shared_ptr<RoboasmRobot> RoboasmRobotPtr;

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
    RoboasmPartsPtr makeParts(const std::string &parts_key);
    RoboasmPartsPtr makeParts(const std::string &parts_key, const std::string &name);
private:
    class Impl;
    Impl *impl;
};
typedef std::shared_ptr<RobotAssemblerConfiguration> RobotAssemblerConfigurationPtr;

typedef std::vector<RoboasmCoords *> coordsList;
typedef std::vector<RoboasmCoordsPtr> coordsPtrList;
typedef std::vector<RoboasmConnectingPointPtr> connectingPointPtrList;
typedef std::vector<RoboasmPartsPtr> partsPtrList;

class RoboasmCoords : public coordinates
{

public:
    RoboasmCoords(const std::string &_name);

    const std::string &name() const;
    coordinates &worldcoords();
    const coordinates &worldcoords() const;
    void copyWorldcoords(coordinates &w);

    void set(coordinates &c);
    RoboasmCoords *parent();
    bool hasParent();
    bool hasDescendants();
    // virtual ??
    void update();
    void updateDescendants();
    void assoc(RoboasmCoordsPtr c);
    bool dissoc(RoboasmCoordsPtr c);
    bool dissocParent();

    // ???
    void toRootList(coordsList &lst);
    void allDescendants(coordsList &lst);
    void allDescendants(coordsPtrList &lst);
    template <typename T> void allDescendants(coordsPtrList &lst);
    template <typename T> void allDescendants(std::vector< std::shared_ptr < T > >&lst);
    void toNextLink(); // ??
    void connectingPoints(coordsPtrList &lst);
    void connectingPoints(coordsPtrList &activelst, coordsPtrList &inactivelst);
    void activeConnectingPoints(coordsPtrList &lst);
    void inactiveConnectingPoints(coordsPtrList &lst);
    void actuators(coordsPtrList &lst);
    void actuators(coordsPtrList &activelst, coordsPtrList &inactivelst);
    void activeActuators(coordsPtrList &lst);
    void inactiveActuators(coordsPtrList &lst);
    void allParts(coordsPtrList &lst);
    RoboasmCoordsPtr find(const std::string &name);
    template <typename T> RoboasmCoordsPtr find(const std::string &name);

    enum ClassIDType {
        None,
        ID_Coords,
        ID_ConnectingPoint,
        ID_Parts,
        ID_Robot
    };
    //virtual ClassIDType classID();
protected:
    std::string name_str;
    //RoboasmCoordsPtr parent;
    RoboasmCoords *_parent;
    std::set<RoboasmCoordsPtr> descendants;
    //bool updated;
    coordinates _worldcoords;
    //void _replaceParent(RoboasmCoords *p);
    bool _dissoc(RoboasmCoords *c);
    bool _existing_descendant(RoboasmCoordsPtr c);
    bool _existing_descendant(RoboasmCoords *c);
    bool _erase_descendant(RoboasmCoords *c);
};

class RoboasmConnectingPoint : public RoboasmCoords
{
public:
    RoboasmConnectingPoint(const std::string &_name, ConnectingPoint *_info);
protected:
    ConnectingPoint *info;
    //void createFromInfo(ConnectingPoint *_info);
    //connecting point info
    //virtual ClassIDType classID() override;
    friend RoboasmCoords;
    friend RoboasmParts;
    friend RoboasmRobot;
};

class RoboasmParts : public RoboasmCoords
{
public:
    //RoboasmParts(const std::string &_name);
    RoboasmParts(const std::string &_name, Parts *_info);

protected:
    Parts *info;
    void createConnectingPoints();
    // parts info
    //virtual ClassIDType classID() override;
    friend RoboasmCoords;
    friend RoboasmConnectingPoint;
    friend RoboasmRobot;
};

class RoboasmRobot : public RoboasmCoords
{
public:
    //RoboasmRobot(const std::string &_name);
    RoboasmRobot(const std::string &_name, RoboasmPartsPtr parts);
    // robot info
    //virtual ClassIDType classID() override;

    bool attach(RoboasmRobotPtr robot,
                RoboasmConnectingPointPtr parts_point, //
                RoboasmConnectingPointPtr robot_point, //
                int configuration = 0, bool just_align = false);
    bool attach(RoboasmRobotPtr robot,
                RoboasmConnectingPointPtr parts_point, //
                RoboasmConnectingPointPtr robot_point, //
                coordinates &coords, bool just_align = false);

    bool attach(RoboasmPartsPtr parts,
                RoboasmConnectingPointPtr parts_point, //
                RoboasmConnectingPointPtr robot_point, //
                int configuration = 0, bool just_align = false);
    bool attach(RoboasmPartsPtr parts,
                RoboasmConnectingPointPtr parts_point, //
                RoboasmConnectingPointPtr robot_point, //
                coordinates &coords, bool just_align = false);

    bool attach(RoboasmCoordsPtr robot_or_parts,
                RoboasmConnectingPointPtr parts_point, //
                RoboasmConnectingPointPtr robot_point, //
                int configuration = 0, bool just_align = false);
    bool attach(RoboasmCoordsPtr robot_or_parts,
                RoboasmConnectingPointPtr parts_point, //
                RoboasmConnectingPointPtr robot_point, //
                coordinates &coords, bool just_align = false);

    friend RoboasmCoords;
    friend RoboasmParts;
    friend RoboasmConnectingPoint;
};

}; };

//typedef std::shared_ptr<TestBase> spBase;
//typedef std::shared_ptr<TestDerived> spDerived;
//spBase base = std::make_shared<TestDerived>();
//spDerived derived = std::dynamic_pointer_cast<spDerived::element_type>(base);
//// or:
//spDerived derived2 = std::dynamic_pointer_cast<TestDerived>(base);
