#include "RobotAssemblerSettings.h"
#include <irsl_choreonoid/Coordinates.h>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <set>

#pragma once

namespace cnoid {
namespace robot_assembler {

// [Roboasm] classes
class RoboasmCoords;
class RoboasmConnectingPoint;
class RoboasmParts;
class RoboasmRobot;
class Roboasm;
typedef std::shared_ptr<RoboasmCoords> RoboasmCoordsPtr;
typedef std::shared_ptr<RoboasmParts> RoboasmPartsPtr;
typedef std::shared_ptr<RoboasmConnectingPoint> RoboasmConnectingPointPtr;
typedef std::shared_ptr<RoboasmRobot> RoboasmRobotPtr;
////
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

    void newcoords(coordinates &c);
    RoboasmCoords *parent();
    bool hasParent();
    bool hasDescendants();
    // virtual ??
    void update();
    void updateDescendants();
    void assoc(RoboasmCoordsPtr c);
    bool dissoc(RoboasmCoordsPtr c);
    bool dissocParent();
    bool isDirectDescendant(RoboasmCoordsPtr c);
    bool isDescendant(RoboasmCoordsPtr c);
    // ???
    void toRootList(coordsList &lst);
    void directDescendants(coordsPtrList &lst);
    void allDescendants(coordsList &lst);
    void allDescendants(coordsPtrList &lst);
    template <typename T> void allDescendants(coordsPtrList &lst);
    template <typename T> void allDescendants(std::vector< std::shared_ptr < T > >&lst);
    void toNextLink(); // ??
    //// point
    void connectingPoints(connectingPointPtrList &lst);
    void connectingPoints(connectingPointPtrList &activelst, connectingPointPtrList &inactivelst);
    void activeConnectingPoints(connectingPointPtrList &lst);
    void inactiveConnectingPoints(connectingPointPtrList &lst);
    void actuators(connectingPointPtrList &lst);
    void actuators(connectingPointPtrList &activelst, connectingPointPtrList &inactivelst);
    void activeActuators(connectingPointPtrList &lst);
    void inactiveActuators(connectingPointPtrList &lst);
    //// parts
    void allParts(partsPtrList &lst);
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
    RoboasmCoords *parent_ptr;
    std::set<RoboasmCoordsPtr> descendants;
    //bool updated;
    coordinates buf_worldcoords;
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

    // inline??
    bool isActuator();
    bool hasConfiguration();
    const std::string &currentConfiguration();
    bool definedConfiguration();
    ConnectingConfigurationID configurationID();
    void configurationCoords(coordinates &_coords);

    //bool isActive();
    ConnectingPoint *info;
protected:
    //void createFromInfo(ConnectingPoint *_info);
    //connecting point info
    //virtual ClassIDType classID() override;

    ConnectingConfigurationID current_configuration_id;
    std::string configuration_str;
    coordinates configuration_coords;
    ConnectingConfiguration *current_configuration;
    ConnectingTypeMatch *current_type_match;

    friend RoboasmCoords;
    friend RoboasmParts;
    friend RoboasmRobot;
};

class RoboasmParts : public RoboasmCoords
{
public:
    //RoboasmParts(const std::string &_name);
    RoboasmParts(const std::string &_name, Parts *_info);

    Parts *info;
protected:
    void createConnectingPoints(const std::string &_namespace);
    void createConnectingPoints(bool use_name_as_namespace = true);

    void assocConnectingPoint(ConnectingPoint* cp, const std::string &_namespace);
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
    RoboasmRobot() = delete;
    RoboasmRobot(const std::string &_name,
                 RoboasmPartsPtr parts,
                 SettingsPtr _settings);

    // robot info
    //virtual ClassIDType classID() override;
    bool reverseParentChild(RoboasmPartsPtr _parent, RoboasmConnectingPointPtr _chld);
    bool checkCorrectPoint(RoboasmCoordsPtr robot_or_parts,
                           RoboasmConnectingPointPtr _parts_point, RoboasmConnectingPointPtr _robot_point);
    bool checkAttachByName(RoboasmCoordsPtr robot_or_parts,
                           const std::string &name_parts_point,
                           const std::string &name_robot_point,
                           const std::string &name_config,
                           RoboasmConnectingPointPtr &_res_parts_point,
                           RoboasmConnectingPointPtr &_res_robot_point,
                           ConnectingConfiguration * &_res_config,
                           ConnectingTypeMatch * &_res_match);
    bool checkAttach(RoboasmCoordsPtr robot_or_parts,
                     RoboasmConnectingPointPtr _parts_point,
                     RoboasmConnectingPointPtr _robot_point,
                     ConnectingConfiguration *_config,
                     ConnectingTypeMatch * &_res_match, bool check = true);
    bool searchMatch(RoboasmCoordsPtr robot_or_parts,
                     RoboasmConnectingPointPtr _parts_point,
                     RoboasmConnectingPointPtr _robot_point,
                     std::vector<ConnectingTypeMatch*> &_res_match_lst, bool check = true);
    bool attach(RoboasmCoordsPtr robot_or_parts,
                RoboasmConnectingPointPtr _parts_point,
                RoboasmConnectingPointPtr _robot_point,
                coordinates &_conf_coords,
                bool just_align = false) {
        return attach(robot_or_parts, _parts_point, _robot_point,
                      _conf_coords, nullptr, nullptr, just_align);
    }
    bool attach(RoboasmCoordsPtr robot_or_parts,
                RoboasmConnectingPointPtr _parts_point,
                RoboasmConnectingPointPtr _robot_point,
                ConnectingConfiguration *_config,
                bool just_align = false) {
        coordinates tmp;
        return attach(robot_or_parts, _parts_point, _robot_point,
                      tmp, _config, nullptr, just_align);
    }
    bool attach(RoboasmCoordsPtr robot_or_parts,
                RoboasmConnectingPointPtr _parts_point,
                RoboasmConnectingPointPtr _robot_point,
                coordinates &_conf_coords,
                ConnectingConfiguration *_config = nullptr,
                ConnectingTypeMatch *_match = nullptr,
                bool just_align = false);
    // attch by name
protected:

    SettingsPtr settings;
    friend RoboasmCoords;
    friend RoboasmParts;
    friend RoboasmConnectingPoint;
};

class Roboasm
{
public:
    Roboasm() = delete;
    Roboasm(const std::string &filename);
    bool isReady();

    RoboasmPartsPtr makeParts(const std::string &_parts_key);
    RoboasmPartsPtr makeParts(const std::string &_parts_key, const std::string &_parts_name);

    RoboasmRobotPtr makeRobot(const std::string &_name, const std::string &_parts_key);
    RoboasmRobotPtr makeRobot(const std::string &_name, const std::string &_parts_key, const std::string &_parts_name);
    RoboasmRobotPtr makeRobot(const std::string &_name, RoboasmPartsPtr _parts);

private:
    //class Impl;
    //static Impl *impl;
    SettingsPtr current_settings;

    int pid;
    int parts_counter;
};

} }
