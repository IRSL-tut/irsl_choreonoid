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

    bool isActuator();
    bool hasConfiguration();
    const std::string &currentConfiguration();
    bool definedConfiguration();
    ConnectingConfigurationID configurationID();
    void configurationCoords(coordinates &_coords);

protected:
    ConnectingPoint *info;
    //void createFromInfo(ConnectingPoint *_info);
    //connecting point info
    //virtual ClassIDType classID() override;

    ConnectingConfigurationID current_configuration;
    std::string configuration_str;
    coordinates configuration_coords;
    ConnectingTypeMatch *current_type_match;
    ConnectingConfiguration *current_configuration_ptr;

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
                           ConnectingTypeMatch *_res_match,
                           ConnectingConfiguration *_res_config);
    bool checkAttach(RoboasmCoordsPtr robot_or_parts,
                     RoboasmConnectingPointPtr _parts_point,
                     RoboasmConnectingPointPtr _robot_point,
                     ConnectingConfiguration *_config,
                     ConnectingTypeMatch *_res_match, bool check = true);
    bool searchMatch(RoboasmCoordsPtr robot_or_parts,
                     RoboasmConnectingPointPtr _parts_point,
                     RoboasmConnectingPointPtr _robot_point,
                     std::vector<ConnectingTypeMatch*> &_res_match_lst, bool check = true);

    bool attach(RoboasmCoordsPtr robot_or_parts,
                RoboasmConnectingPointPtr _parts_point,
                RoboasmConnectingPointPtr _robot_point,
                ConnectingTypeMatch *_match,
                ConnectingConfiguration *_config, bool just_align = false);

#if 0
    bool attachXX(RoboasmRobotPtr robot,
                RoboasmConnectingPointPtr parts_point, //
                RoboasmConnectingPointPtr robot_point, //
                int configuration = 0, bool just_align = false);

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
#endif
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
