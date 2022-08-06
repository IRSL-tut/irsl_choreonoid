#include "RobotAssemblerSettings.h"
#include <irsl_choreonoid/Coordinates.h>
#include <string>
#include <vector>
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
    ~RoboasmCoords();
    const std::string &name() const { return name_str; }
    coordinates &worldcoords() { return buf_worldcoords; }
    const coordinates &worldcoords() const {  return buf_worldcoords; }
    void copyWorldcoords(coordinates &w) { w = buf_worldcoords; }
    void newcoords(coordinates &c) {  pos = c.pos; rot = c.rot; update(); }
    RoboasmCoords *parent() { return parent_ptr; }
    bool hasParent() { return (!!parent_ptr); }
    bool hasDescendants() { return (descendants.size() > 0); }
    // virtual ??
    void update();
    void updateDescendants() {
        for(auto it = descendants.begin(); it != descendants.end(); it++) {
        (*it)->update();
        }
    }
    void assoc(RoboasmCoordsPtr c);
    bool dissoc(RoboasmCoordsPtr c) { return _dissoc(c.get()); }
    bool dissocFromParent() {
        if (!!parent_ptr) {
            return parent_ptr->_dissoc(this);
        }
        return false;
    }
    bool isDirectDescendant(RoboasmCoordsPtr c) {
        auto it = std::find(descendants.begin(), descendants.end(), c);
        return (it != descendants.end());
    }
    RoboasmCoordsPtr isDirectDescendant(RoboasmCoords* c) {
        for(auto it = descendants.begin(); it != descendants.end(); it++) {
            if((*it).get() == c) return *it;
        }
        return nullptr;
    }
    bool isDescendant(RoboasmCoordsPtr c);
    RoboasmCoordsPtr isDescendant(RoboasmCoords *c);
    // ???
    void toRootList(coordsList &lst); // self to root
    void toRootList(coordsPtrList &lst);
    void directDescendants(coordsPtrList &lst);
    template <typename T> void directDescendants(std::vector< std::shared_ptr < T > >&lst);
    void allDescendants(coordsList &lst);
    void allDescendants(coordsPtrList &lst);
    template <typename T> void allDescendants(coordsPtrList &lst);
    template <typename T> void allDescendants(std::vector< std::shared_ptr < T > >&lst);
    void toNextLink(); // ??
    //// point
    void connectingPoints(connectingPointPtrList &lst) {
        allDescendants<RoboasmConnectingPoint> (lst);
    }
    void connectingPoints(connectingPointPtrList &activelst, connectingPointPtrList &inactivelst);
    void activeConnectingPoints(connectingPointPtrList &lst);
    void inactiveConnectingPoints(connectingPointPtrList &lst);
    void actuators(connectingPointPtrList &lst);
    void actuators(connectingPointPtrList &activelst, connectingPointPtrList &inactivelst);
    void activeActuators(connectingPointPtrList &lst);
    void inactiveActuators(connectingPointPtrList &lst);
    //// parts
    void allParts(partsPtrList &lst) {
        allDescendants<RoboasmParts> (lst);
    }
    RoboasmCoordsPtr find(const std::string &name);
    template <typename T> RoboasmCoordsPtr find(const std::string &name);
    bool isConnectingPoint(); // inline
    bool isActuator(); // inline
    bool isParts(); // inline
    bool isRobot(); // inline
    RoboasmConnectingPoint *toConnectingPoint(); // inline
    RoboasmParts *toParts(); // inline
    RoboasmRobot *toRobot(); // inline
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

    friend std::ostream& operator<< (std::ostream& ostr, const RoboasmCoords &output);
    friend RoboasmConnectingPoint;
    friend RoboasmParts;
    friend RoboasmRobot;
};

class RoboasmConnectingPoint : public RoboasmCoords
{
public:
    RoboasmConnectingPoint(const std::string &_name, ConnectingPoint *_info);
    ~RoboasmConnectingPoint();
    bool checkValidity();
    // inline??
    bool isActuator() {
        if (info->getType() != ConnectingPoint::Parts) {
            return true;
        }
        return false;
    }
    bool hasConfiguration() {
        return (current_configuration_id >= 0 || configuration_str.size() > 0);
    }
    const std::string &currentConfiguration() {
        return configuration_str;
    }
    bool definedConfiguration() {
        return (current_configuration_id >= 0);
    }
    ConnectingConfigurationID configurationID() {
        return current_configuration_id;
    }
    void configurationCoords(coordinates &_coords) {
        _coords = configuration_coords;
    }

    bool isActive() { return (descendants.size() < 1); }
    bool isConnected() { return (descendants.size() > 0); }
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
    ~RoboasmParts();
    bool checkValidity();
    bool dumpConnectFromParent(AttachHistory &history);
    void childParts(partsPtrList &lst);  // parts of just direct child
    bool parentParts(RoboasmPartsPtr &_res_parent,
                     RoboasmConnectingPointPtr &_res_parent_point,
                     RoboasmConnectingPointPtr &_res_self_point);
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
    ~RoboasmRobot();
    // robot info
    //virtual ClassIDType classID() override;
    bool reverseParentChild(RoboasmPartsPtr _parent, RoboasmConnectingPointPtr _chld); // static method
    bool changeRoot(RoboasmConnectingPointPtr _chld); // static method
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
                ConnectingConfigurationID _config_id,
                bool just_align = false) {
        ConnectingConfiguration &_config = settings->listConnectingConfiguration[_config_id];
        coordinates tmp;
        return attach(robot_or_parts, _parts_point, _robot_point,
                      tmp, &_config, nullptr, just_align);
    }
    bool attach(RoboasmCoordsPtr robot_or_parts,
                const std::string &name_parts_point,
                const std::string &name_robot_point,
                const std::string &name_config, bool just_align = false) {
        RoboasmConnectingPointPtr _res_parts_point;
        RoboasmConnectingPointPtr _res_robot_point;
        ConnectingConfiguration *_res_config;
        ConnectingTypeMatch * _res_match;
        if (! checkAttachByName(robot_or_parts,
                                name_parts_point, name_robot_point,
                                name_config,
                                _res_parts_point, _res_robot_point,
                                _res_config, _res_match) )
        {
            return false;
        }
        return attach(robot_or_parts, _res_parts_point, _res_robot_point, _res_config, just_align);
    }
    bool attach(RoboasmCoordsPtr robot_or_parts,
                RoboasmConnectingPointPtr _parts_point,
                RoboasmConnectingPointPtr _robot_point,
                coordinates &_conf_coords,
                ConnectingConfiguration *_config = nullptr,
                ConnectingTypeMatch *_match = nullptr,
                bool just_align = false);

    // RoboasmRobotPtr detach(RoboasmPartsPtr _parts);
    size_t partsNum() {
        partsPtrList lst;
        allParts(lst);
        return lst.size();
    }
    size_t connectingPointNum() {
        connectingPointPtrList lst;
        connectingPoints(lst);
        return lst.size();
    }
    bool checkValidity();
    RoboasmPartsPtr rootParts() {
      partsPtrList plst;
      directDescendants<RoboasmParts>(plst);
      if(plst.size() == 1) {
        return plst[0];
      } else if (plst.size() > 1) {
          // not valid (many child parts)
      } else {
          // not valid (no child parts)
      }
      return nullptr;
    }
    bool createRoboasm(RoboasmFile &_roboasm);
    void connectedPoints(connectingPointPtrList &lst);
protected:
    SettingsPtr settings;
    AssembleConfig asm_config;
    friend RoboasmCoords;
    friend RoboasmParts;
    friend RoboasmConnectingPoint;
    bool writeConfig(AssembleConfig &_config);
};

class Roboasm
{
public:
    Roboasm() = delete;
    Roboasm(const std::string &filename);
    Roboasm(SettingsPtr settings);
    ~Roboasm();
    bool isReady();
    static RoboasmConnectingPointPtr toConnectingPoint(RoboasmCoordsPtr p) {
        return std::dynamic_pointer_cast<RoboasmConnectingPoint>(p);
    }
    static RoboasmPartsPtr toParts(RoboasmCoordsPtr p) {
        return std::dynamic_pointer_cast<RoboasmParts>(p);
    }
    static RoboasmRobotPtr toRobot(RoboasmCoordsPtr p) {
        return std::dynamic_pointer_cast<RoboasmRobot>(p);
    }
    RoboasmPartsPtr makeParts(const std::string &_parts_key);
    RoboasmPartsPtr makeParts(const std::string &_parts_key, const std::string &_parts_name);

    RoboasmRobotPtr makeRobot(const std::string &_name, const std::string &_parts_key);
    RoboasmRobotPtr makeRobot(const std::string &_name, const std::string &_parts_key, const std::string &_parts_name);
    RoboasmRobotPtr makeRobot(const std::string &_name, RoboasmPartsPtr _parts);

    RoboasmRobotPtr makeRobot(RoboasmFile &_roboasm_file);
    RoboasmRobotPtr makeRobotFromFile(const std::string &_filename) {
        RoboasmFile roboasm(_filename);
        if(roboasm.isValid()) return makeRobot(roboasm);
        return nullptr;
    }

    bool canMatch(RoboasmConnectingPointPtr _a, RoboasmConnectingPointPtr _b);
private:
    //class Impl;
    //static Impl *impl;
    SettingsPtr current_settings;

    int pid;
    int parts_counter;
};
typedef std::shared_ptr<Roboasm> RoboasmPtr;

// inline
inline bool RoboasmCoords::isConnectingPoint() {
    RoboasmConnectingPoint *ptr = dynamic_cast<RoboasmConnectingPoint*>(this);
    if(!!ptr) return true;
    return false;
}
inline bool RoboasmCoords::isActuator() {
    RoboasmConnectingPoint *ptr = dynamic_cast<RoboasmConnectingPoint*>(this);
    if(!ptr) return false;
    return ptr->isActuator();
}
inline bool RoboasmCoords::isParts() {
    RoboasmParts *ptr = dynamic_cast<RoboasmParts*>(this);
    if(!!ptr) return true;
    return false;
}
inline bool RoboasmCoords::isRobot() {
    RoboasmRobot *ptr = dynamic_cast<RoboasmRobot*>(this);
    if(!!ptr) return true;
    return false;
}
inline RoboasmConnectingPoint *RoboasmCoords::toConnectingPoint() {
    return dynamic_cast<RoboasmConnectingPoint*>(this);
}
inline RoboasmParts *RoboasmCoords::toParts() {
    return dynamic_cast<RoboasmParts*>(this);
}
inline RoboasmRobot *RoboasmCoords::toRobot() {
    return dynamic_cast<RoboasmRobot*>(this);
}
std::ostream& operator<< (std::ostream& ostr, const cnoid::coordinates &output);
std::ostream& operator<< (std::ostream& ostr, const cnoid::robot_assembler::RoboasmCoords &output);
} }
