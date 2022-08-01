#include "RobotAssembler.h"

#include <iostream>
#include <sstream>
#include <algorithm> //std::find
#include <sstream>

// get pid
#include <sys/types.h>
#include <unistd.h>

using namespace cnoid;

namespace cnoid {
namespace robot_assembler {

//// [roboasm coords] ////
RoboasmCoords::RoboasmCoords(const std::string &_name)
    : parent_ptr(nullptr)
{
    name_str = _name;
}
const std::string &RoboasmCoords::name() const
{
    return name_str;
}
coordinates &RoboasmCoords::worldcoords()
{
    return buf_worldcoords;
}
const coordinates &RoboasmCoords::worldcoords() const {
    return buf_worldcoords;
}
void RoboasmCoords::copyWorldcoords(coordinates &w)
{
    w = buf_worldcoords;
}
void RoboasmCoords::newcoords(coordinates &c)
{
    pos = c.pos;
    rot = c.rot;
    update();
}
RoboasmCoords *RoboasmCoords::parent()
{
    return parent_ptr;
}
bool RoboasmCoords::hasParent()
{
    return (!!parent_ptr);
}
bool RoboasmCoords::hasDescendants()
{
    return (descendants.size() > 0);
}
// virtual ??
void RoboasmCoords::update()
{
    if(!!parent_ptr) {
#if 0
        std::cout << "pp : " << parent_ptr << std::endl;
        std::cout << "w0 ";
        print(buf_worldcoords); std::cout << std::endl;
#endif
        parent_ptr->copyWorldcoords(buf_worldcoords);
#if 0
        std::cout << "w1 ";
        print(buf_worldcoords); std::cout << std::endl;
        std::cout << "this ";
        print(*dynamic_cast<coordinates *>(this)); std::cout << std::endl;
#endif
        buf_worldcoords.transform(*this);
#if 0
        std::cout << "w2 ";
        print(buf_worldcoords); std::cout << std::endl;
#endif
    } else {
        buf_worldcoords = *this;
    }
}
void RoboasmCoords::updateDescendants()
{
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        (*it)->update();
    }
}
void RoboasmCoords::assoc(RoboasmCoordsPtr c)
{
    if ( ! _existing_descendant(c) ) {
        //c->_replaceParent(this);
        if (!!(c->parent_ptr)) {
            c->parent_ptr->_dissoc(c.get());
        }
        c->parent_ptr = this;
        coordinates newcoords;
        buf_worldcoords.transformation(newcoords, c->buf_worldcoords);
        c->newcoords(newcoords);
        descendants.insert(c);
    }
}
bool RoboasmCoords::dissoc(RoboasmCoordsPtr c)
{
    return _dissoc(c.get());
}
bool RoboasmCoords::_dissoc(RoboasmCoords *c)
{
    if (c->parent_ptr == this) {
        _erase_descendant(c);
        c->parent_ptr = nullptr;
        c->newcoords(c->buf_worldcoords);
        return true;
    }
    return false;
}
bool RoboasmCoords::dissocParent()
{
    if (!!parent_ptr) {
        return parent_ptr->_dissoc(this);
    }
    return false;
}
bool RoboasmCoords::isDirectDescendant(RoboasmCoordsPtr c)
{
    auto it = std::find(descendants.begin(), descendants.end(), c);
    return (it != descendants.end());
}
void RoboasmCoords::toRootList(coordsList &lst)
{
    lst.push_back(this);
    if (!!parent_ptr) {
        parent_ptr->toRootList(lst);
    }
}
void RoboasmCoords::allDescendants(coordsList &lst)
{
    // including self
    lst.push_back(this);
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        (*it)->allDescendants(lst);
    }
}
void RoboasmCoords::allDescendants(coordsPtrList &lst)
{
    //lst.push_back(this); // not including self
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        lst.push_back(*it);
        (*it)->allDescendants(lst);
    }
}
template <typename T>
void RoboasmCoords::allDescendants(coordsPtrList &lst)
{
    //lst.push_back(this); // not including self
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        std::shared_ptr<T> p = std::dynamic_pointer_cast<T> (*it);
        if (!!p) {
            lst.push_back(*it);
        }
        (*it)->allDescendants<T>(lst);
    }
}
template <typename T>
void RoboasmCoords::allDescendants(std::vector< std::shared_ptr < T > >&lst)
{
    //lst.push_back(this); // not including self
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        std::shared_ptr<T> p = std::dynamic_pointer_cast<T> (*it);
        if (!!p) {
            lst.push_back(p);
        }
        (*it)->allDescendants<T>(lst);
    }
}
// templateSettings
//template void RoboasmCoords::allDescendants<RoboasmCoords>(coordsPtrList &lst);
template void RoboasmCoords::allDescendants<RoboasmConnectingPoint>(coordsPtrList &lst);
template void RoboasmCoords::allDescendants<RoboasmParts>(coordsPtrList &lst);
//template void RoboasmCoords::allDescendants<RoboasmRobot>(coordsPtrList &lst);
template void RoboasmCoords::allDescendants<RoboasmConnectingPoint>(connectingPointPtrList &lst);
template void RoboasmCoords::allDescendants<RoboasmParts>(partsPtrList &lst);

void RoboasmCoords::toNextLink(){}
void RoboasmCoords::connectingPoints(coordsPtrList &lst)
{
    allDescendants<RoboasmConnectingPoint> (lst);
}
void RoboasmCoords::connectingPoints(coordsPtrList &activelst,
                                     coordsPtrList &inactivelst)
{
    coordsPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if (!(*it)->hasDescendants()) {
            activelst.push_back(*it);
        } else {
            inactivelst.push_back(*it);
        }
    }
}
void RoboasmCoords::activeConnectingPoints(coordsPtrList &lst)
{
    coordsPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if (!(*it)->hasDescendants()) {
            lst.push_back(*it);
        }
    }
}
void RoboasmCoords::inactiveConnectingPoints(coordsPtrList &lst)
{
    coordsPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if ((*it)->hasDescendants()) {
            lst.push_back(*it);
        }
    }
}
void RoboasmCoords::actuators(coordsPtrList &lst)
{
    connectingPointPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if ((*it)->isActuator()) {
            lst.push_back(*it);
        }
    }
}
void RoboasmCoords::actuators(coordsPtrList &activelst,
                              coordsPtrList &inactivelst)
{
    connectingPointPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if ((*it)->isActuator()) {
            if(!(*it)->hasDescendants()) {
                activelst.push_back(*it);
            } else {
                inactivelst.push_back(*it);
            }
        }
    }
}
void RoboasmCoords::activeActuators(coordsPtrList &lst)
{
    connectingPointPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if ((*it)->isActuator() &&
            !(*it)->hasDescendants()) {
            lst.push_back(*it);
        }
    }
}
void RoboasmCoords::inactiveActuators(coordsPtrList &lst)
{
    connectingPointPtrList tmp;
    allDescendants<RoboasmConnectingPoint> (tmp);
    for(auto it = tmp.begin(); it != tmp.end(); it++) {
        if ((*it)->isActuator() &&
            (*it)->hasDescendants()) {
            lst.push_back(*it);
        }
    }
}
void RoboasmCoords::allParts(coordsPtrList &lst)
{
    allDescendants<RoboasmParts> (lst);
}
RoboasmCoordsPtr RoboasmCoords::find(const std::string &name)
{
    RoboasmCoordsPtr ret;
    coordsPtrList lst;
    allDescendants (lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        if ((*it)->name() == name) {
            ret = *it;
            break;
        }
    }
    return ret;
}
template <typename T>
RoboasmCoordsPtr RoboasmCoords::find(const std::string &name)
{
    RoboasmCoordsPtr ret;
    coordsPtrList lst;
    allDescendants<T> (lst);
    for(auto it = lst.begin(); it != lst.end(); it++) {
        if ((*it)->name() == name) {
            ret = *it;
            break;
        }
    }
    return ret;
}
// template settings
//template RoboasmCoords RoboasmCoords::find<RoboasmCoords>(const std::string &name); // use not template function
template RoboasmCoordsPtr RoboasmCoords::find<RoboasmParts>(const std::string &name);
template RoboasmCoordsPtr RoboasmCoords::find<RoboasmConnectingPoint>(const std::string &name);
//template RoboasmCoords RoboasmCoords::find<RoboasmRobot>(const std::string &name); //
// protected
#if 0
void RoboasmCoords::_replaceParent(RoboasmCoords *p)
{
    if (!!parent_ptr) {
        parent_ptr->dissoc(this);
        parent_ptr = p;
    } else {
        parent_ptr = p;
    }
}
#endif
bool RoboasmCoords::_existing_descendant(RoboasmCoordsPtr c)
{
    auto res = descendants.find(c);
    if (res != descendants.end()) {
        return true;
    }
    return false;
}
bool RoboasmCoords::_existing_descendant(RoboasmCoords *c)
{
    auto res = descendants.end();
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        if ((*it).get() == c) {
            res = it;
            break;
        }
    }
    if (res != descendants.end()) {
        return true;
    }
    return  false;
}
bool RoboasmCoords::_erase_descendant(RoboasmCoords *c)
{
    auto res = descendants.end();
    for(auto it = descendants.begin(); it != descendants.end(); it++) {
        if ((*it).get() == c) {
            res = it;
            break;
        }
    }
    if (res != descendants.end()) {
        descendants.erase(res);
        return true;
    }
    return false;
}

//// [roboasm connecting point] ////
RoboasmConnectingPoint::RoboasmConnectingPoint(const std::string &_name,
                                               ConnectingPoint *_info)
    : RoboasmCoords(_name), current_type_match(nullptr), current_configuration(-1)
{
    info = _info;
}
bool RoboasmConnectingPoint::isActuator()
{
    if (info->getType() != ConnectingPoint::Parts) {
        return true;
    }
    return false;
}
bool RoboasmConnectingPoint::hasConfiguration()
{
    return (current_configuration >= 0 || configuration_str.size() > 0);
}
const std::string &RoboasmConnectingPoint::currentConfiguration()
{
    return configuration_str;
}
bool RoboasmConnectingPoint::definedConfiguration()
{
    return (current_configuration >= 0);
}
ConnectingConfigurationID RoboasmConnectingPoint::configurationType()
{
    return current_configuration;
}
void RoboasmConnectingPoint::configurationCoords(coordinates &_coords)
{
    _coords = configuration_coords;
}

//// [roboasm connecting parts] ////
RoboasmParts::RoboasmParts(const std::string &_name, Parts *_info)
    : RoboasmCoords(_name)
{
    info = _info;
    createConnectingPoints();
}
void RoboasmParts::createConnectingPoints(bool use_name_as_namespace)
{
    std::string namespace_;
    if(use_name_as_namespace) {
        namespace_ = name();
    }
    createConnectingPoints(namespace_);
}
void RoboasmParts::createConnectingPoints(const std::string &_namespace)
{
    for(auto it = info->connecting_points.begin();
        it != info->connecting_points.end(); it++) {
        assocConnectingPoint(&(*it), _namespace);
    }
    for(auto it = info->actuators.begin();
        it != info->actuators.end(); it++) {
        assocConnectingPoint(&(*it), _namespace);
    }
    updateDescendants();
}
void RoboasmParts::assocConnectingPoint(ConnectingPoint* cp, const std::string &_namespace)
{
    std::string nm;
    if (_namespace.size() > 0) {
        nm = _namespace + "/" + cp->name;
    } else {
        nm = cp->name;
    }
    RoboasmCoordsPtr ptr = std::make_shared<RoboasmConnectingPoint> (nm, cp);
    ptr->newcoords(cp->coords);
    this->assoc(ptr);
}

//// [roboasm connecting robot] ////
RoboasmRobot::RoboasmRobot(const std::string &_name, RoboasmPartsPtr parts,
                           SettingsPtr _settings)
    : RoboasmCoords(_name), settings(_settings)
{
    assoc(parts);
    updateDescendants();
}

bool RoboasmRobot::reverseParentChild(RoboasmPartsPtr _parent, RoboasmConnectingPointPtr _chld)
{
    // check _chld is descendants of _parent
    if(!_parent->isDirectDescendant(_chld)) {
        // [todo]
        return false;
    }
    // check _chld has no descendants
    if(_chld->hasDescendants()) {
        // [todo]
        return false;
    }
    // check _parent has no parent
    if(_parent->hasParent()) {
        // [todo]
        return false;
    }
    _chld->dissocParent();
    _chld->assoc(_parent);
    return true;
}
#if 0
bool RoboasmRobot::checkAttachByName(RoboasmCoordsPtr robot_or_parts,
                                     const std::string &name_parts_point,
                                     const std::string &name_robot_point,
                                     const std::string &name_config,
                                     RoboasmConnectingPointPtr &_res_parts_point,
                                     RoboasmConnectingPointPtr &_res_robot_point,
                                     *_res_config);
                                     //configuration);
{
    _res_robot_point = std::dynamic_pointer_cast<RoboasmConnectingPoint> (
        find<RoboasmConnectingPoint>(name_robot_point));
    if(_res_robot_point->hasDescendants()) {
        // [todo]
        return false;
    }
    _res_parts_point = std::dynamic_pointer_cast<RoboasmConnectingPoint> (
        robot_or_parts->find<RoboasmConnectingPoint>(name_parts_point));
    if(_res_parts_point->hasDescendants()) {
        // [todo]
        return false;
    }
    // search configuration
    std::vector<ConnectingTypeID> &rtp = _res_robot_point->info->type_list;
    std::vector<ConnectingTypeID> &ptp = _res_parts_point->info->type_list;

    ConnectingTypeMatch *tm_ = nullptr;
    ConnectingConfiguration *cc_ = nullptr;
    bool match_ = false;
    for(int i = 0; i < rtp.size(); i++) {
        for(int j = 0; j < ptp.size(); j++) {
            tm_ = conf->searchConnection(rtp[i], ptp[j], name_config, cc_);
            if (!!tm_ && !!cc_) {
                match_ = true;
                break;
            }
        }
    }
    if (!match_) {
        // [todo]
        return false;
    }

    return true;
}
#endif
#if 0
bool RoboasmRobot::attachXX(RoboasmCoordsPtr robot_or_parts,
                            RoboasmConnectingPointPtr parts_point, //
                            RoboasmConnectingPointPtr robot_point, //
                            int configuration = 0, bool just_align = false)
{
    coordsPtrList plst;
    robot_or_parts.activeConnectingPoints(plst);
    auto resp = std::find(plst.begin(), plst.end(), parts_point);
    if (resp == plst.end()) {
        // [todo]
        return false;
    }
    coordsPtrList rlst;
    activeConnectingPoints(rlst);
    auto resr = std::find(rlst.begin(), rlst.end(), robot_point);
    if( resr == rlst.end()) {
        // [todo]
        return false;
    }
}
#endif

//// [Roboasm] ////
Roboasm::Roboasm(const std::string &filename)
{
    SettingsPtr p = std::make_shared<Settings>();
    if (p->parseYaml(filename)) {
        parts_counter = 0;
        pid = getpid();
        current_settings = p;
    } else {
        current_settings = nullptr;
    }
}
bool Roboasm::isReady()
{
    return (!!current_settings);
}
RoboasmPartsPtr Roboasm::makeParts(const std::string &_parts_key)
{
    std::ostringstream os;
    os << _parts_key << "_" << pid << "_" << parts_counter;
    parts_counter++;
    return makeParts(_parts_key, os.str());
}
RoboasmPartsPtr Roboasm::makeParts
(const std::string &_parts_key, const std::string &_parts_name)
{
    auto res = current_settings->mapParts.find(_parts_key);
    if (res == current_settings->mapParts.end()) {
        return nullptr;
    }
    RoboasmPartsPtr ret = std::make_shared<RoboasmParts> (_parts_name, &(res->second));
    return ret;
}
RoboasmRobotPtr Roboasm::makeRobot(const std::string &_name, const std::string &_parts_key)
{
    RoboasmPartsPtr pt_ = makeParts(_parts_key);
    return std::make_shared<RoboasmRobot>(_name, pt_, current_settings);
}
RoboasmRobotPtr Roboasm::makeRobot(const std::string &_name, const std::string &_parts_key,
                                          const std::string &_parts_name)
{
    RoboasmPartsPtr pt_ = makeParts(_parts_key, _parts_name);
    return std::make_shared<RoboasmRobot>(_name, pt_, current_settings);
}
RoboasmRobotPtr Roboasm::makeRobot(const std::string &_name, RoboasmPartsPtr _parts)
{
    return std::make_shared<RoboasmRobot>(_name, _parts, current_settings);
}

} }
