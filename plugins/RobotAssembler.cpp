#include "RobotAssembler.h"
#include <cnoid/YAMLReader>
#include <iostream>
#include <cmath>

using namespace cnoid;

namespace {
class UnitConfig
{
public:
    enum Unit {
        None,
        radian,
        degree,
        m,
        mm,
        g,
        kg,
    };
public:
    UnitConfig() : angle_scale(1.0), length_scale(1.0), mass_scale(1.0), inertia_scale(1.0) { }

    void setScale(Unit angle, Unit length, Unit mass)
    {
        if (angle == degree) {
            // degree to radian
            angle_scale = M_PI/180.0;
        }
        if (length == mm) {
            // mm to m
            length_scale = 0.001;
        }
        if (mass == g) {
            // g to kg
            mass_scale = 0.001;
        }
        inertia_scale = mass_scale * length_scale * length_scale;
    }

    // angle in config to [rad]
    double convAngle(double config_angle) {
        return (config_angle * angle_scale);
    }
    // length in config to [m]
    double convLength(double config_length) {
        return (config_length * length_scale);
    }
    // mass in config to [kg]
    double convMass(double config_mass) {
        return (config_mass * mass_scale);
    }
    // inertia in config to [kg*m^2]
    double convInertia(double config_inertia) {
        return (config_inertia * inertia_scale);
    }
    // length vector3 -> vector3
    void convPoint(const Vector3 &in, Vector3 &out) {
        out = length_scale * in;
    }
    void convPoint(const std::vector<double> &in, Vector3 &out) {
        Vector3 vec(in[0], in[1], in[2]);
        convPoint(vec, out);
    }
    // vector4 -> angleAxis
    void convRotationAngle(const Vector4 &in, AngleAxis &out) {
        Vector3 ax_(in(0), in(1), in(2));
        double an_ = convAngle(in(3));
        AngleAxis aa_(an_, ax_);
        out = aa_;
    }
    void convRotationAngle(const std::vector<double> &in, AngleAxis &out) {
        Vector4 vec(in[0], in[1], in[2], in[3]);
        convRotationAngle(vec, out);
    }
    // Matrix3 -> Matrix3
    void convInertiaTensor(const Matrix3 &in, Matrix3 &out) {
        out = inertia_scale * in;
    }
    void convInertiaTensor(const std::vector<double> in, Matrix3 &out) {
        Matrix3 mat = Matrix3::Zero();
        if (in.size() >= 9) {
            mat(0, 0) = in[0];
        } else if (in.size() >= 6) {
            mat(0, 0) = in[0];
        }
        convInertiaTensor(mat, out);
    }
private:
    double angle_scale;
    double length_scale;
    double mass_scale;
    double inertia_scale;
};

void printNode(std::ostream &out, ValueNode *vn, int indent = 0)
{
    if (vn->isScalar()) {
        for(int i = 0; i < indent; i++) out << "  ";
        out << vn->toString() << std::endl;
    } else if (vn->isListing()) {
        Listing *lst = vn->toListing();
        for(int i = 0; i < indent; i++) out << "  ";
        out << "[" << std::endl;
        for(int i = 0; i < lst->size(); i++) {
            printNode(out, lst->at(i), indent + 1);
        }
        for(int i = 0; i < indent; i++) out << "  ";
        out << "]" << std::endl;
    } else if (vn->isMapping()) {
        for(int i = 0; i < indent; i++) out << "  ";
        out << "{" << std::endl;
        Mapping *mp = vn->toMapping();
        for(auto it = mp->begin(); it != mp->end(); it++) {
            for(int i = 0; i < indent; i++) out << "  ";
            out << it->first << ":" << std::endl;
            printNode(out, it->second, indent + 1);
        }
        for(int i = 0; i < indent; i++) out << "  ";
        out << "}" << std::endl;
    } else {
        out << "type error???" << std::endl;
    }
}

bool readString(ValueNode *val, std::string &str, std::ostream &out = std::cerr, bool require = true)
{
    if(!val->isScalar()) {
        out << "<<scalar expected but:" << std::endl;
        printNode(out, val);
        out << ">>" << std::endl;
        if (!require) return true;
        return false;
    }
    str = val->toString();
    return true;
}
bool readDouble(ValueNode *val, double &dbl, std::ostream &out = std::cerr, bool require = true)
{
    if(!val->isScalar()) {
        out << "<<scalar expected but:" << std::endl;
        printNode(out, val);
        out << ">>" << std::endl;
        if (!require) return true;
        return false;
    }
    dbl = val->toDouble();
    return true;
}
bool readVector(ValueNode *val, std::vector<double> &vec, std::ostream &out = std::cerr, bool require = true)
{
    if( ! val->isListing() ) {
        out << "<<list expected but:" << std::endl;
        printNode(out, val);
        out << ">>" << std::endl;
        if (!require) return true;
        return false;
    }
    Listing *lst = val->toListing();
    for(int i = 0; i < lst->size(); i++) {
        if(lst->at(i)->isScalar()) {
            vec.push_back(lst->at(i)->toDouble());
        }
    }
    return true;
}
bool mapString(Mapping *mp, const std::string &key, std::string &str, std::ostream &out = std::cerr, bool require = true)
{
    ValueNode *val = mp->find(key);
    if( ! val->isValid() ) {
        if (!require) return true;
        out << "<< key " << key << " not found:" << std::endl;
        printNode(out, mp);
        out << ">>" << std::endl;
        return false;
    }
    return readString(val, str, out, require);
}
bool mapDouble(Mapping *mp, const std::string &key, double &dbl, std::ostream &out = std::cerr, bool require = true)
{
    ValueNode *val = mp->find(key);
    if( ! val->isValid() ) {
        if (!require) return true;
        out << "<< key " << key << " not found:" << std::endl;
        printNode(out, mp);
        out << ">>" << std::endl;
        return false;
    }
    return readDouble(val, dbl, out, require);
}
bool mapVector(Mapping *mp, const std::string &key, std::vector<double> &vec,
               std::ostream &out = std::cerr, bool require = true)
{
    ValueNode *val = mp->find(key);
    if( ! val->isValid() ) {
        if (!require) return true;
        out << "<< key " << key << " not found:" << std::endl;
        printNode(out, mp);
        out << ">>" << std::endl;
        return false;
    }
    return readVector(val, vec, out, require);
}

};

namespace cnoid {
namespace robot_assembler {

class RobotAssemblerConfiguration::Impl
{
public:
    RobotAssemblerConfiguration *self;

    ::UnitConfig uc;

    Impl(RobotAssemblerConfiguration *self_);

    bool parseYaml(const std::string &filename);
    bool parseGeneralSettings(ValueNode *vn);
    bool parseConnectingConstraintSettings(ValueNode *vn);
    bool parsePartsSettings(ValueNode *vn);

    //
    bool parseConnectingConf(ValueNode *vn, ConnectingConfiguration &out);
    bool parseConnectingTypeMatch(ValueNode *vn, ConnectingTypeMatch &out);

    bool parseParts(ValueNode *vn, Parts &out);
    bool parseGeometry(ValueNode *vn, Geometry &geom);
    bool parseConnectingPoint(ValueNode *vn, ConnectingPoint &cpt);
    bool parseActuator(ValueNode *vn, Actuator &act);
    bool parseExtraInfo(ValueNode *vn, ExtraInfo &einfo);

    //
    bool parseCoords(Mapping *map_, coordinates &cds);

    // reverse map (name -> index)
    std::map<std::string, int> reverseTypeNames; // listConnectingTypeNames
    std::map<std::string, int> reverseConfNames;
};

RobotAssemblerConfiguration::RobotAssemblerConfiguration()
{
    impl = new Impl(this);
}
RobotAssemblerConfiguration::~RobotAssemblerConfiguration()
{
    delete impl;
}

RobotAssemblerConfiguration::Impl::Impl(RobotAssemblerConfiguration *self_)
{
    self = self_;
}

bool RobotAssemblerConfiguration::parseYaml(const std::string &filename)
{
    return impl->parseYaml(filename);
}

bool RobotAssemblerConfiguration::Impl::parseYaml(const std::string &filename)
{
    YAMLReader yaml_reader;
    if (! yaml_reader.load(filename)) {
        std::cerr << "File Loading error : " << filename << std::endl;
        return false;
    }

    bool ret = false;
    for(int i = 0; i < yaml_reader.numDocuments(); i++) {
        ValueNode *val = yaml_reader.document(i);
        if ( val->isMapping() ) {
            std::string key = "GeneralSettings";
            ValueNode *target = val->toMapping()->find(key);
            if(target->isValid()) {
                std::cout << "--- target " << key << " found ---" << std::endl;
                ret = parseGeneralSettings(target);
                break;
            }
        }
    }
    if (!ret) {
        std::cerr << "failed parse settings" << std::endl;
        return false;
    }
    //
    ret = false;
    for(int i = 0; i < yaml_reader.numDocuments(); i++) {
        ValueNode *val = yaml_reader.document(i);
        if ( val->isMapping() ) {
            std::string key = "ConnectingConstraintSettings";
            ValueNode *target = val->toMapping()->find(key);
            if(target->isValid()) {
                std::cout << "--- target " << key << " found ---" << std::endl;
                ret = parseConnectingConstraintSettings(target);
                break;
            }
        }
    }
    if (!ret) {
        std::cerr << "failed parse constraint" << std::endl;
        return false;
    }
    //
    ret = false;
    for(int i = 0; i < yaml_reader.numDocuments(); i++) {
        ValueNode *val = yaml_reader.document(i);
        if ( val->isMapping() ) {
            std::string key = "PartsSettings";
            ValueNode *target = val->toMapping()->find(key);
            if(target->isValid()) {
                std::cout << "--- target " << key << " found ---" << std::endl;
                ret = parsePartsSettings(target);
                break;
            }
        }
    }
    if (!ret) {
        std::cerr << "failed parse parts" << std::endl;
        return false;
    }

    return true;
}

bool RobotAssemblerConfiguration::Impl::parseGeneralSettings(ValueNode *vn)
{
    if ( !vn->isMapping() ) {
        //
        return false;
    }

    ::UnitConfig::Unit _a_unit = ::UnitConfig::None;
    ::UnitConfig::Unit _l_unit = ::UnitConfig::None;
    ::UnitConfig::Unit _m_unit = ::UnitConfig::None;
    {
        ValueNode *target = vn->toMapping()->find("angleUnit");
        if(target->isValid() && target->isString()) {
            std::string str = target->toString();
            if (str == "deg" || str == "degree" || str == "DEG" || str == "DEGREE" ) {
                _a_unit = ::UnitConfig::degree;
            }
        }
    }
    {
        ValueNode *target = vn->toMapping()->find("lengthUnit");
        if(target->isValid() && target->isString()) {
            std::string str = target->toString();
            if (str == "mm" || str == "MM" || str == "millimeter" || str == "Millimeter" ) {
                _l_unit = ::UnitConfig::mm;
            }
        }
    }
    {
        ValueNode *target = vn->toMapping()->find("massUnit");
        if(target->isValid() && target->isString()) {
            std::string str = target->toString();
            if (str == "g" || str == "gram" ) {
                _m_unit = ::UnitConfig::g;
            }
        }
    }
    uc.setScale(_a_unit, _l_unit, _m_unit);
    return true;
}
bool RobotAssemblerConfiguration::Impl::parseConnectingConstraintSettings(ValueNode *vn)
{
    if ( !vn->isMapping() ) {
        //
        return false;
    }
    Mapping *val = vn->toMapping();
    {
        ValueNode *target = val->find("connecting-type-list");
        if(target->isValid() && target->isListing()) {
            Listing *lst = target->toListing();
            bool strlist = true;
            for(int i = 0; i < lst->size(); i++) {
                if (!lst->at(i)->isScalar()) {
                    strlist = false;
                    break;
                }
                std::string str = lst->at(i)->toString();
                self->listConnectingTypeNames.push_back(str);
                reverseTypeNames.insert(std::make_pair(str, i));
            }
            if (!strlist) {
                // invalid type
            }
        } else {
            // error ?
        }
    }
    {
        ValueNode *target = val->find("connecting-configuration-list");
        if(target->isValid() && target->isListing()) {
            Listing *lst = target->toListing();
            bool maplist = true;
            std::cerr << "conf-list / size: " << lst->size() << std::endl;
            for(int i = 0; i < lst->size(); i++) {
                std::cerr << "c " << i << std::endl;
                ConnectingConfiguration _conf;
                if (! parseConnectingConf(lst->at(i), _conf)) {
                    maplist = false;
                    continue;
                }
                self->listConnectingConfiguration.push_back(_conf);
                reverseConfNames.insert(std::make_pair(_conf.name, i));
            }
            if (!maplist) {
                // invalid type
                std::cerr << "invalid type 0" << std::endl;
            }
        } else {
            // error ?
            std::cerr << "error 0" << std::endl;
            return false;
        }
    }
    {
        ValueNode *target = val->find("connecting-type-match-list");
        if(target->isValid() && target->isListing()) {
            Listing *lst = target->toListing();
            bool maplist = true;
            std::cerr << "match-list / size: " << lst->size() << std::endl;
            for(int i = 0; i < lst->size(); i++) {
                std::cerr << "m " << i << std::endl;
                ConnectingTypeMatch _match;
                if (! parseConnectingTypeMatch(lst->at(i), _match)) {
                    maplist = false;
                    break;
                }
                self->listConnectingTypeMatch.push_back(_match);
            }
            if (!maplist) {
                // invalid type
                std::cerr << "invalid type 1" << std::endl;
            }
        } else {
            // error ?
            std::cerr << "error 1" << std::endl;
        }
    }
    return true;
}
bool RobotAssemblerConfiguration::Impl::parseCoords(Mapping *map_, coordinates &cds)
{
    bool all_res = true;
    {
    ValueNode *val = map_->find("rotation");
    if (val->isValid()) {
        std::vector<double> vec;
        if(readVector(val, vec, std::cerr)) {
            if (vec.size() >= 4) {
                AngleAxis _aa;
                uc.convRotationAngle(vec, _aa);
                cds.set(_aa);
            } else {
                std::cerr << "<<rotation size is required to be more than 4:" << std::endl;
                ::printNode(std::cerr, val);
                std::cerr << ">>" << std::endl;
                all_res = false;
            }
        } else {
            std::cerr << "<<rotation is required to be a list:" << std::endl;
            ::printNode(std::cerr, val);
            std::cerr << ">>" << std::endl;
            all_res = false;
        }
    } }
    {
    ValueNode *val = map_->find("translation");
    if (val->isValid()) {
        std::vector<double> vec;
        if(readVector(val, vec, std::cerr)) {
            if (vec.size() >= 3) {
                Vector3 _a_pos;
                uc.convPoint(vec, _a_pos);
                cds.set(_a_pos);
            } else {
                std::cerr << "<<translation size is required to be more than 3:" << std::endl;
                ::printNode(std::cerr, val);
                std::cerr << ">>" << std::endl;
                all_res = false;
            }
        } else {
            std::cerr << "<<translation is required to be a list:" << std::endl;
            ::printNode(std::cerr, val);
            std::cerr << ">>" << std::endl;
            all_res = false;
        }
    } }
    return all_res;
}

bool RobotAssemblerConfiguration::Impl::parseConnectingConf(ValueNode *vn, ConnectingConfiguration &out) {
    if ( !vn->isMapping() ) {
        // error message
        return false;
    }
    Mapping *val = vn->toMapping();
    ValueNode *ret;
    ret = val->find("name");
    if (ret->isValid() && ret->isScalar()) {
        out.name = ret->toString();
    } else {
        std::cerr << "not scalar" << std::endl;
        return false;
    }
    ret = val->find("description");
    if (ret->isValid() && ret->isScalar()) {
        out.description = ret->toString();
    }
    return parseCoords(val, out.coords);
}

bool RobotAssemblerConfiguration::Impl::parseConnectingTypeMatch(ValueNode *vn, ConnectingTypeMatch &out) {
    if ( !vn->isMapping() ) {
        // error message
        return false;
    }
    Mapping *val = vn->toMapping();
    ValueNode *ret = val->find("pair");
    if (ret->isValid() && ret->isListing()) {
        Listing *lst = ret->toListing();
        if (lst->size() < 2 || !lst->at(0)->isScalar() || !lst->at(1)->isScalar() ) {
            // error message
            return false;
        }
        std::string pair_a = lst->at(0)->toString();
        std::string pair_b = lst->at(1)->toString();
        auto it_a = reverseTypeNames.find(pair_a);
        if (it_a == reverseTypeNames.end()) { // not found
            return false;
        }
        out.pair[0] = it_a->second;
        auto it_b = reverseTypeNames.find(pair_b);
        if (it_b == reverseTypeNames.end()) { // not found
            return false;
        }
        out.pair[1] = it_b->second;
    } else {
        // error message
        return false;
    }
    ret = val->find("allowed-configuration");
    if (ret->isValid() && ret->isListing()) {
        Listing *lst = ret->toListing();
        for(int i = 0; i < lst->size(); i++) {
            if (lst->at(i)->isScalar()) {
                std::string str = lst->at(i)->toString();
                auto it = reverseConfNames.find(str);
                if (it == reverseConfNames.end()) { // not found
                    // error message
                    return false;
                }
                out.allowed_configuration.push_back(it->second);
            } else {
                // invalid but not critical
            }
        }
    } else {
        // error message
        return false;
    }
    return true;
}
bool RobotAssemblerConfiguration::Impl::parsePartsSettings(ValueNode *vn)
{
    if ( !vn->isListing() ) {
        //
        return false;
    }
    Listing *lst = vn->toListing();
    for(int i = 0; i < lst->size(); i++) {
        Parts pt;
        if(parseParts(lst->at(i), pt)) {
            self->mapParts.insert(std::make_pair(pt.type, pt));
        } else {
            std::cerr << "<<parseParts error: "<< std::endl;
            ::printNode(std::cerr, lst->at(i));
            std::cerr << ">>"<< std::endl;
        }
    }
    return true;
}
bool RobotAssemblerConfiguration::Impl::parseParts(ValueNode *vn, Parts &out)
{
    if( !vn->isMapping() ) {
        return false;
    }
    Mapping *mp = vn->toMapping();
    if(! ::mapString(mp, "type", out.type, std::cerr) ) {
        return false;
    }
    if(! ::mapString(mp, "class", out.class_name, std::cerr, false) ) {
        return false;
    }
    {   // <<visual
    ValueNode *val = mp->find("visual");
    if ( ! val->isValid() ) {
        //
    } else if ( ! val->isListing() ) {

    } else {
        Listing *lst = val->toListing();
        for(int i = 0; i < lst->size(); i++) {
            Geometry geom;
            if(parseGeometry(lst->at(i), geom)) {
                out.visual.push_back(geom);
            }
        }
    } } // >>visual
    {   // <<collision
    ValueNode *val = mp->find("collision");
    if ( ! val->isValid() ) {
        //
    } else if ( ! val->isListing() ) {

    } else {
        Listing *lst = val->toListing();
        for(int i = 0; i < lst->size(); i++) {
            Geometry geom;
            if(parseGeometry(lst->at(i), geom)) {
                out.collision.push_back(geom);
            }
        }
    } } // >>collision
    {   // <<mass-param
    out.hasMassParam = false;
    ValueNode *val = mp->find("mass-param");
    if (val->isValid() && val->isMapping()) {
        Mapping *mass_map = val->toMapping();
        if(!mapDouble(mass_map, "mass", out.mass, std::cerr)) {
            //
            return false;
        }
        std::vector<double> com;
        if(!mapVector(mass_map, "center-of-mass", com, std::cerr)) {
            //
            return false;
        } else {
            if (com.size() >= 3) {
                uc.convPoint(com, out.COM);
            } else {
                std::cerr << "size of center-of-mass is required to be more than 3:" << std::endl;
                return false;
            }
        }
        std::vector<double> it;
        if(!mapVector(mass_map, "inertia-tensor", it, std::cerr)) {
            //
            return false;
        } else {
            if (it.size() >= 9) {
                uc.convInertiaTensor(it, out.inertia_tensor);
            } else {
                std::cerr << "size of inertia-tensor is required to be more than 9 (6):" << std::endl;
                return false;
            }
        }
        out.hasMassParam = true;
    } else if (!!val) {
        std::cerr << "mass-param required to be dictionary type" << std::endl;
        ::printNode(std::cerr, val);
    } } // mass-param

    {   // <<connecting-points
    ValueNode *val = mp->find("connecting-points");
    if ( ! val->isValid() ) {
        //
    } else if ( ! val->isListing() ) {

    } else {
        Listing *lst = val->toListing();
        for(int i = 0; i < lst->size(); i++) {
            ConnectingPoint cpt;
            if(parseConnectingPoint(lst->at(i), cpt)) {
                out.connecting_points.push_back(cpt);
            }
        }
    } } // >>connecting-points
    {   // <<actuators
    ValueNode *val = mp->find("actuators");
    if ( ! val->isValid() ) {
        //
    } else if ( ! val->isListing() ) {

    } else {
        Listing *lst = val->toListing();
        for(int i = 0; i < lst->size(); i++) {
            Actuator act;
            if(parseActuator(lst->at(i), act)) {
                out.actuators.push_back(act);
            }
        }
    } } // >>actuators
    {   // <<extra-data
    ValueNode *val = mp->find("extra-data");
    if ( ! val->isValid() ) {
        //
    } else if ( ! val->isListing() ) {

    } else {
        Listing *lst = val->toListing();
        for(int i = 0; i < lst->size(); i++) {
            ExtraInfo einfo;
            if(parseExtraInfo(lst->at(i), einfo)) {
                out.extra_data.push_back(einfo);
            }
        }
    } } // >>extra-data
    return true;
}

bool RobotAssemblerConfiguration::Impl::parseGeometry(ValueNode *vn, Geometry &geom)
{
    return true;
}
bool RobotAssemblerConfiguration::Impl::parseConnectingPoint(ValueNode *vn, ConnectingPoint &cpt)
{
    return true;
}
bool RobotAssemblerConfiguration::Impl::parseActuator(ValueNode *vn, Actuator &act)
{
    return true;
}
bool RobotAssemblerConfiguration::Impl::parseExtraInfo(ValueNode *vn, ExtraInfo &einfo)
{
    return true;
}
}; };
