#include "RobotAssembler.h"
#include <cnoid/YAMLReader>
#include <iostream>
#include <cmath>
#include <sstream>
// get pid
#include <sys/types.h>
#include <unistd.h>
using namespace cnoid;

static void print(coordinates &cds)
{
    std::cout << "((" << cds.pos(0) << ", "
              << cds.pos(1) << ", " << cds.pos(2);
    Vector3 rpy; cds.getRPY(rpy);
    std::cout << ") (" << rpy(0)  << ", " << rpy(1)  << ", "
              << rpy(2) << "))";
}

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
    // length vector
    void convLengthVector(const std::vector<double> &in, std::vector<double>  &out) {
        out.clear();
        out.resize(in.size());
        for(int i = 0; i < in.size(); i++) out[i] = length_scale * in[i];
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
        // [TODO]
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

    int pid;
    int parts_counter;
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
    : parts_counter(0)
{
    self = self_;
    pid = getpid();
}

bool RobotAssemblerConfiguration::parseYaml(const std::string &filename)
{
    return impl->parseYaml(filename);
}

RoboasmPartsPtr RobotAssemblerConfiguration::makeParts(const std::string &parts_key)
{
    std::ostringstream os;
    os << parts_key << "_" << impl->pid << "_" << impl->parts_counter;
    impl->parts_counter++;
    makeParts(parts_key, os.str());
}

RoboasmPartsPtr RobotAssemblerConfiguration::makeParts(const std::string &parts_key, const std::string &name)
{
    std::cerr << "mk0" << std::endl;
    auto res = mapParts.find(parts_key);
    if (res == mapParts.end()) {
        std::cerr << "mk1" << std::endl;
        return nullptr;
    }
    std::cerr << "mk2" << std::endl;
    RoboasmPartsPtr ret = std::make_shared<RoboasmParts> (name, &(res->second));
    return ret;
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
    { // angle
    ValueNode *target = vn->toMapping()->find("angleUnit");
    if(target->isValid() && target->isString()) {
        std::string str = target->toString();
        if (str == "deg" || str == "degree" || str == "DEG" || str == "DEGREE" ) {
            _a_unit = ::UnitConfig::degree;
        }
    } }

    { // length
    ValueNode *target = vn->toMapping()->find("lengthUnit");
    if(target->isValid() && target->isString()) {
        std::string str = target->toString();
        if (str == "mm" || str == "MM" || str == "millimeter" || str == "Millimeter" ) {
            _l_unit = ::UnitConfig::mm;
        }
    } }

    { // mass
    ValueNode *target = vn->toMapping()->find("massUnit");
    if(target->isValid() && target->isString()) {
        std::string str = target->toString();
        if (str == "g" || str == "gram" ) {
            _m_unit = ::UnitConfig::g;
        }
    } }
    uc.setScale(_a_unit, _l_unit, _m_unit);
    return true;
}
bool RobotAssemblerConfiguration::Impl::parseConnectingConstraintSettings(ValueNode *vn)
{
    if ( !vn->isMapping() ) {
        // [todo]
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
                // [todo] invalid type
            }
        } else {
            // [todo] error ?
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
                // [todo] invalid type
                std::cerr << "invalid type 0" << std::endl;
            }
        } else {
            // [todo] error ?
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
                // [todo] invalid type
                std::cerr << "invalid type 1" << std::endl;
            }
        } else {
            // [todo] error ?
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
        // [todo] error message
        return false;
    }
    Mapping *val = vn->toMapping();
    ValueNode *ret;
    ret = val->find("name");
    if (ret->isValid() && ret->isScalar()) {
        out.name = ret->toString();
    } else {
        // [todo] error message
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
        // [todo] error message
        return false;
    }
    Mapping *val = vn->toMapping();
    ValueNode *ret = val->find("pair");
    if (ret->isValid() && ret->isListing()) {
        Listing *lst = ret->toListing();
        if (lst->size() < 2 || !lst->at(0)->isScalar() || !lst->at(1)->isScalar() ) {
            // [todo] error message
            return false;
        }
        std::string pair_a = lst->at(0)->toString();
        std::string pair_b = lst->at(1)->toString();
        auto it_a = reverseTypeNames.find(pair_a);
        if (it_a == reverseTypeNames.end()) { // not found
            // [todo]
            return false;
        }
        out.pair[0] = it_a->second;
        auto it_b = reverseTypeNames.find(pair_b);
        if (it_b == reverseTypeNames.end()) { // not found
            // [todo]
            return false;
        }
        out.pair[1] = it_b->second;
    } else {
        // [todo] error message
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
                    // [todo] error message
                    return false;
                }
                out.allowed_configuration.push_back(it->second);
            } else {
                // [todo] invalid but not critical
            }
        }
    } else {
        // [todo] error message
        return false;
    }
    return true;
}
bool RobotAssemblerConfiguration::Impl::parsePartsSettings(ValueNode *vn)
{
    if ( !vn->isListing() ) {
        // [todo]
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
        // [todo]
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
        // [todo]
    } else if ( ! val->isListing() ) {
        // [todo]
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
        // [todo]
    } else if ( ! val->isListing() ) {
        // [todo]
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
            // [todo]
            return false;
        }
        std::vector<double> com;
        if(!mapVector(mass_map, "center-of-mass", com, std::cerr)) {
            // [todo]
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
            // [todo]
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
        // [todo]
    } else if ( ! val->isListing() ) {
        // [todo]
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
        // [todo]
    } else if ( ! val->isListing() ) {
        // [todo]
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
        // [todo]
    } else if ( ! val->isListing() ) {
        // [todo]
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
    if ( ! vn->isMapping() ) {
        // [todo]
        return false;
    }
    Mapping *mp = vn->toMapping();
    // coords
    parseCoords(mp, geom.coords);
    // scale
    geom.scale = 1.0;
    mapDouble(mp, "scale", geom.scale, std::cerr, false);
    // type
    std::string tp_; geom.type = Geometry::None;
    mapString(mp, "type", tp_, std::cerr, false);
    if (tp_ == "mesh") {
        geom.type = Geometry::Mesh;
    } else if (tp_ == "box") {
        geom.type = Geometry::Box;
    } else if (tp_ == "cylinder") {
        geom.type = Geometry::Cylinder;
    } else if (tp_ == "sphere") {
        geom.type = Geometry::Sphere;
    } else if (tp_ == "cone") {
        geom.type = Geometry::Cone;
    } else if (tp_ == "capsule") {
        geom.type = Geometry::Capsule;
    } else if (tp_ == "ellipsoid") {
        geom.type = Geometry::Ellipsoid;
    } else if (tp_.size() > 0) {
        std::cerr << "unknown geometry type: " << tp_ << std::endl;
        return false;
    }

    if (geom.type == Geometry::None) {
        ValueNode *val;
        std::vector<double> vec;
        if ( (val = mp->find("box"))->isValid() ) {
            if( readVector(val, vec, std::cerr) ) {
                if (vec.size() >= 3) {
                    geom.type = Geometry::Box;
                    std::vector<double> nvec;
                    uc.convLengthVector(vec, nvec);
                    geom.parameter = nvec;
                } else {
                    // [todo]
                }
            }
        } else if ( (val = mp->find("cylinder"))->isValid() ) {
            if( readVector(val, vec, std::cerr) ) {
                if (vec.size() >= 2) {
                    geom.type = Geometry::Cylinder;
                    std::vector<double> nvec;
                    uc.convLengthVector(vec, nvec);
                    geom.parameter = nvec;
                } else {
                    // [todo]
                }
            }
        } else if ( (val = mp->find("sphere"))->isValid() ) {
            if( readVector(val, vec, std::cerr) ) {
                if (vec.size() >= 1) {
                    geom.type = Geometry::Sphere;
                    std::vector<double> nvec;
                    uc.convLengthVector(vec, nvec);
                    geom.parameter = nvec;
                } else {
                    // [todo]
                }
            }
        } else if ( (val = mp->find("cone"))->isValid() ) {
            if( readVector(val, vec, std::cerr) ) {
                if (vec.size() >= 1) { // [todo] param length
                    geom.type = Geometry::Cone;
                    std::vector<double> nvec;
                    uc.convLengthVector(vec, nvec);
                    geom.parameter = nvec;
                } else {
                    // [todo]
                }
            }
        } else if ( (val = mp->find("capsule"))->isValid() ) {
            if( readVector(val, vec, std::cerr) ) {
                if (vec.size() >= 1) { // [todo] param length
                    geom.type = Geometry::Capsule;
                    std::vector<double> nvec;
                    uc.convLengthVector(vec, nvec);
                    geom.parameter = nvec;
                } else {
                    // [todo]
                }
            }
        } else if ( (val = mp->find("ellipsoid"))->isValid() ) {
            if( readVector(val, vec, std::cerr) ) {
                if (vec.size() >= 1) { // [todo] param length
                    geom.type = Geometry::Ellipsoid;
                    std::vector<double> nvec;
                    uc.convLengthVector(vec, nvec);
                    geom.parameter = nvec;
                } else {
                    // [todo]
                }
            }
        } else if ( (val = mp->find("dummy"))->isValid() ) {
            geom.type = Geometry::Dummy;
        }
        //
        if (geom.type == Geometry::None) {
            // [todo]
            std::cerr << "Could not found geometry type!" << std::endl;
            return false;
        }
    } else if (geom.type == Geometry::Mesh) {
        if(! mapString(mp, "url", geom.url, std::cerr, true)) {
            // [todo]
            std::cerr << "mesh type geometry requires url:" << std::endl;
            return false;
        }
    } else {
        std::vector<double> vec;
        if(! mapVector(mp, "parameter", vec, std::cerr, true)) {
            // [todo]
            std::cerr << "non mesh type geometry requires parameter:" << std::endl;
            return false;
        }
        std::vector<double> nvec;
        uc.convLengthVector(vec, nvec);
        geom.parameter = nvec;
    }
    return true;
}
bool RobotAssemblerConfiguration::Impl::parseConnectingPoint(ValueNode *vn, ConnectingPoint &cpt)
{
    if ( ! vn->isMapping() ) {
        return false;
    }
    Mapping *mp = vn->toMapping();
    if (!mapString(mp, "name", cpt.name, std::cerr)) {
        return false;
    }
    ValueNode *val = mp->find("types");
    if ( ! val->isValid() ) {
        return false;
    }
    if ( ! val->isListing() ) {
        return false;
    }
    Listing *lst = val->toListing();
    for(int i = 0; i < lst->size(); i++) {
        if(lst->at(i)->isScalar()) {
            std::string key = lst->at(i)->toString();
            auto it = reverseTypeNames.find(key);
            if (it == reverseTypeNames.end()) {
                // [todo] key not found (not fatal)
                continue;
            }
            cpt.type_list.push_back(it->second);
        } else {
            // [todo] error (not fatal)
        }
    }
    if(cpt.type_list.size() < 1) {
        // [todo] no types
        return false;
    }
    parseCoords(mp, cpt.coords);

    return true;
}
bool RobotAssemblerConfiguration::Impl::parseActuator(ValueNode *vn, Actuator &act)
{
    if (! vn->isMapping() ) {
        return false;
    }
    Mapping *mp = vn->toMapping();
    std::string act_type;
    if (!mapString(mp, "actuator-type", act_type, std::cerr)) {
        return false;
    }
    ConnectingPoint::PartsType tp_;
    if (act_type == "rotational") {
        tp_ =  ConnectingPoint::Rotational;
    } else if (act_type == "linear") {
        tp_ =  ConnectingPoint::Linear;
    } else if (act_type ==  "fixed") {
        tp_ =  ConnectingPoint::Fixed;
    } else {
        std::cerr << "unknown actuator-type:" << act_type << std::endl;
        return false;
    }
    act = Actuator(tp_);

    ValueNode *val = mp->find("axis");
    if ( ! val->isValid() ) {
        // [todo] no axis
        act.axis = Vector3(0, 0, 1);
    } else if ( val->isScalar() ) {
        std::string ax = val->toString();
        if (ax == "x") {
            act.axis = Vector3(1, 0, 0);
        } else if (ax == "y") {
            act.axis = Vector3(0, 1, 0);
        } else if (ax == "z") {
            act.axis = Vector3(0, 0, 1);
        } else if (ax == "-x") {
            act.axis = Vector3(-1, 0, 0);
        } else if (ax == "-y") {
            act.axis = Vector3(0, -1, 0);
        } else if (ax == "-z") {
            act.axis = Vector3(0, 0, -1);
        } else {
            std::cerr << "invalid axis[string] : " << ax << std::endl;
            return false;
        }
    } else if ( val->isListing() ) {
        std::vector<double> ax;
        readVector(val, ax, std::cerr);
        if (ax.size() >= 3) {
            Vector3 v(ax[0], ax[1], ax[2]);
            v.normalize();
            act.axis = v;
        } else {
            // [todo] size error
        }
    } else {
        // [todo] invalid (mapping)
        std::cerr << "invalid axis " << std::endl;
        return false;
    }
    std::vector<double> qlim;
    mapVector(mp, "limit", qlim, std::cerr, false);
    std::vector<double> vlim;
    mapVector(mp, "vlimit", vlim, std::cerr, false);
    std::vector<double> tlim;
    mapVector(mp, "tqlimit", tlim, std::cerr, false);

    bool ret;
    ret = parseConnectingPoint(vn, *static_cast<ConnectingPoint *>(&act) );
    if (!ret) {
        // [todo] fatal
        return false;
    }

    return true;
}
bool RobotAssemblerConfiguration::Impl::parseExtraInfo(ValueNode *vn, ExtraInfo &einfo)
{
    // TODO
    return true;
}

//// Roboasm ////
RoboasmCoords::RoboasmCoords(const std::string &_name)
    : _parent(nullptr)
{
    name_str = _name;
}

const std::string &RoboasmCoords::name() const
{
    return name_str;
}
coordinates &RoboasmCoords::worldcoords()
{
    return _worldcoords;
}
const coordinates &RoboasmCoords::worldcoords() const {
    return _worldcoords;
}
void RoboasmCoords::copyWorldcoords(coordinates &w)
{
    w = _worldcoords;
}

void RoboasmCoords::set(coordinates &c)
{
    pos = c.pos;
    rot = c.rot;
    update();
}
RoboasmCoords *RoboasmCoords::parent()
{
    return _parent;
}
bool RoboasmCoords::hasParent()
{
    return (!!_parent);
}
bool RoboasmCoords::hasDescendants()
{
    return (descendants.size() > 0);
}
// virtual ??
void RoboasmCoords::update()
{
    if(!!_parent) {
#if 0
        std::cout << "pp : " << _parent << std::endl;
        std::cout << "w0 ";
        print(_worldcoords); std::cout << std::endl;
#endif
        _parent->copyWorldcoords(_worldcoords);
#if 0
        std::cout << "w1 ";
        print(_worldcoords); std::cout << std::endl;
        std::cout << "this ";
        print(*dynamic_cast<coordinates *>(this)); std::cout << std::endl;
#endif
        _worldcoords.transform(*this);
#if 0
        std::cout << "w2 ";
        print(_worldcoords); std::cout << std::endl;
#endif
    } else {
        _worldcoords = *this;
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
        if (!!(c->_parent)) {
            c->_parent->_dissoc(c.get());
        }
        c->_parent = this;
        coordinates newcoords;
        _worldcoords.transformation(newcoords, c->_worldcoords);
        c->set(newcoords);
        descendants.insert(c);
    }
}
bool RoboasmCoords::dissoc(RoboasmCoordsPtr c)
{
    return _dissoc(c.get());
}
bool RoboasmCoords::_dissoc(RoboasmCoords *c)
{
    if (c->_parent == this) {
        _erase_descendant(c);
        c->_parent = nullptr;
        c->set(c->_worldcoords);
        return true;
    }
    return false;
}
bool RoboasmCoords::dissocParent()
{
    if (!!_parent) {
        return _parent->_dissoc(this);
    }
    return false;
}

void RoboasmCoords::toRootList(coordsList &lst)
{
    lst.push_back(this);
    if (!!_parent) {
        _parent->toRootList(lst);
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
        if ((*it)->info->getType() != ConnectingPoint::Parts) {
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
        if ((*it)->info->getType() != ConnectingPoint::Parts) {
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
        if ((*it)->info->getType() != ConnectingPoint::Parts &&
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
        if ((*it)->info->getType() != ConnectingPoint::Parts &&
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
//template RoboasmCoords RoboasmCoords::find<RoboasmCoords>(const std::string &name); // use not template function
template RoboasmCoordsPtr RoboasmCoords::find<RoboasmParts>(const std::string &name);
template RoboasmCoordsPtr RoboasmCoords::find<RoboasmConnectingPoint>(const std::string &name);
//template RoboasmCoords RoboasmCoords::find<RoboasmRobot>(const std::string &name); //
// protected
#if 0
void RoboasmCoords::_replaceParent(RoboasmCoords *p)
{
    if (!!_parent) {
        _parent->dissoc(this);
        _parent = p;
    } else {
        _parent = p;
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

// connecting point
RoboasmConnectingPoint::RoboasmConnectingPoint(const std::string &_name,
                                               ConnectingPoint *_info)
    : RoboasmCoords(_name)
{
    std::cerr << "c0" << std::endl;
    info = _info;
}

// parts
RoboasmParts::RoboasmParts(const std::string &_name, Parts *_info)
    : RoboasmCoords(_name)
{
    info = _info;
    createConnectingPoints();
}

void RoboasmParts::createConnectingPoints()
{
    std::cerr << "ccp0" << std::endl;
    for(int i = 0; i < info->connecting_points.size(); i++) {
        std::cerr << "ccp1" << std::endl;
        RoboasmCoordsPtr ptr = std::make_shared<RoboasmConnectingPoint>
        (info->connecting_points[i].name, &info->connecting_points[i]);
        std::cerr << "ccp1_1" << std::endl;
        ptr->set(info->connecting_points[i].coords);

        std::cerr << "ccp1_2" << std::endl;
        this->assoc(ptr);
    }
    for(int i = 0; i < info->actuators.size(); i++) {
        std::cerr << "ccp2" << std::endl;
        RoboasmCoordsPtr ptr = std::make_shared<RoboasmConnectingPoint>
        (info->actuators[i].name, &info->actuators[i]);
        ptr->set(info->actuators[i].coords);
        this->assoc(ptr);
    }
    std::cerr << "ccp3" << std::endl;
    updateDescendants();
}

// robot
RoboasmRobot::RoboasmRobot(const std::string &_name, RoboasmPartsPtr parts)
    : RoboasmCoords(_name)
{
    assoc(parts);
    updateDescendants();
}
#if 0
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
}; };
