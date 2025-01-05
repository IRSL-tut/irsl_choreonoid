/**
   @author YoheiKakiuchi
*/

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/iostream.h> // add_ostream_redirect
#include <pybind11/numpy.h>
//
#include <cnoid/PyUtil>
#include <cnoid/PyEigenTypes>
#include <cnoid/ViewManager>

//
#include <cnoid/SceneGraph>
#include <cnoid/SceneDrawables>

//
#include <QCoreApplication>

//
#include <QMutex>
#include <QThread>

#include <vector>
#include <algorithm>
//#include <iostream>

using namespace cnoid;
namespace py = pybind11;

void get_viewtitles(std::vector<std::string> &res, bool mounted)
{
    auto viewClasses = ViewManager::viewClasses();
    for(auto& viewClass : viewClasses) {
        auto viewInstances = viewClass->instances();
        for(auto& view : viewInstances) {
            if (mounted && !(view->isMounted())) continue;
            res.push_back(view->windowTitle().toStdString());
            //view->isMounted();
            //view->mountOnMainWindow(true);
            //view->unmount();
        }
    }
}
void unmount_viewtitles(std::vector<std::string> &_in)
{
    auto viewClasses = ViewManager::viewClasses();
    for(auto& viewClass : viewClasses) {
        auto viewInstances = viewClass->instances();
        for(auto& view : viewInstances) {
            auto res = std::find(_in.begin(), _in.end(), view->windowTitle().toStdString());
            if (res != _in.end()) {
                view->unmount();
            }
        }
    }
}

void fixSgPosTransform(SgPosTransform &_trs)
{
    Matrix4 T = _trs.T().matrix();
    T(0, 2) = -T(0, 2);
    T(1, 2) = -T(1, 2);
    T(2, 2) = -T(2, 2);
    Isometry3 newT(T);
    _trs.setPosition(newT);
}

void fixMesh(SgMesh &_mesh)
{
    if(_mesh.hasNormals()) {
        SgNormalArray *nm = _mesh.normals();
        for(auto it = nm->begin(); it != nm->end(); it++) {
            (*it)(2) = - (*it)(2);
        }
    }
    if(_mesh.hasVertices()) {
        SgVertexArray *vt = _mesh.vertices();
        for(auto it = vt->begin(); it != vt->end(); it++) {
            (*it)(2) = - (*it)(2);
        }
    }
}

class MyMutex
{
public:
    QMutex mutex;

    void lock() {  mutex.lock(); }
    void unlock() {  mutex.unlock(); }
    bool try_lock(int wait) {
        if (wait >= 0) {
            return mutex.tryLock(wait);
        } else {
            return mutex.tryLock();
        }
    }
};

void setTextureImage(SgShape *shape, const py::array_t<unsigned char> &img, const std::string &name)
{
    SgTexture *tex = new SgTexture();
    SgImage *sgimg = tex->getOrCreateImage();
    sgimg->setUri(name, name);
    int dim    = img.ndim();
    int height = img.shape(0);
    int width  = img.shape(1);
    int comp   = 1;
    if (dim > 2) {
        comp   = img.shape(2);
    }
    //std::cout << "dim: " << dim << std::endl;
    //std::cout << "height: " << height << std::endl;
    //std::cout << "width: " << width << std::endl;
    //std::cout << "comp: " << comp << std::endl;
    sgimg->setSize(width, height, comp);
    const unsigned char *src = (const unsigned char *)(img.data(0));
    long len = width*height*comp - 1;
    for (long i = 0; i < width*height*comp; i++) {
        sgimg->pixels()[i] = src[i];
    }
    sgimg->image().applyVerticalFlip();
    shape->setTexture(tex);
}
#if 0
void debugTextureImage(SgShape *shape)
{
    SgTexture *tex = shape->getOrCreateTexture();
    SgImage *sgimg = tex->getOrCreateImage();
    int height = sgimg->height();
    int width = sgimg->width();
    int comp = sgimg->numComponents();
    std::cout << "height: " << height << std::endl;
    std::cout << "width: " << width << std::endl;
    std::cout << "comp: " << comp << std::endl;
    long cntr = 0;
    for (long i = 0; i < height; i++) {
        for (long j = 0; j < width; j++) {
            for (long k = 0; k < comp; k++) {
                std::cout << (int)sgimg->pixels()[cntr++] << " ";
            }
            std::cout << " ";
        }
        std::cout << std::endl;
    }
}
#endif
//// calcMassProperties
//// Refer: http://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
#define Subexpressions(w0, w1, w2, f1, f2, f3, g0, g1, g2) \
    {                                                      \
    double temp0 = w0 + w1;                                \
    double temp1 = w0 * w0;                                \
    double temp2 = temp1 + w1 * temp0;                     \
    f1 = temp0 + w2;                                       \
    f2 = temp2 + (w2 * f1);                                \
    f3 = (w0 * temp1) + (w1 * temp2) + (w2 * f2);          \
    g0 = f2 + w0*( f1 + w0 );                              \
    g1 = f2 + w1*( f1 + w1 );                              \
    g2 = f2 + w2*( f1 + w2 );                              \
    }                                                      \

void calcMassProperties(SgMeshPtr mesh, bool center_relative,
                        double &mass,
                        double &cm_x, double &cm_y, double &cm_z,
                        double &i_xx, double &i_yy, double &i_zz, double &i_xy, double &i_yz, double &i_xz)
{
    SgVertexArray *va = mesh->vertices();
    const SgIndexArray &index = mesh->faceVertexIndices();
    const size_t tmax = index.size()/3;
    const double mult[10] = {1.0/6, 1.0/24, 1.0/24, 1.0/24, 1.0/60, 1.0/60, 1.0/60, 1.0/120, 1.0/120, 1.0/120};
    double intg[10] = {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 }; // order: 1, x, y, z, xˆ2, yˆ2, zˆ2, xy, yz, zx

    int i0, i1, i2;
    double x0, x1, x2;
    double y0, y1, y2;
    double z0, z1, z2;
    double d0, d1, d2;
    double f1x, f2x, f3x, g0x, g1x, g2x;
    double f1y, f2y, f3y, g0y, g1y, g2y;
    double f1z, f2z, f3z, g0z, g1z, g2z;

    for (int t = 0; t < tmax; t++) {
        // get vertices of triangle t
        i0 = index[ 3*t ];
        i1 = index[ 3*t + 1 ];
        i2 = index[ 3*t + 2 ];

        x0 = (*va)[i0].x();
        y0 = (*va)[i0].y();
        z0 = (*va)[i0].z();

        x1 = (*va)[i1].x();
        y1 = (*va)[i1].y();
        z1 = (*va)[i1].z();

        x2 = (*va)[i2].x();
        y2 = (*va)[i2].y();
        z2 = (*va)[i2].z();

        // get edges and cross product of edges
        double a1 = x1-x0;
        double b1 = y1-y0;
        double c1 = z1-z0;
        double a2 = x2-x0;
        double b2 = y2-y0;
        double c2 = z2-z0;
        d0 = b1*c2-b2*c1;
        d1 = a2*c1-a1*c2;
        d2 = a1*b2-a2*b1;

        // compute integral terms
        Subexpressions( x0, x1, x2, f1x, f2x, f3x, g0x, g1x, g2x );
        Subexpressions( y0, y1, y2, f1y, f2y, f3y, g0y, g1y, g2y );
        Subexpressions( z0, z1, z2, f1z, f2z, f3z, g0z, g1z, g2z );

        // update integrals
        intg[0] += d0 * f1x;
        intg[1] += d0 * f2x;
        intg[2] += d1 * f2y;
        intg[3] += d2 * f2z;
        intg[4] += d0 * f3x;
        intg[5] += d1 * f3y;
        intg[6] += d2 * f3z;
        intg[7] += d0 * ( y0*g0x + y1*g1x + y2*g2x );
        intg[8] += d1 * ( z0*g0y + z1*g1y + z2*g2y );
        intg[9] += d2 * ( x0*g0z + x1*g1z + x2*g2z );
    }

    for (int i = 0; i < 10; i++) {
        intg[i] *= mult[i];
    }
    // mass
    mass = intg[0];
    // center of mass
    cm_x = intg[1] / mass;
    cm_y = intg[2] / mass;
    cm_z = intg[3] / mass;
    if (center_relative) {
        // inertia tensor relative to center of mass
        i_xx = intg[5] + intg [6] - mass * (cm_y * cm_y + cm_z * cm_z );
        i_yy = intg[4] + intg [6] - mass * (cm_z * cm_z + cm_x * cm_x );
        i_zz = intg[4] + intg [5] - mass * (cm_x * cm_x + cm_y * cm_y );
        i_xy = -(intg [7] - mass * cm_x * cm_y);
        i_yz = -(intg [8] - mass * cm_y * cm_z);
        i_xz = -(intg [9] - mass * cm_z * cm_x);
    } else {
        // inertia tensor relative to center of mass
        i_xx = intg[5] + intg [6];
        i_yy = intg[4] + intg [6];
        i_zz = intg[4] + intg [5];
        i_xy = -intg [7];
        i_yz = -intg [8];
        i_xz = -intg [9];
    }
}
////

PYBIND11_MODULE(IRSLUtil, m)
{
    m.doc() = "utilitiy for choreonoid (IRSL)";

    py::module::import("cnoid.Base");
    // py::module::import("cnoid.Util");

    //
    m.def("getViewTitles", [] (bool mounted) {
        std::vector<std::string> res;
        get_viewtitles(res, mounted);
        return res;
    });
    m.def("unmountViewTitles", &unmount_viewtitles);

    // fix for left-hand coordinates
    m.def("fixSgPosTransform", &fixSgPosTransform);
    m.def("fixMesh", &fixMesh);

    m.def("processEvent", [](){
        QCoreApplication::processEvents(QEventLoop::AllEvents);
    });
    m.def("usleep", [](int usec){
        QThread::usleep(usec);
    });

    py::class_< MyMutex >(m, "Mutex")
    .def(py::init<>())
    .def("lock", &MyMutex::lock)
    .def("unlock", &MyMutex::unlock)
    .def("try_lock", &MyMutex::try_lock)
    .def("__enter__", [] (MyMutex &self) {
        self.lock();
        //return self;
    })
    .def("__exit__", [] (MyMutex &self, py::object &exc_type, py::object &exc_value, py::object &traceback) {
        self.unlock();
    })
    ;

    m.def("setTextureImage", &setTextureImage);
    //m.def("debugTextureImage", &debugTextureImage);

    //
    m.def("calcMassProperties", [](SgMeshPtr mesh) {
                                    double mass;
                                    double cm_x, cm_y, cm_z;
                                    double i_xx, i_yy, i_zz, i_xy, i_yz, i_xz;
                                    calcMassProperties(mesh, true,
                                                       mass, cm_x, cm_y, cm_z,
                                                       i_xx, i_yy, i_zz, i_xy, i_yz, i_xz);
                                    py::list _lst;
                                    _lst.append(mass); _lst.append(cm_x); _lst.append(cm_y); _lst.append(cm_z);
                                    _lst.append(i_xx); _lst.append(i_yy); _lst.append(i_zz); _lst.append(i_xy); _lst.append(i_yz); _lst.append(i_xz);
                                    return _lst;
                                });
    // bind C++ output-stream
    py::add_ostream_redirect(m, "ostream_redirect");
}
