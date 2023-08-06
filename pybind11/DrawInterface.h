/**
   @author Kunio Kojima
*/
// copied from jsk_choreonoid ( https://github.com/kindsenior/jsk_choreonoid )

#include <vector>
#include <iostream>

#include <cnoid/SceneWidget>
#include <cnoid/SceneView>
#include <cnoid/SceneDrawables>

#include <QCoreApplication>

#include "irsl_choreonoid/Coordinates.h"
namespace cnoid {

    class DrawInterface : public Referenced
    {
    protected:
        Vector3f colorVec_;
        SceneView* sv;
        SceneWidget* sw;
        SgLineSetPtr lineSet;
        SgVertexArrayPtr vertices;
        SgColorArrayPtr colors;
    // added by IRSL
    public:
        SgPosTransformPtr posTrans;
    public: static void flushAll()
        {
            //update view??
            QCoreApplication::processEvents(QEventLoop::AllEvents);
            // TODO: How to notify for redrawing (SgUpdate::REMOVED | SgUpdate::ADDED | SgUpdate::MODIFIED)
            //SceneView::instance()->sceneWidget()->sceneRoot()->notifyUpdate();
            SceneView::instance()->sceneWidget()->scene()->notifyUpdate();
        }
    public: static void convertVector3(const Vector3 &_in, Vector3f &_out)
        {
            _out.x() = _in.x(); _out.y() = _in.y(); _out.z() = _in.z();
        }
    public:
        // added by IRSL
        void render(bool doImmediately) {
            // sw->sceneRoot() or sw->scene()
            sw->renderScene(doImmediately);
        }
        void flush() {
            // SgUpdate::ADDED | SgUpdate::MODIFIED
            if (!!lineSet) { // check parent??
                lineSet->notifyUpdate();
            }
            if (!!posTrans) { // check parent??
                posTrans->notifyUpdate();
            }
        }
        void viewAll() {
            //SceneView::instance()->sceneWidget()->viewAll();
            sw->viewAll();
        }
        void setOrigin(const coordinates &_cds) {
            _cds.toPosition(posTrans->T());
        }
        void getOrigin(coordinates &_res) {
            _res = posTrans->T();
        }
        DrawInterface() { } // just using at GeneralDrawInterface
        // original settings
        DrawInterface(Vector3f colorVec){
            sv = SceneView::instance();
            sw = sv->sceneWidget();
            lineSet = new SgLineSet;
            vertices = lineSet->getOrCreateVertices();
            colors = lineSet->getOrCreateColors();
            //sw->sceneRoot()->addChild(lineSet);

            posTrans = new SgPosTransform();
            posTrans->T().setIdentity();
            sw->sceneRoot()->addChild(posTrans);

            setLineWidth(1);

            setColor(colorVec);
        }
        ~DrawInterface() {
            //std::cerr << "delete : " << this << std::endl;
            if (!!lineSet) {
                hide();
                lineSet->clear();
                lineSet->clearLines();
            }
            sw->sceneRoot()->removeChild(posTrans);
            vertices = nullptr;
            colors   = nullptr;
            lineSet  = nullptr;
            posTrans = nullptr;
        }
        void reset(){
            lineSet->clear();
            lineSet->clearLines();
            setColor(colorVec_);
            //
            posTrans->T().setIdentity();
        }

        void setColor(Vector3f colorVec){
            colorVec_ = colorVec;
            if(colors->size() < 1) colors->push_back(colorVec_);
            else colors->at(0) = colorVec_;
        }

        void setLineWidth(float width){
            lineSet->setLineWidth(width);
        }

        void show(bool flush=true){
            if (flush) {
                posTrans->addChildOnce(lineSet, SgUpdate::Added);
            } else {
                posTrans->addChildOnce(lineSet);
            }
        }

        void hide(bool flush=true){
            if (flush) {
                posTrans->removeChild(lineSet, SgUpdate::Added);
            } else {
                posTrans->removeChild(lineSet);
            }
        }

        void hide_and_show(bool flush=true) {
            posTrans->removeChild(lineSet);
            if (flush) {
                posTrans->addChildOnce(lineSet, SgUpdate::Added);
            } else {
                posTrans->addChildOnce(lineSet);
            }
        }

        void drawLine(Vector3f startPos, Vector3f endPos){
            reset();
            SgIndexArray& colorIndices = lineSet->colorIndices();

            // vertices->reserve(1);
            // lineSet->reserveNumLines(1);
            vertices->push_back(startPos);
            vertices->push_back(endPos);
            lineSet->addLine(0,1);

            // colorIndices.reserve(2);
            for(int i=0; i<2*2; ++i) colorIndices.push_back(0);
        }

        void drawArc(Vector3f centerPos, Vector3f radiousVec, Vector3f axisVec, float angle){// angle [deg]
            drawArcImpl(centerPos, radiousVec, axisVec, angle, false);
        }

        void drawArrowTip(Vector3f posVec, Vector3f directionVec, float length, Vector3f axisVec, float angle){
            drawArrowTipImpl(posVec, directionVec, length, axisVec, angle, directionVec, false);
        }

        void drawArrow(Vector3f startPos, Vector3f endPos, float arrowLength, Vector3f axisVec, float angle){
            drawLine(startPos, endPos);
            drawArrowTipPreserve(endPos, endPos-startPos, arrowLength, axisVec, angle, endPos-startPos);
        }

        void drawArcArrow(Vector3f centerPos, Vector3f radiousVec, Vector3f axisVec, float rotAngle, float arrowLength, float arrowAngle){
            drawArc(centerPos, radiousVec, axisVec, rotAngle);
            axisVec.normalize();
            Quaternionf q(AngleAxisf(rotAngle*M_PI/180, axisVec));
            Vector3f endPos = centerPos + q*radiousVec;
            Vector3f directionVec = q*axisVec.cross(radiousVec);
            drawArrowTipPreserve(endPos, directionVec, arrowLength, axisVec, arrowAngle, q*radiousVec);
        }

        void drawLineArcArrow(Vector3f centerPos, Vector3f radiousVec, Vector3f axisVec, float rotAngle, float arrowLength, float arrowAngle, float arcPosRate=0){
            drawArrow(centerPos, centerPos+axisVec, arrowLength, radiousVec, arrowAngle);
            centerPos += arcPosRate*axisVec;
            drawArcPreserve(centerPos, radiousVec, axisVec, rotAngle);
            axisVec.normalize();
            Quaternionf q(AngleAxisf(rotAngle*M_PI/180, axisVec));
            Vector3f endPos = centerPos + q*radiousVec;
            Vector3f directionVec = q*axisVec.cross(radiousVec);
            drawArrowTipPreserve(endPos, directionVec, arrowLength, axisVec, arrowAngle, q*radiousVec);
        }

        // add
        int addColor(Vector3f colorVec) {
            colors->push_back(colorVec);
            return colors->size() - 1;
        }
        void addDrawLine(Vector3f startPos, Vector3f endPos, int color_idx = 0) {
            SgIndexArray& colorIndices = lineSet->colorIndices();

            vertices->push_back(startPos);
            vertices->push_back(endPos);
            int sz = vertices->size();
            lineSet->addLine(sz-2, sz-1);

            // colorIndices.reserve(2);
            //for(int i=0; i<2*2; ++i) colorIndices.push_back(color_idx);
            for(int i=0; i < 2; ++i) colorIndices.push_back(color_idx);
        }
        void drawAxis(coordinates &_cds, int _axis, double _length)
        {
            Vector3 tmp;
            Vector3f v_a;
            switch(_axis) {
            case 0:
                _cds.x_axis(tmp);
                break;
            case 1:
                _cds.y_axis(tmp);
                break;
            case 2:
                _cds.z_axis(tmp);
                break;
            }
            convertVector3(tmp, v_a);
            v_a *= _length;
            Vector3f st(_cds.pos.x(), _cds.pos.y(), _cds.pos.z());
            this->drawLine(st, st + v_a);
        }
        void addAxis(coordinates &_cds, int _axis, double _length, int _color = 0)
        {
            Vector3 tmp;
            Vector3f v_a;
            switch(_axis) {
            case 0:
                _cds.x_axis(tmp);
                break;
            case 1:
                _cds.y_axis(tmp);
                break;
            case 2:
                _cds.z_axis(tmp);
                break;
            }
            convertVector3(tmp, v_a);
            v_a *= _length;
            Vector3f st(_cds.pos.x(), _cds.pos.y(), _cds.pos.z());
            this->addDrawLine(st, st + v_a, _color);
        }
        void addAxis3(coordinates &_cds, double _length, int _x_color = 0, int _y_color = 0, int _z_color = 0)
        {
            this->addAxis(_cds, 0, _length, _x_color);
            this->addAxis(_cds, 1, _length, _y_color);
            this->addAxis(_cds, 2, _length, _z_color);
        }
        void addBDAxis(coordinates &_cds, int _axis, double _length, int _color = 0)//Both Direction
        {
            Vector3 tmp;
            Vector3f v_a;
            switch(_axis) {
            case 0:
                _cds.x_axis(tmp);
                break;
            case 1:
                _cds.y_axis(tmp);
                break;
            case 2:
                _cds.z_axis(tmp);
                break;
            }
            convertVector3(tmp, v_a);
            v_a *= (0.5 * _length);
            Vector3f st(_cds.pos.x(), _cds.pos.y(), _cds.pos.z());
            this->addDrawLine(st - v_a, st + v_a, _color);
        }
        void addBDAxis3(coordinates &_cds, double _length, int _x_color = 0, int _y_color = 0, int _z_color = 0)//Both Direction
        {
            this->addBDAxis(_cds, 0, _length, _x_color);
            this->addBDAxis(_cds, 1, _length, _y_color);
            this->addBDAxis(_cds, 2, _length, _z_color);
        }
        //// copy fron SgPosTransform
        cnoidPosition &T() { return posTrans->T(); }
        const cnoidPosition &T() const { return posTrans->T(); }
        template<class Scalar, int Mode, int Options>
        void setPosition(const Eigen::Transform<Scalar, 3, Mode, Options>& T) {
            posTrans->setPosition(T);
        }
        template<class Derived>
        void setPosition(const Eigen::MatrixBase<Derived>& T) {
            posTrans->setPosition(T);
        }
    private:
        void drawArcPreserve(Vector3f posVec, Vector3f radiousVec, Vector3f axisVec, float rotAngle){
            drawArcImpl(posVec, radiousVec, axisVec, rotAngle, true);
        }

        void drawArcImpl(Vector3f posVec, Vector3f radiousVec, Vector3f axisVec, float rotAngle, bool preserve){// rotAngle [deg]
            if (!preserve) reset();
            SgIndexArray& colorIndices = lineSet->colorIndices();
            int startIdx = vertices->size();

            rotAngle *= M_PI/180;// [rad]
            float diffangle = 2*M_PI/(36*2);// [rad]
            int divisionNum = rotAngle/diffangle;

            axisVec.normalize();

            Quaternionf q;
            Vector3f v0, v1;

            vertices->push_back(posVec + radiousVec);
            colorIndices.reserve(divisionNum*2);
            colorIndices.push_back(0);
            for(int i=0; i < divisionNum; ++i){
                q = AngleAxisf((i+1)*diffangle, axisVec);
                v1 = q*radiousVec;
                vertices->push_back(posVec + v1);
                lineSet->addLine(startIdx+i,startIdx+i+1);

                // twice for one point?
                colorIndices.push_back(0);
                colorIndices.push_back(0);

                v0 = v1;
            }
        }

        void drawArrowTipPreserve(Vector3f posVec, Vector3f directionVec, float length, Vector3f axisVec, float arrowAngle, Vector3f radiousVec){
            drawArrowTipImpl(posVec, directionVec, length, axisVec, arrowAngle, radiousVec, true);
        }

        void drawArrowTipImpl(Vector3f posVec, Vector3f directionVec, float length, Vector3f axisVec, float arrowAngle, Vector3f radiousVec, bool preserve){
            if (!preserve) reset();
            SgIndexArray& colorIndices = lineSet->colorIndices();
            int startIdx = vertices->size();

            arrowAngle *= 0.5*M_PI/180;// [rad]
            axisVec.normalize();
            directionVec = length*directionVec.normalized();

            vertices->push_back(posVec);

            Quaternionf offet_q(AngleAxisf(-0.5*acos( radiousVec.normalized().dot((radiousVec+directionVec).normalized()) ), axisVec)); // for small radious arc

            Quaternionf q(AngleAxisf(arrowAngle, axisVec));
            Vector3f v0, v1;
            vertices->push_back(posVec - offet_q*q*directionVec);
            vertices->push_back(posVec - offet_q*q.inverse()*directionVec);

            lineSet->addLine(startIdx,startIdx+1);
            lineSet->addLine(startIdx,startIdx+2);

            for(int i=0; i<3*2; ++i) colorIndices.push_back(0);
        }
    };
    typedef ref_ptr<DrawInterface> DrawInterfacePtr;

    class GeneralDrawInterface : public DrawInterface
    {
    public:
        GeneralDrawInterface() : GeneralDrawInterface(false) {}
        GeneralDrawInterface(bool useRoot) {
            sv = SceneView::instance();
            sw = sv->sceneWidget();
            //lineSet.reset();
            //vetices.reset();
            //colors.reset();
            posTrans = new SgPosTransform();
            posTrans->T().setIdentity();
            if (useRoot) {
                root = (SgGroup *)sw->sceneRoot();
            } else {
                root = (SgGroup *)sw->scene();
            }
            root->addChild(posTrans);
        }
        ~GeneralDrawInterface() {
            posTrans->clearChildren();
            root->removeChild(posTrans, true);
        }
        void flush() {
            // SgUpdate::ADDED | SgUpdate::MODIFIED
            if (!!posTrans) {
                posTrans->notifyUpdate();
            } else if (!!root) {
                root->notifyUpdate();
            }
        }
        void cpp_clear(bool flush) {
            posTrans->clearChildren();
            if (flush) this->flush();
            //root->removeChild(posTrans, true);
        }
        void add_object(SgNodePtr &obj, bool update) {
            posTrans->addChildOnce(obj, update);
        }
        void add_object(SgGroupPtr &obj, bool update) {
            posTrans->addChildOnce(obj, update);
        }
        void add_object(SgTransformPtr &obj, bool update) {
            posTrans->addChildOnce(obj, update);
        }
        void add_object(SgPosTransformPtr &obj, bool update) {
            posTrans->addChildOnce(obj, update);
        }
        void add_object(SgShapePtr &obj, bool update) {
            posTrans->addChildOnce(obj, update);
        }
        void remove_object(SgNodePtr &obj, bool update) {
            posTrans->removeChild(obj, update);
        }
        void remove_object(SgGroupPtr &obj, bool update) {
            posTrans->removeChild(obj, update);
        }
        void remove_object(SgTransformPtr &obj, bool update) {
            posTrans->removeChild(obj, update);
        }
        void remove_object(SgPosTransformPtr &obj, bool update) {
            posTrans->removeChild(obj, update);
        }
        void remove_object(SgShapePtr &obj, bool update) {
            posTrans->removeChild(obj, update);
        }
    private:
        SgGroup *root;
    };
    typedef ref_ptr<GeneralDrawInterface> GeneralDrawInterfacePtr;

}
