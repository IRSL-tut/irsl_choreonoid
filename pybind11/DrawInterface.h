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

namespace cnoid {

    class DrawInterface : public Referenced
    {
    private:
        Vector3f colorVec_;
        SceneView* sv;
        SceneWidget* sw;
        SgLineSetPtr lineSet;
        SgVertexArrayPtr vertices;
        SgColorArrayPtr colors;
    // added by IRSL
    public: static void flush()
        {
            //update view??
            QCoreApplication::processEvents(QEventLoop::AllEvents);
        }

    public:
        // added by IRSL
        DrawInterface() {
            sv = SceneView::instance();
            sw = sv->sceneWidget();
            //lineSet.reset();
            //vetices.reset();
            //colors.reset();
        }
        void add_object(SgNodePtr &obj, bool update) {
            sw->sceneRoot()->addChildOnce(obj, update);
        }
        void remove_object(SgNodePtr &obj, bool update) {
            sw->sceneRoot()->removeChild(obj, update);
        }

        // original settings
        DrawInterface(Vector3f colorVec){
            sv = SceneView::instance();
            sw = sv->sceneWidget();

            lineSet = new SgLineSet;
            vertices = lineSet->getOrCreateVertices();
            colors = lineSet->getOrCreateColors();

            sw->sceneRoot()->addChild(lineSet);

            setLineWidth(1);

            setColor(colorVec);
        }

        void reset(){
            lineSet->clear();
            lineSet->clearLines();
            setColor(colorVec_);
        }

        void setColor(Vector3f colorVec){
            colorVec_ = colorVec;
            if(colors->size() < 1) colors->push_back(colorVec_);
            else colors->at(0) = colorVec_;
        }

        void setLineWidth(float width){
            lineSet->setLineWidth(width);
        }

        void show(){
            sw->sceneRoot()->addChildOnce(lineSet, true);
        }

        void hide(){
            sw->sceneRoot()->removeChild(lineSet, true);
        }

        void hide_and_show() {
            sw->sceneRoot()->removeChild(lineSet);
            sw->sceneRoot()->addChildOnce(lineSet, true);
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

}
