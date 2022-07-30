#ifndef CNOID_ROBOT_ASSEMBLER_PLUGIN_SCENEBODY_H
#define CNOID_ROBOT_ASSEMBLER_PLUGIN_SCENEBODY_H

#include <cnoid/SceneWidgetEventHandler>
#include <cnoid/SceneBody>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class AssemblerBodyItem;
class AssemblerSceneBody;

class CNOID_EXPORT AssemblerSceneLink : public SceneLink
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    AssemblerSceneLink(AssemblerSceneBody* sceneBody, Link* link);
    ~AssemblerSceneLink();

#if 0
    void showOrigin(bool on);
    bool isOriginShown() const;
    void enableHighlight(bool on);

    void showMarker(const Vector3f& color, float transparency);
    void hideMarker();
    void setColliding(bool on);
#endif
private:
    class Impl;
    Impl* impl;

    friend class AssemblerSceneBody;
};
typedef ref_ptr<AssemblerSceneLink> AssemblerSceneLinkPtr;

class CNOID_EXPORT AssemblerSceneBody : public SceneBody, public SceneWidgetEventHandler
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static void initializeClass(ExtensionManager* ext);

    AssemblerSceneBody(AssemblerBodyItem* bodyItem);

    AssemblerBodyItem* bodyItem();

    AssemblerSceneLink* xxxSceneLink(int index);
    void setLinkVisibilities(const std::vector<bool>& visibilities);

    virtual void updateSceneModel() override;

    //// overrides
    virtual void onSceneModeChanged(SceneWidgetEvent* event) override;
    virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onDoubleClickEvent(SceneWidgetEvent* event) override;
    virtual bool onButtonReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    virtual void onPointerLeaveEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyPressEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onScrollEvent(SceneWidgetEvent* event) override;
    virtual void onFocusChanged(SceneWidgetEvent* event, bool on) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event) override;

protected:
    virtual ~AssemblerSceneBody();

private:
    AssemblerSceneBody(const AssemblerSceneBody& org);

    class Impl;
    Impl* impl;
    friend class AssemblerSceneLink;
};
typedef ref_ptr<AssemblerSceneBody> AssemblerSceneBodyPtr;

}

#endif
