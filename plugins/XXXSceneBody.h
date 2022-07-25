#ifndef CNOID_IRSL_XXX_PLUGIN_SCENEBODY_H
#define CNOID_IRSL_XXX_PLUGIN_SCENEBODY_H

#include <cnoid/SceneBody>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class XXXBodyItem;
class XXXSceneBody;

class CNOID_EXPORT XXXSceneLink : public SceneLink
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    XXXSceneLink(XXXSceneBody* sceneBody, Link* link);
    ~XXXSceneLink();

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

    friend class XXXSceneBody;
};
typedef ref_ptr<XXXSceneLink> XXXSceneLinkPtr;

class CNOID_EXPORT XXXSceneBody : public SceneBody, public SceneWidgetEventHandler
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static void initializeClass(ExtensionManager* ext);

    XXXSceneBody(XXXBodyItem* bodyItem);

    XXXBodyItem* bodyItem();

    XXXSceneLink* xxxSceneLink(int index);
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
    virtual ~XXXSceneBody();

private:
    XXXSceneBody(const XXXSceneBody& org);

    class Impl;
    Impl* impl;
    friend class XXXSceneLink;
};
typedef ref_ptr<XXXSceneBody> XXXSceneBodyPtr;

}

#endif
