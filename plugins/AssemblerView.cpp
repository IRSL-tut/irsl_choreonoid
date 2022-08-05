#include "AssemblerView.h"
#include "AssemblerItem.h"
#include "RobotAssemblerHelper.h"

#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
//#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
#include <cnoid/SceneView>
#include <cnoid/SceneWidget>
#include <cnoid/ItemFileIO>
//#include <cnoid/Mapping>

// itemtreeview?
#include <cnoid/Archive>
#include <cnoid/ActionGroup>
#include <cnoid/ConnectionSet>
#include <cnoid/Widget>
#include <cnoid/Buttons>

#include <QLabel>
#include <QStyle>
#include <QBoxLayout>
#include <QScrollArea>
#include <QTabWidget>
#include <QTextEdit>

#include <vector>

#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;

namespace ra = cnoid::robot_assembler;

namespace cnoid {

class AssemblerSeneEvent : public SceneWidgetEventHandler
{
public:
    int uniq_id;

    // signals

    //// overrides : SceneWidgetEventHandler
    virtual void onSceneModeChanged(SceneWidgetEvent* event) override
    {
        DEBUG_STREAM_NL(std::endl);
    }
    //virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onDoubleClickEvent(SceneWidgetEvent* event) override
    {
        DEBUG_STREAM_NL(std::endl);
        // override double-click default behavior(change mode)
        return true;
    }
    //virtual bool onButtonReleaseEvent(SceneWidgetEvent* event) override;
    //virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    //virtual void onPointerLeaveEvent(SceneWidgetEvent* event) override;
    //virtual bool onKeyPressEvent(SceneWidgetEvent* event) override;
    //virtual bool onKeyReleaseEvent(SceneWidgetEvent* event) override;
    //virtual bool onScrollEvent(SceneWidgetEvent* event) override;
    //virtual void onFocusChanged(SceneWidgetEvent* event, bool on) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event) override
    {
        DEBUG_STREAM_NL(std::endl);
        auto menu = event->contextMenu();

        menu->addSeparator();
        menu->addItem("Align");
        menu->addItem("UnAlign");
        menu->addSeparator();
        menu->addItem("Attach");
        menu->addSeparator();
        menu->addItem("Undo");
        menu->addSeparator();
        menu->addItem("Write body");
        menu->addItem("Write urdf");
        menu->addSeparator();
        menu->addItem("Delete All");

        return true;
    }
};

// view manager
class AssemblerView::Impl
{
public:
    AssemblerView* self;
    QLabel targetLabel;

    QVBoxLayout *topLayout;
    QTabWidget *partsTab;

    Impl(AssemblerView* self);

    void initialize(bool config);

    void createButtons(ra::SettingsPtr &_ra_settings);
    void partsButtonClicked(int index);

    // assemble manager
    int pointClicked(ra::RASceneConnectingPoint *_cp);
    int partsClicked(ra::RASceneParts *_pt);

    void updateConnectingPoints();
    void updateMatchedPoints(ra::RASceneConnectingPoint *_pt, bool clearSelf = true,
                             ra::RASceneConnectingPoint::Clicked clearState =  ra::RASceneConnectingPoint::DEFAULT,
                             ra::RASceneConnectingPoint::Clicked matchedState =  ra::RASceneConnectingPoint::CAN_CONNECT1);
    void clearAllPoints();
    void updateRobots();
    bool robotExist(ra::RASceneRobot *_rb) {
        auto it = srobot_set.find(_rb);
        return (it != srobot_set.end());
    }
    void deleteRobot(ra::RASceneRobot *_rb);
    void attach();

    // attach history
    // align_
    ra::RASceneConnectingPoint *clickedPoint0;
    ra::RASceneConnectingPoint *clickedPoint1;
    std::set<ra::RASceneConnectingPoint *> selectable_spoint_set;

    ra::SettingsPtr ra_settings;
    ra::RoboasmPtr roboasm;
    std::vector<PushButton *> partsButtons;
    std::set<ra::RASceneRobot*> srobot_set;

    void notifyUpdate() {
        for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
            (*it)->notifyUpdate(SgUpdate::Added | SgUpdate::Removed | SgUpdate::Modified);
        }
    }

    AssemblerSeneEvent *self_event;
};

}

void AssemblerView::initializeClass(ExtensionManager* ext)
{
    DEBUG_STREAM_NL(std::endl);
    ext->viewManager().registerClass<AssemblerView>("AssemblerView", "AssemblerView_View");
}
AssemblerView* AssemblerView::instance()
{
    static AssemblerView* instance_ = ViewManager::getOrCreateView<AssemblerView>();
    return instance_;
}

AssemblerView::AssemblerView()
{
    DEBUG_STREAM_NL(std::endl);
    impl = new Impl(this);
}
AssemblerView::~AssemblerView()
{
    delete impl;
}
void AssemblerView::onActivated()
{
    DEBUG_STREAM_NL(std::endl);
}
void AssemblerView::onDeactivated()
{
    DEBUG_STREAM_NL(std::endl);
}
void AssemblerView::onAttachedMenuRequest(MenuManager& menu)
{
    DEBUG_STREAM_NL(std::endl);
}

bool AssemblerView::storeState(Archive& archive)
{
#if 0
    impl->positionWidget->storeState(&archive);
    switch(impl->positionWidget->targetLinkType()){
    case AssemblerWidget::AnyLink:
        archive.write("target_link_type", "any_link");
        break;
    case AssemblerWidget::RootOrIkLink:
        archive.write("target_link_type", "root_or_ik_link");
        break;
    case AssemblerWidget::IkLink:
        archive.write("target_link_type", "ik_link");
        break;
    }
#endif
    return true;
}

bool AssemblerView::restoreState(const Archive& archive)
{
#if 0
    impl->positionWidget->restoreState(&archive);
    string type;
    if(archive.read("target_link_type", type)){
        if(type == "any_link"){
            impl->positionWidget->setTargetLinkType(AssemblerWidget::AnyLink);
        } else if(type == "root_or_ik_link"){
            impl->positionWidget->setTargetLinkType(AssemblerWidget::RootOrIkLink);
        } else if(type == "ik_link"){
            impl->positionWidget->setTargetLinkType(AssemblerWidget::IkLink);
        }
    }
#endif
    return true;
}

void AssemblerView::createButtons(ra::SettingsPtr &ra_settings)
{
    impl->createButtons(ra_settings);
}

//// Impl
AssemblerView::Impl::Impl(AssemblerView* self)
    : self(self), partsTab(nullptr), clickedPoint0(nullptr), clickedPoint1(nullptr)
{
    initialize(false);
    //
    int id_ = SceneWidget::issueUniqueCustomModeId();
    self_event = new AssemblerSeneEvent();
    SceneView::instance()->sceneWidget()->activateCustomMode(self_event, id_);
}

void AssemblerView::Impl::initialize(bool config)
{
    self->setDefaultLayoutArea(MiddleRightArea);

    topLayout = new QVBoxLayout;
    topLayout->setContentsMargins(0, 0, 0, 0);
    topLayout->setSpacing(0);
    self->setLayout(topLayout);

    auto style = self->style();
    int lmargin = style->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int rmargin = style->pixelMetric(QStyle::PM_LayoutRightMargin);
    int tmargin = style->pixelMetric(QStyle::PM_LayoutTopMargin);
    int bmargin = style->pixelMetric(QStyle::PM_LayoutBottomMargin);

    auto hbox = new QHBoxLayout;
    hbox->setContentsMargins(lmargin, tmargin / 2, rmargin, bmargin / 2);
    targetLabel.setStyleSheet("font-weight: bold");
    targetLabel.setAlignment(Qt::AlignLeft);
    targetLabel.setText("---not initialized---");
    hbox->addWidget(&targetLabel);
    hbox->addStretch();
    //
    topLayout->addLayout(hbox);

#if 0
    if (!config) {
        return;
    }
    addTabs(config);
#endif
}

//void AssemblerView::Impl::addTabs(bool config)
void AssemblerView::Impl::createButtons(ra::SettingsPtr &_ra_settings)
{
    if (!!partsTab) {
        partsButtons.clear();
        delete partsTab;
    }
    ra_settings = _ra_settings;
    roboasm = std::make_shared<ra::Roboasm>(_ra_settings);

    int parts_num = ra_settings->mapParts.size();
    int tab_num = ((parts_num - 1) / 10) + 1;

    targetLabel.setText("---initialized---");

    partsTab = new QTabWidget(self); // parent??
    //// tabbed
    //auto hbox = new QHBoxLayout;

    int parts_index = 0;
    auto parts = ra_settings->mapParts.begin();
    for(int tab_idx = 0; tab_idx < tab_num; tab_idx++) {
        Widget *wd = new Widget(partsTab);
        QVBoxLayout *qvbox = new QVBoxLayout(wd);
        //
        for(int j = 0; j < 10; j++) {
            if (parts != ra_settings->mapParts.end()) {
                std::string name = parts->first;
                PushButton *bp = new PushButton(name.c_str(), partsTab);
                bp->sigClicked().connect( [this, parts_index]() { partsButtonClicked(parts_index); } );
                qvbox->addWidget(bp);
                partsButtons.push_back(bp);
                parts_index++;
            } else {
                break;
            }
            parts++;
        }
        std::string tab_name = "Tab";
        partsTab->addTab(wd, tab_name.c_str());
    }

    topLayout->addWidget(partsTab);
}
// assemble manager
void AssemblerView::Impl::partsButtonClicked(int index)
{
    DEBUG_STREAM_NL( " index: " << index << std::endl);
    PushButton *bp = partsButtons[index];

    std::string name = bp->text().toStdString();
    const BoundingBox &bb = SceneView::instance()->sceneWidget()->scene()->boundingBox();
    Vector3 cent = bb.center();
    Vector3 size = bb.size();
    DEBUG_STREAM_NL(" cent: " << cent(0) << ", " << cent(1) << ", " << cent(2) << std::endl);
    DEBUG_STREAM_NL(" size: " << size(0) << ", " << size(1) << ", " << size(2) << std::endl);
    std::string rb_name;
    if (srobot_set.size() == 0) {
        rb_name = "AssembleRobot";
    } else {
        rb_name = name;
    }
    AssemblerItemPtr itm = AssemblerItem::createItem(rb_name, name, roboasm);
    if (!!itm) {
        ra::RASceneRobot* rb_scene = dynamic_cast<ra::RASceneRobot*> (itm->getScene());
        const BoundingBox &rb_bb = rb_scene->boundingBox();
        Vector3 rb_cent = rb_bb.center();
        Vector3 rb_size = rb_bb.size();
        DEBUG_STREAM_NL(" rb_cent: " << rb_cent(0) << ", " << rb_cent(1) << ", " << rb_cent(2) << std::endl);
        DEBUG_STREAM_NL(" rb_size: " << rb_size(0) << ", " << rb_size(1) << ", " << rb_size(2) << std::endl);
        coordinates cds(Vector3(0, cent(1) + size(1)/2 +  rb_size(1)/2, 0));
        rb_scene->setCoords(cds);
        if (!!rb_scene) {
            rb_scene->sigPointClicked().connect( [this](ra::RASceneConnectingPoint *_p) { return pointClicked(_p); });
            rb_scene->sigPartsClicked().connect( [this](ra::RASceneParts *_p) { return partsClicked(_p); });
            rb_scene->sigDeleteRobot().connect( [this](ra::RASceneRobot *_rb) { this->deleteRobot(_rb); });
            rb_scene->sigAttach().connect( [this]() { this->attach(); });
        } else {
            //
        }
        clickedPoint0 = nullptr;
        clickedPoint1 = nullptr;
        selectable_spoint_set.clear();
        itm->setChecked(true);
        RootItem::instance()->addChildItem(itm);

        //
        updateRobots();
        clearAllPoints();
        notifyUpdate();
        SceneView::instance()->sceneWidget()->viewAll();
    }
}
int AssemblerView::Impl::pointClicked(ra::RASceneConnectingPoint *_cp)
{
    DEBUG_STREAM_NL(" " << _cp->name() << std::endl);
    //_cp->switchOn(false);
    //_cp->notifyUpdate(SgUpdate::REMOVED | SgUpdate::ADDED | SgUpdate::MODIFIED);// on scene graph

    if ( !robotExist(_cp->scene_robot()) ) {
        DEBUG_STREAM_NL(" robot not exist??" << std::endl);
    }
    bool modified = false;
    if (clickedPoint0 == _cp) {
        if (!!clickedPoint1) {
            // move 1 -> 0 / (0: a, 1: b) :=> (0: b, 1: null)
            DEBUG_STREAM_NL(" state0 : " << _cp->name() << std::endl);
            clickedPoint0 = clickedPoint1;
            clickedPoint1 = nullptr;
            modified = true;
        } else {
            // toggle selection0
            DEBUG_STREAM_NL(" state1 : " << _cp->name() << std::endl);
            clickedPoint0 = nullptr;
            modified = true;
        }
    } else if (clickedPoint1 == _cp) {
        // toggle selection1
        DEBUG_STREAM_NL(" state2 : " << _cp->name() << std::endl);
        clickedPoint1 = nullptr;
        modified = true;
    } else if (!clickedPoint0 && !clickedPoint1) {
        // add first one
        DEBUG_STREAM_NL(" state3 : " << _cp->name() << std::endl);
        clickedPoint0 = _cp;
        modified = true;
    } else if (!!clickedPoint0 && !!clickedPoint1) {
        DEBUG_STREAM_NL(" state4 : " << _cp->name() << std::endl);
        if(clickedPoint0->scene_robot() == _cp->scene_robot()) {
            DEBUG_STREAM_NL(" state4.0 : " << _cp->name() << std::endl);
            // same robot of selection0
            clickedPoint0 = _cp;
            modified = true;
        } else if (clickedPoint1->scene_robot() == _cp->scene_robot()) {
            DEBUG_STREAM_NL(" state4.1 : " << _cp->name() << std::endl);
            // same robot of selection1
            clickedPoint1 = _cp;
            modified = true;
        } else {
            DEBUG_STREAM_NL(" state4.2 : " << _cp->name() << std::endl);
            // replace last one
            clickedPoint1 = _cp;
            modified = true;
        }
    } else if (!!clickedPoint0) {
        DEBUG_STREAM_NL(" state5 : " << _cp->name() << std::endl);
        if(clickedPoint0->scene_robot() == _cp->scene_robot()) {
            DEBUG_STREAM_NL(" state5.0 : " << _cp->name() << std::endl);
            // replace selection0
            clickedPoint0 = _cp;
            modified = true;
        } else {
            DEBUG_STREAM_NL(" state5.1 : " << _cp->name() << std::endl);
            // add second one
            clickedPoint1 = _cp;
            modified = true;
        }
    } else if (!!clickedPoint1) {
        DEBUG_STREAM_NL(" state6 : (not occur!!) " << _cp->name() << std::endl);
        if(clickedPoint1->scene_robot() == _cp->scene_robot()) {
            DEBUG_STREAM_NL(" state6.0 " << _cp->name() << std::endl);
            clickedPoint0 = _cp;
            clickedPoint1 = nullptr;
            modified = true;
        } else {
            DEBUG_STREAM_NL(" state6.1 " << _cp->name() << std::endl);
            // add second one?
            clickedPoint0 = clickedPoint1;
            clickedPoint1 = _cp;
            modified = true;
        }
    } else {
        DEBUG_STREAM_NL(" === unknown state === : " << _cp->name() << std::endl);
    }
    if(modified) {
        updateConnectingPoints();
        for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
            (*it)->notifyUpdate(SgUpdate::Added | SgUpdate::Removed | SgUpdate::Modified);
        }
    }
    return 1;
}
int AssemblerView::Impl::partsClicked(ra::RASceneParts *_pt)
{
    DEBUG_STREAM_NL(" " << _pt->name() << std::endl);
    return 1;
}
void AssemblerView::Impl::updateConnectingPoints()
{
    //clickedPoint0
    //clickedPoint1
    if(!!clickedPoint0 && !!clickedPoint1) {
        DEBUG_STREAM_NL(" state 0 : both clicked" << std::endl);
        //selectable_spoint_set.clear();
        // can match clickedPoint0/clickedPoint1
        bool can_match = roboasm->canMatch(clickedPoint0->point(), clickedPoint1->point());
        updateMatchedPoints(clickedPoint0);
        // updateMatchedPoints(clickedPoint1);
        if(can_match) {
            clickedPoint0->changeState(ra::RASceneConnectingPoint::SELECT_GOOD0);
            clickedPoint1->changeState(ra::RASceneConnectingPoint::SELECT_GOOD1);
        } else {
            clickedPoint0->changeState(ra::RASceneConnectingPoint::SELECT_BAD0);
            clickedPoint1->changeState(ra::RASceneConnectingPoint::SELECT_BAD1);
        }
    } else if (!!clickedPoint0 &&  !clickedPoint1) {
        DEBUG_STREAM_NL(" state 1 : one  clicked" << std::endl);
        updateMatchedPoints(clickedPoint0);
        clickedPoint0->changeState(ra::RASceneConnectingPoint::SELECT_BAD0);
        //selectable_spoint_set.clear();
    } else if ( !clickedPoint0 && !!clickedPoint1) {
        DEBUG_STREAM_NL(" state 2 : not occur!!" << std::endl);
        // not occur
        updateMatchedPoints(clickedPoint1);
        clickedPoint1->changeState(ra::RASceneConnectingPoint::SELECT_BAD1);
        //selectable_spoint_set.clear();
    } else {
        DEBUG_STREAM_NL(" state 3 : all clear" << std::endl);
        //selectable_spoint_set.clear();
        clearAllPoints();
    }
}
void AssemblerView::Impl::clearAllPoints()
{
    for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
        auto pit_end = (*it)->spoint_set.end();
        for(auto pit = (*it)->spoint_set.begin(); pit != pit_end; pit++) {
            (*pit)->changeState(ra::RASceneConnectingPoint::DEFAULT);
        }
    }
}
void AssemblerView::Impl::updateMatchedPoints(ra::RASceneConnectingPoint *_pt, bool clearSelf,
                                              ra::RASceneConnectingPoint::Clicked clearState,
                                              ra::RASceneConnectingPoint::Clicked matchedState)
{
    ra::RoboasmConnectingPointPtr cp_ = _pt->point();
    for(auto it = srobot_set.begin(); it != srobot_set.end(); it++) {
        if( (*it) == _pt->scene_robot() ) {
            if(clearSelf) {
                auto pit_end = (*it)->spoint_set.end();
                for(auto pit = (*it)->spoint_set.begin(); pit != pit_end; pit++) (*pit)->changeState(clearState);
            }
            continue;
        }
        auto pit_end = (*it)->spoint_set.end();
        for(auto pit = (*it)->spoint_set.begin(); pit != pit_end; pit++) {
            if(roboasm->canMatch(cp_, (*pit)->point())) {
                (*pit)->changeState(matchedState);
            } else {
                (*pit)->changeState(clearState);
            }
        }
    }
}
void AssemblerView::Impl::updateRobots()
{
    ItemList<AssemblerItem> lst =  RootItem::instance()->checkedItems<AssemblerItem>();
    DEBUG_STREAM_NL(" lst : " << lst.size() << std::endl);
    srobot_set.clear();
    for(auto it = lst.begin(); it != lst.end(); it++) {
        SgNode *node = (*it)->getScene();
        ra::RASceneRobot *rbt = dynamic_cast<ra::RASceneRobot*>(node);
        if(!!rbt) {
            srobot_set.insert(rbt);
        }
    }
    DEBUG_STREAM_NL(" robot_list : " << srobot_set.size() << std::endl);
}
void AssemblerView::Impl::deleteRobot(ra::RASceneRobot *_rb)
{
    DEBUG_STREAM_NL(" delete robot : " << _rb->name() << std::endl);
    ItemList<AssemblerItem> lst =  RootItem::instance()->checkedItems<AssemblerItem>();
    srobot_set.clear();
    for(auto it = lst.begin(); it != lst.end(); it++) {
        SgNode *node = (*it)->getScene();
        ra::RASceneRobot *rbt = dynamic_cast<ra::RASceneRobot*>(node);
        if (!!rbt) {
            if (rbt == _rb) {
                // delete
                DEBUG_STREAM_NL(" delete:" << std::endl);
                (*it)->removeFromParentItem();
            } else {
                srobot_set.insert(rbt);
            }
        }
    }
    clickedPoint0 = nullptr;
    clickedPoint1 = nullptr;
    selectable_spoint_set.clear();
    clearAllPoints();
    notifyUpdate();
    SceneView::instance()->sceneWidget()->viewAll();
}
void AssemblerView::Impl::attach()
{
    DEBUG_STREAM_NL(std::endl);
    if(!clickedPoint0 || !clickedPoint1) {
        DEBUG_STREAM_NL( " require 2 clicked point" << std::endl);
        return;
    }

    ra::RASceneConnectingPoint *cp0 = clickedPoint0;
    ra::RASceneConnectingPoint *cp1 = clickedPoint1;
    if(cp1->scene_robot()->robot()->partsNum() >
       cp0->scene_robot()->robot()->partsNum() ) { // swap 0 and 1
        ra::RASceneConnectingPoint *tmp = cp0;
        cp0 = cp1; cp1 = cp0;
    }

    ra::RoboasmRobotPtr rb0 = cp0->scene_robot()->robot();
    ra::RoboasmRobotPtr rb1 = cp1->scene_robot()->robot();

    DEBUG_STREAM_NL(" rb0: " << rb0->name() << " <=(attach) rb1:" << rb1->name() << std::endl);
    DEBUG_STREAM_NL(" cp0-point(): " << cp0->point()->name() << " cp1-point()" << cp1->point()->name() << std::endl);

    bool res;
    std::vector<ra::ConnectingTypeMatch*> res_match_lst;
    res = rb0->searchMatch(rb1, cp1->point(), cp0->point(),
                           res_match_lst);
    if(!res) {
        DEBUG_STREAM_NL(" not matched " << std::endl);
        return;
    }
    DEBUG_STREAM_NL(" matched : " << res_match_lst.size() << std::endl);
    ra::ConnectingTypeMatch* mt_ = res_match_lst[0];
    ra::ConnectingConfigurationID ccid = mt_->allowed_configuration[0];

    bool res_attach = rb0->attach(rb1, cp1->point(), cp0->point(), ccid);
    if (!res_attach) {
        DEBUG_STREAM_NL(" attach failed " << std::endl);
        return;
    }

    DEBUG_STREAM_NL(" attached !! " << std::endl);
    {
        ra::coordsPtrList lst;
        rb0->allDescendants(lst);
        int cntr = 0;
        for(auto it = lst.begin(); it != lst.end(); it++) {
            std::cerr << cntr++ << " : " << *(*it) << std::endl;
        }
    }
    // erase(rb1)
    // update position of cp0,cp1 <- worldcoords
    cp0->scene_robot()->updateFromSelf();
    cp1->scene_robot()->updateFromSelf();

    //marge cp0->scene_robot() <=: cp1->scene_robot()
    ra::RASceneRobot *to_delete = cp1->scene_robot(); // after merged robots, all connecting point should be
    cp0->scene_robot()->mergeRobot(cp1->scene_robot());
    DEBUG_STREAM_NL(" scene_robot0: " << cp0->scene_robot()->name() << " / scene_robot1: " << to_delete->name() << std::endl);
    deleteRobot(to_delete);
    //notifyUpdate(); // update at delete robot
}
