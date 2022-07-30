#include "AssemblerView.h"
//#include "AssemblerWidget.h"
#include "AssemblerBodyItem.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
//#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
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

namespace cnoid {

class AssemblerView::Impl
{
public:
    AssemblerView* self;
    QLabel targetLabel;
    //AssemblerWidget* positionWidget;
    //ScopedConnection activeStateConnection;
    QVBoxLayout *topLayout;
    QTabWidget *partsTab;

    Impl(AssemblerView* self);
    //bool setTargetBodyAndLink(AssemblerBodyItem* bodyItem, Link* link);
    //void onAttachedMenuRequest(MenuManager& menuManager); // the same as base class
    //bool storeState(Archive& archive); // the same as base class
    //bool restoreState(const Archive& archive); //  the same as base class

    void initialize(bool config);
    void addTabs(bool config);
    void partsButtonClicked(int index);
    std::vector<PushButton *> partsButtons;

};

}

void AssemblerView::initializeClass(ExtensionManager* ext)
{
    DEBUG_STREAM_INFO(AssemblerView,initializeClass, std::endl);
    ext->viewManager().registerClass<AssemblerView>("AssemblerView", "AssemblerView_View");
}
AssemblerView* AssemblerView::instance()
{
    static AssemblerView* instance_ = ViewManager::getOrCreateView<AssemblerView>();
    return instance_;
}

AssemblerView::AssemblerView()
{
    DEBUG_STREAM_INFO(AssemblerView,AssemblerView(), std::endl);
    impl = new Impl(this);
}
AssemblerView::~AssemblerView()
{
    delete impl;
}
void AssemblerView::onActivated()
{
    DEBUG_STREAM_INFO(AssemblerView,onActivated, std::endl);
    //auto bsm = BodySelectionManager::instance();
#if 0
    impl->activeStateConnection =
        bsm->sigCurrentSpecified().connect(
            [this, bsm](BodyItem* bodyItem, Link* link){
                if(!link) link = bsm->currentLink();
                impl->setTargetBodyAndLink(bodyItem, link);
            });
#endif
}
void AssemblerView::onDeactivated()
{
    DEBUG_STREAM_INFO(AssemblerView,onDeactivated, std::endl);
    //impl->activeStateConnection.disconnect();
}
void AssemblerView::onAttachedMenuRequest(MenuManager& menu)
{
    DEBUG_STREAM_INFO(AssemblerView,onAttachedMenuRequest, std::endl);
#if 0
    menu.setPath("/").setPath(_("Target link type"));

    auto checkGroup = new ActionGroup(menu.topMenu());
    menu.addRadioItem(checkGroup, _("Any links"));
    menu.addRadioItem(checkGroup, _("IK priority link and root link"));
    menu.addRadioItem(checkGroup, _("IK priority link"));
    checkGroup->actions()[impl->positionWidget->targetLinkType()]->setChecked(true);
    checkGroup->sigTriggered().connect(
        [=](QAction* check){
            impl->positionWidget->setTargetLinkType(checkGroup->actions().indexOf(check)); });
    menu.setPath("/");
    menu.addSeparator();
    impl->positionWidget->setOptionMenuTo(menu);
#endif
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

void AssemblerView::createButtons()
{
    impl->addTabs(true);
}

//// Impl
AssemblerView::Impl::Impl(AssemblerView* self)
    : self(self), partsTab(nullptr)
{
    initialize(false);
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

    if (!config) {
        return;
    }

    addTabs(config);
}

void AssemblerView::Impl::addTabs(bool config)
{
    if (!!partsTab) {
        //delete partsButtons
        delete partsTab;
    }

    targetLabel.setText("---initialized---");

    partsTab = new QTabWidget(self); // parent??
    //// tabbed
    auto hbox = new QHBoxLayout;

    // tab1
    Widget *wd = new Widget(partsTab);
    QVBoxLayout *qvbox = new QVBoxLayout(wd);
    PushButton *bp = new PushButton("bt1", partsTab);
    bp->sigClicked().connect( [&]() { partsButtonClicked(1); } );
    qvbox->addWidget(bp);
    qvbox->addWidget(new PushButton("bt2", partsTab));
    qvbox->addWidget(new PushButton("bt3", partsTab));
    qvbox->addWidget(new PushButton("bt4", partsTab));
    partsTab->addTab(wd, "Tab1");

    topLayout->addWidget(partsTab);
}
void AssemblerView::Impl::partsButtonClicked(int index)
{
    DEBUG_STREAM_INFO(AssemblerView::Impl,partsButtonClicked, " index: " << index << std::endl);

    //ItemTreeView::instance()
    ItemFileIO *ptr = AssemblerBodyItem::meshFileIO();
    Mapping options;
    //options.write("meshLengthUnitHint", "Millimeter");
    options.write("meshLengthUnitHint", "millimeter");
    Item *item = ptr->loadItem("/home/leus/sandbox/choreonoid_ws/src/irsl_choreonoid/DarwinOP3.stl",
                               nullptr, true, nullptr, &options);


    if (!!item) {
        RootItem::instance()->addChildItem(item);
    }
}
#if 0
PushButton *AssemblerView::Impl::createButton(QWidget *parent)
{
    PushButton *bt = new PushButton("", parent);
    partsButtons.push_back(bt);
    return bt;
}

bool AssemblerView::Impl::createPartsButton(int index)
{

}
#endif

#if 0
bool AssemblerView::Impl::setTargetBodyAndLink(AssemblerBodyItem* bodyItem, Link* link)
{
#if 0
    positionWidget->setTargetBodyAndLink(bodyItem, link);
    auto targetBodyItem = positionWidget->targetBodyItem();
    auto targetLink = positionWidget->targetLink();
    if(targetBodyItem && targetLink){
        targetLabel.setText(
            format("{0} / {1}",
                   targetBodyItem->displayName(), targetLink->name()).c_str());
    } else {
        targetLabel.setText("------");
    }
    return (targetBodyItem != nullptr);
#endif
}
#endif
