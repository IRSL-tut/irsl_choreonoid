#include "XXXView.h"
//#include "XXXWidget.h"
//#include "XXXBodyItem.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/Archive>
#include <cnoid/ActionGroup>
#include <cnoid/ConnectionSet>
#include <QLabel>
#include <QStyle>
#include <QBoxLayout>
#include <QScrollArea>

#include <iostream>

using namespace cnoid;

namespace cnoid {

class XXXView::Impl
{
public:
    XXXView* self;
    QLabel targetLabel;
    //XXXWidget* positionWidget;
    //ScopedConnection activeStateConnection;

    Impl(XXXView* self);
    //bool setTargetBodyAndLink(XXXBodyItem* bodyItem, Link* link);
    //void onAttachedMenuRequest(MenuManager& menuManager); // the same as base class
    //bool storeState(Archive& archive); // the same as base class
    //bool restoreState(const Archive& archive); //  the same as base class
};

}

void XXXView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<XXXView>("XXXView", "XXXView_View");
}
XXXView* XXXView::instance()
{
    static XXXView* instance_ = ViewManager::getOrCreateView<XXXView>();
    return instance_;
}

XXXView::XXXView()
{
    impl = new Impl(this);
}
XXXView::~XXXView()
{
    delete impl;
}
void XXXView::onActivated()
{
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
void XXXView::onDeactivated()
{
    //impl->activeStateConnection.disconnect();
}
void XXXView::onAttachedMenuRequest(MenuManager& menu)
{
    std::cerr << "onAttachedMenuRequest" << std::endl;
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

bool XXXView::storeState(Archive& archive)
{
#if 0
    impl->positionWidget->storeState(&archive);
    switch(impl->positionWidget->targetLinkType()){
    case XXXWidget::AnyLink:
        archive.write("target_link_type", "any_link");
        break;
    case XXXWidget::RootOrIkLink:
        archive.write("target_link_type", "root_or_ik_link");
        break;
    case XXXWidget::IkLink:
        archive.write("target_link_type", "ik_link");
        break;
    }
#endif
    return true;
}

bool XXXView::restoreState(const Archive& archive)
{
#if 0
    impl->positionWidget->restoreState(&archive);
    string type;
    if(archive.read("target_link_type", type)){
        if(type == "any_link"){
            impl->positionWidget->setTargetLinkType(XXXWidget::AnyLink);
        } else if(type == "root_or_ik_link"){
            impl->positionWidget->setTargetLinkType(XXXWidget::RootOrIkLink);
        } else if(type == "ik_link"){
            impl->positionWidget->setTargetLinkType(XXXWidget::IkLink);
        }
    }
#endif
    return true;
}

//// Impl
XXXView::Impl::Impl(XXXView* self)
    : self(self)
{
    self->setDefaultLayoutArea(MiddleRightArea);

    auto topLayout = new QVBoxLayout;
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
    targetLabel.setText("---initial---");
    hbox->addWidget(&targetLabel);
    hbox->addStretch();
    //
    topLayout->addLayout(hbox);

#if 0
    positionWidget = new XXXWidget(self);
    //positionWidget->setTargetLinkType(XXXWidget::IkLink);
    positionWidget->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    positionWidget->setAutoFillBackground(false);
    //
    topLayout->addWidget(positionWidget);
#endif
}

#if 0
bool XXXView::Impl::setTargetBodyAndLink(XXXBodyItem* bodyItem, Link* link)
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
