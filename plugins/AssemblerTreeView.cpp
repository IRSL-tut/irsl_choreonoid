#include "AssemblerTreeView.h"

#include <cnoid/ComboBox>
#include <cnoid/TreeWidget>
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>

#include <QStyle>
#include <QBoxLayout>
#include <QAbstractItemModel>

#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;

namespace ra = cnoid::robot_assembler;
//namespace filesystem = cnoid::stdx::filesystem;

namespace cnoid {

void AssemblerTreeView::initializeClass(ExtensionManager* ext)
{
    //DEBUG_PRINT();
    ext->viewManager().registerClass<AssemblerTreeView>("AssemblerTreeView", "AssemblerTreeView_View");
}
AssemblerTreeView* AssemblerTreeView::instance()
{
    static AssemblerTreeView* instance_ = ViewManager::getOrCreateView<AssemblerTreeView>();
    return instance_;
}

class AssemblerTreeItem : public QTreeWidgetItem
{
public:
    virtual QVariant data(int column, int role) const override
    {
        DEBUG_PRINT();
        return QTreeWidgetItem::data(column, role);
    }
    virtual void setData(int column, int role, const QVariant& value) override
    {
        DEBUG_PRINT();
        return QTreeWidgetItem::setData(column, role, value);
    }
};
class AssemblerTreeView::Impl
{
public:
    Impl(AssemblerTreeView *_self);
    ~Impl();

    AssemblerTreeView* self;

    TreeWidget tree;
    ComboBox modeCombo;

    void setMode(int mode, bool doUpdate);
};

AssemblerTreeView::AssemblerTreeView()
{
    impl = new Impl(this);
}
AssemblerTreeView::~AssemblerTreeView()
{
    delete impl;
}

AssemblerTreeView::Impl::Impl(AssemblerTreeView *_self):
    self(_self), tree(nullptr)
{
    self->setDefaultLayoutArea(BottomLeftArea);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);

    modeCombo.addItem("ALL");
    modeCombo.addItem("Parts");
    //modeCombo.addItem("ConnectingPoint");
    //modeCombo.addItem("Actuator");
    modeCombo.addItem("Link");

    vbox->addWidget(&modeCombo);

    // tree settings
    tree.setSelectionMode(QAbstractItemView::ExtendedSelection);
    tree.setIndentation(12);
    tree.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    tree.setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    tree.setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);

    vbox->addWidget(&tree);

    self->setLayout(vbox);

    modeCombo.sigCurrentIndexChanged().connect(
        [&](int index){ setMode(index, true); });

    //// test
    auto item = new AssemblerTreeItem();
    item->setText(0, "hoge");

    tree.addTopLevelItem(item);

    auto citem = new AssemblerTreeItem();
    citem->setText(0, "moge");
    item->addChild(citem);

}
AssemblerTreeView::Impl::~Impl()
{
}
void AssemblerTreeView::Impl::setMode(int mode, bool doUpdate)
{
    DEBUG_STREAM(" mode: " << mode << ", updata: " << doUpdate);
}
}
