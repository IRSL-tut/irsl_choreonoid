#include "AssemblerBar.h"
//#include "AssemblerBodyItem.h"

#include <vector>

#define IRSL_DEBUG
#include "irsl_debug.h"

using namespace cnoid;

namespace cnoid {

class AssemblerBar::Impl
{
public:
    Impl(AssemblerBar* _self);

    AssemblerBar *self;
    std::vector<ToolButton *> buttons;

    void addButton(const char *icon, const char *tooltip, std::function<void()> func);

    void buttonClicked(int n);
};

}

AssemblerBar* AssemblerBar::instance()
{
    static AssemblerBar* instance = new AssemblerBar;
    return instance;
}

AssemblerBar::AssemblerBar()
    : ToolBar("AssemblerBar")
{
    impl = new Impl(this);
}

AssemblerBar::~AssemblerBar()
{
    delete impl;
}

AssemblerBar::Impl::Impl(AssemblerBar* _self)
{
    self = _self;
    addButton(":/Body/icon/storepose.svg", "No tooltip 0",
              [&](){ buttonClicked(0); } );
}

void AssemblerBar::Impl::addButton(const char *icon, const char *tooltip, std::function<void()> func)
{
    ToolButton *button;
    button = self->addButton(QIcon(icon));
    button->setToolTip(tooltip);
    button->sigClicked().connect(func);

    buttons.push_back(button);
}

void AssemblerBar::Impl::buttonClicked(int n)
{
    DEBUG_STREAM_FUNC(" buttonClicked:" << n << std::endl);
}
