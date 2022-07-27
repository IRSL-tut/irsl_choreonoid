#include "XXXBar.h"
//#include "XXXBodyItem.h"

#include <vector>
#include <iostream>

using namespace cnoid;

namespace cnoid {

class XXXBar::Impl
{
public:
    Impl(XXXBar* _self);

    XXXBar *self;
    std::vector<ToolButton *> buttons;

    void addButton(const char *icon, const char *tooltip, std::function<void()> func);

    void buttonClicked(int n);
};

}

XXXBar* XXXBar::instance()
{
    static XXXBar* instance = new XXXBar;
    return instance;
}

XXXBar::XXXBar()
    : ToolBar("XXXBar")
{
    impl = new Impl(this);
}

XXXBar::~XXXBar()
{
    delete impl;
}

XXXBar::Impl::Impl(XXXBar* _self)
{
    self = _self;
    addButton(":/Body/icon/storepose.svg", "No tooltip 0",
              [&](){ buttonClicked(0); } );
}

void XXXBar::Impl::addButton(const char *icon, const char *tooltip, std::function<void()> func)
{
    ToolButton *button;
    button = self->addButton(QIcon(icon));
    button->setToolTip(tooltip);
    button->sigClicked().connect(func);

    buttons.push_back(button);
}

void XXXBar::Impl::buttonClicked(int n)
{
    std::cerr << "buttonClicked:" << n << std::endl;
}
