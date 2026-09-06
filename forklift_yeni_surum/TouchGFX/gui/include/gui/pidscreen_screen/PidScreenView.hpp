#ifndef PIDSCREENVIEW_HPP
#define PIDSCREENVIEW_HPP

#include <gui_generated/pidscreen_screen/PidScreenViewBase.hpp>
#include <gui/pidscreen_screen/PidScreenPresenter.hpp>

class PidScreenView : public PidScreenViewBase
{
public:
    PidScreenView();
    virtual ~PidScreenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();

    virtual void updateBackgroundColor();

protected:
};

#endif // PIDSCREENVIEW_HPP
