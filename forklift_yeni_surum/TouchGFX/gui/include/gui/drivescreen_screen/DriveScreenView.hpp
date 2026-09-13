#ifndef DRIVESCREENVIEW_HPP
#define DRIVESCREENVIEW_HPP

#include <gui_generated/drivescreen_screen/DriveScreenViewBase.hpp>
#include <gui/drivescreen_screen/DriveScreenPresenter.hpp>

class DriveScreenView : public DriveScreenViewBase
{
public:
    DriveScreenView();
    virtual ~DriveScreenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();

    virtual void stopButtonClicked();
    virtual void updateBackgroundColor();

protected:
    static const uint16_t TXTRPMVALUE_SIZE = 10;
    touchgfx::Unicode::UnicodeChar txtRpmValueBuffer[TXTRPMVALUE_SIZE];
    touchgfx::Unicode::UnicodeChar txtRpmValue_1Buffer[TXTRPMVALUE_SIZE];
    touchgfx::Unicode::UnicodeChar txtSpeedValueBuffer[TXTRPMVALUE_SIZE];
};

#endif // DRIVESCREENVIEW_HPP
