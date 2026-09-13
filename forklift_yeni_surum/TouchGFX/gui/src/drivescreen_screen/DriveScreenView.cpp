#include <gui/drivescreen_screen/DriveScreenView.hpp>
#include <touchgfx/Color.hpp>
extern "C" {
#include "motor_control.h"
}

DriveScreenView::DriveScreenView() {}

void DriveScreenView::setupScreen()
{
    DriveScreenViewBase::setupScreen();
    updateBackgroundColor();
}

void DriveScreenView::tearDownScreen()
{
    DriveScreenViewBase::tearDownScreen();
}

void DriveScreenView::handleTickEvent()
{
    DriveScreenViewBase::handleTickEvent();
    updateBackgroundColor();

    float left_rpm = MotorControl_GetLeftRpm();
    float right_rpm = MotorControl_GetRightRpm();
    float left_mms = MotorControl_GetRightMeasuredMmPs();
    float right_mms = MotorControl_GetLeftMeasuredMmPs();
    float absolute_speed = (left_mms + right_mms) / 2;

    Unicode::snprintfFloat(txtRpmValueBuffer, TXTRPMVALUE_SIZE, "%.1f", left_rpm);
    txtRpmValue.setWildcard(txtRpmValueBuffer);
    txtRpmValue.invalidate();

    Unicode::snprintfFloat(txtRpmValue_1Buffer, TXTRPMVALUE_SIZE, "%.1f", right_rpm);
    txtRpmValue.setWildcard(txtRpmValue_1Buffer);
    txtRpmValue.invalidate();

    Unicode::snprintfFloat(txtSpeedValueBuffer, TXTRPMVALUE_SIZE, "%.1f", absolute_speed);
    txtRpmValue.setWildcard(txtSpeedValueBuffer);
    txtRpmValue.invalidate();

}

void DriveScreenView::stopButtonClicked()
{
    bool currentEmergency = presenter->isEmergency();
    presenter->setEmergency(!currentEmergency);
    updateBackgroundColor();
}

void DriveScreenView::updateBackgroundColor()
{
    bool emergencyStop = presenter->isEmergency();
    bool isAutonomous = presenter->isAutonomousMode();
    uint16_t obstacleDistance = presenter->getDistance();
    float temp = presenter->getTemp();
    uint8_t battery = presenter->getBattery();

    // --- Yeni Renk Paleti ve Birebir Eşit Koşullar ---
    if (emergencyStop || obstacleDistance < 25 || temp > 55.0f)
    {
        backgroundBox.setColor(touchgfx::Color::getColorFromRGB(213, 17, 14)); // #d5110e Kırmızı
    }
    else if (obstacleDistance <= 100 || temp > 45.0f || battery < 20)
    {
        backgroundBox.setColor(touchgfx::Color::getColorFromRGB(255, 148, 22)); // #ff9416 Turuncu
    }
    else
    {
        if (isAutonomous)
            backgroundBox.setColor(touchgfx::Color::getColorFromRGB(16, 140, 95)); // #108c5f Yeşil
        else
            backgroundBox.setColor(touchgfx::Color::getColorFromRGB(30, 58, 138)); // #0e10ce Mavi
    }

    backgroundBox.invalidate();
}
