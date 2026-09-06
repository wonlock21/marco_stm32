#include <gui/screen1_screen/Screen1View.hpp>
#include <touchgfx/Color.hpp>
#include <BitmapDatabase.hpp>

Screen1View::Screen1View() {}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
    // Sadece ekran ilk açıldığında Presenter'daki durumu butona aktar
    toggleDriveMode.forceState(presenter->isAutonomousMode());
    updateBackgroundColor();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}

void Screen1View::handleTickEvent()
{
    Screen1ViewBase::handleTickEvent();
    // Tick içinde butona müdahale etmeden sadece renk/ikon güncellemelerini çalıştırıyoruz
    updateBackgroundColor();
}

void Screen1View::toggleDriveModeClicked()
{
    // Buton kendisi zaten duruma geçti (ON ise OFF, OFF ise ON oldu).
    // Bize düşen sadece butonun YENİ durumunu okumak:
    bool isToggled = toggleDriveMode.getState();

    // Yeni durumu Presenter / Model tarafına aktar
    presenter->setAutonomousMode(isToggled);

    // Arka plan rengini anında güncelle
    updateBackgroundColor();
}

void Screen1View::stopButtonClicked()
{
    bool currentEmergency = presenter->isEmergency();
    presenter->setEmergency(!currentEmergency);
    updateBackgroundColor();
}

void Screen1View::updateBackgroundColor()
{
    bool emergencyStop = presenter->isEmergency();
    bool isAutonomous = presenter->isAutonomousMode();
    uint16_t obstacleDistance = presenter->getDistance();
    float temp = presenter->getTemp();
    uint8_t battery = presenter->getBattery();

    // --- Arka Plan Renk Kontrolü ---
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
            backgroundBox.setColor(touchgfx::Color::getColorFromRGB(30, 58, 138)); // YENİ MAVİ (Derin Safir)
    }

    // --- Batarya İkonu Güncellemesi ---
    if (battery >= 75)
    {
        batteryIcon.setBitmap(touchgfx::Bitmap(BITMAP_ON_ID));
    }
    else if (battery >= 50)
    {
        batteryIcon.setBitmap(touchgfx::Bitmap(BITMAP_ONE_ID));
    }
    else if (battery >= 25)
    {
        batteryIcon.setBitmap(touchgfx::Bitmap(BITMAP_ONEM_ID));
    }
    else if (battery >= 10)
    {
        batteryIcon.setBitmap(touchgfx::Bitmap(BITMAP_ONEML_ID));
    }
    else
    {
        batteryIcon.setBitmap(touchgfx::Bitmap(BITMAP_ONEMLI__ID));
    }

    // --- Sıcaklık İkonu Güncellemesi ---
    if (emergencyStop || temp > 55.0f)
    {
        tempIcon.setBitmap(touchgfx::Bitmap(BITMAP_TEMP_DANGER__ID));
    }
    else if (temp > 45.0f)
    {
        tempIcon.setBitmap(touchgfx::Bitmap(BITMAP_TEMP_WARN__ID));
    }
    else
    {
        tempIcon.setBitmap(touchgfx::Bitmap(BITMAP_TEMP_OK__ID));
    }

    // --- Engel (Radar Dalgaları) İkonu Güncellemesi ---
    if (emergencyStop || obstacleDistance < 25)
    {
        obstacleIcon.setBitmap(touchgfx::Bitmap(BITMAP_OBSTACLE_CLOSE_ID));
    }
    else if (obstacleDistance <= 100)
    {
        obstacleIcon.setBitmap(touchgfx::Bitmap(BITMAP_OBSTACLE_MID_ID));
    }
    else
    {
        obstacleIcon.setBitmap(touchgfx::Bitmap(BITMAP_OBSTACLE_FAR_ID));
    }

    // İkon Boyutlandırmaları
    batteryIcon.setWidthHeight(80, 40);
    tempIcon.setWidthHeight(120, 60);
    obstacleIcon.setWidthHeight(75, 150);

    // Yenile
    batteryIcon.invalidate();
    tempIcon.invalidate();
    obstacleIcon.invalidate();
    backgroundBox.invalidate();
}
