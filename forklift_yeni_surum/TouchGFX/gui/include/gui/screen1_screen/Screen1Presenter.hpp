#ifndef SCREEN1PRESENTER_HPP
#define SCREEN1PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>
#include <cstdint>

class Screen1View;

class Screen1Presenter : public touchgfx::Presenter, public ModelListener
{
public:
    Screen1Presenter(Screen1View& v);

    virtual void activate();
    virtual void deactivate();
    virtual ~Screen1Presenter() {};

    bool isAutonomousMode() { return model->isAutonomousMode(); }
    void setAutonomousMode(bool mode) { model->setAutonomousMode(mode); }

    bool isEmergency() { return model->isEmergency(); }
    void setEmergency(bool em) { model->setEmergency(em); }

    uint16_t getDistance() { return model->getDistance(); }
    float getTemp() { return model->getTemp(); }
    uint8_t getBattery() { return model->getBattery(); }

private:
    Screen1Presenter();
    Screen1View& view;
};

#endif // SCREEN1PRESENTER_HPP
