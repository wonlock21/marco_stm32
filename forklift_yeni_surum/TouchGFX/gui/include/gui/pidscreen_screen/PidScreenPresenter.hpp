#ifndef PIDSCREENPRESENTER_HPP
#define PIDSCREENPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>
#include <cstdint>

class PidScreenView;

class PidScreenPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    PidScreenPresenter(PidScreenView& v);

    virtual void activate();
    virtual void deactivate();
    virtual ~PidScreenPresenter() {};

    bool isAutonomousMode() { return model->isAutonomousMode(); }
    void setAutonomousMode(bool mode) { model->setAutonomousMode(mode); }

    bool isEmergency() { return model->isEmergency(); }
    void setEmergency(bool em) { model->setEmergency(em); }

    uint16_t getDistance() { return model->getDistance(); }
    float getTemp() { return model->getTemp(); }
    uint8_t getBattery() { return model->getBattery(); }

private:
    PidScreenPresenter();
    PidScreenView& view;
};

#endif // PIDSCREENPRESENTER_HPP
