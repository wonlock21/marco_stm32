#ifndef DRIVESCREENPRESENTER_HPP
#define DRIVESCREENPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>
#include <cstdint>

class DriveScreenView;

class DriveScreenPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    DriveScreenPresenter(DriveScreenView& v);

    virtual void activate();
    virtual void deactivate();
    virtual ~DriveScreenPresenter() {};

    bool isAutonomousMode() { return model->isAutonomousMode(); }
    void setAutonomousMode(bool mode) { model->setAutonomousMode(mode); }

    bool isEmergency() { return model->isEmergency(); }
    void setEmergency(bool em) { model->setEmergency(em); }

    uint16_t getDistance() { return model->getDistance(); }
    float getTemp() { return model->getTemp(); }
    uint8_t getBattery() { return model->getBattery(); }

private:
    DriveScreenPresenter();
    DriveScreenView& view;
};

#endif // DRIVESCREENPRESENTER_HPP
