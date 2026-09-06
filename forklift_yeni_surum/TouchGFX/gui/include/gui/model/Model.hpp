#ifndef MODEL_HPP
#define MODEL_HPP

#include <cstdint>

class ModelListener;

class Model
{
public:
    Model();
    void bind(ModelListener* listener) { modelListener = listener; }
    void tick();

    bool isAutonomousMode() const { return isAutonomous; }
    void setAutonomousMode(bool autoMode) { isAutonomous = autoMode; }

    bool isEmergency() const { return emergencyStop; }
    void setEmergency(bool em) { emergencyStop = em; }

    uint16_t getDistance() const { return distance; }
    float getTemp() const { return temp; }
    uint8_t getBattery() const { return battery; }

protected:
    ModelListener* modelListener;

    bool isAutonomous;
    bool emergencyStop;
    uint16_t distance;
    float temp;
    uint8_t battery;
};

#endif // MODEL_HPP
