#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

extern "C" {
    bool g_is_autonomous_mode = false;
}

Model::Model() :
    modelListener(0),
    isAutonomous(false),
    emergencyStop(false),
    distance(150),
    temp(32.0f),
    battery(85)
{
}

void Model::tick()
{
}

void Model::setAutonomousMode(bool autoMode)
{
    isAutonomous = autoMode;
    g_is_autonomous_mode = autoMode; // Otonom durumunu C tarafına aktarıyoruz
}
