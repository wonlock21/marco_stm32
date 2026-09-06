#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

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
