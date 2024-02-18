#include <MotionController.hpp>
#include <Utilities.h>

#define MIN_RANGE 30
#define MAX_RANGE 300

float MotionController::computeObstacleAvoidanceSlowdown(float vlx_range_detection_mm, bool const &hasMatchStarted)
{

    float coeff = 1.0f;

    if (vlx_range_detection_mm <= MIN_RANGE)
    {
        coeff = 0.0f;
    }
    else if (vlx_range_detection_mm < MAX_RANGE)
    {
        coeff = (vlx_range_detection_mm - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    }

    return coeff;
}
