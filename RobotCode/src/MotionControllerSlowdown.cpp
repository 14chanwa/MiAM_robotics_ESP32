#include <MotionController.hpp>
#include <Utilities.h>
#include <Strategy.hpp>

#define MIN_RANGE 100
#define MAX_RANGE 300

#define BORDER_RANGE_IGNORE 50
#define TABLE_MAX_X 3000
#define TABLE_MAX_Y 2000

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

    // if detected point is outside of table (+- 5 cm), keep going
    RobotPosition coordinates_of_detected_point = getCurrentPosition();
    coordinates_of_detected_point.x += std::cos(coordinates_of_detected_point.theta) * vlx_range_detection_mm;
    coordinates_of_detected_point.y += std::sin(coordinates_of_detected_point.theta) * vlx_range_detection_mm;

    if (
        coordinates_of_detected_point.x <= BORDER_RANGE_IGNORE || 
        coordinates_of_detected_point.x >= TABLE_MAX_X - BORDER_RANGE_IGNORE ||
        coordinates_of_detected_point.y <= BORDER_RANGE_IGNORE ||
        coordinates_of_detected_point.y >= TABLE_MAX_Y - BORDER_RANGE_IGNORE
    )
    {
        coeff = std::max(coeff, 0.75f);
    }

    // if (mirrored) point is in the end zone, continue
    if (
        strategy::position_in_end_zone(coordinates_of_detected_point)
    )
    {
        coeff = std::max(coeff, 0.25f);
    }

    return coeff;
}
