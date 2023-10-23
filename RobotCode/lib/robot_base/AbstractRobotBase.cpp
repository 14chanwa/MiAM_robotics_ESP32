#include <Arduino.h>
#include <AbstractRobotBase.hpp>

AbstractRobotWheel::AbstractRobotWheel(std::string prefix) : prefix_(prefix)
{

}

void AbstractRobotWheel::printPrefix(const char* value_name)
{
    Serial.print(">");
    Serial.print(prefix_.c_str());
    Serial.print(value_name);
    Serial.print(":");
}