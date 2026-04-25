#include <vlx_sensor.hpp>


void switch_i2c(int channel) {

  // Si le numéro de voie est correct, alors on change "l'aiguillage interne du TCA9548" pour connecter cette sortie à l'entrée
  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

VLXSensor::VLXSensor(
    uint8_t i2c_channel_
)
    : i2c_channel(i2c_channel_)
{
}

// Initialisation du capteur + ressources
bool VLXSensor::init()
{
    vlx_sensor = new SparkFun_VL53L5CX();

    switch_i2c(i2c_channel);
    if (!vlx_sensor->begin(0x29, Wire))
    {
        Serial.println("VL53L5CX init failed");
        return false;
    }

    vlx_sensor->setResolution(8*8); //Enable all 64 pads

    //Using 4x4, min frequency is 1Hz and max is 60Hz
    //Using 8x8, min frequency is 1Hz and max is 15Hz
    bool response = vlx_sensor->setRangingFrequency(15);
    if (response == true)
    {
        int frequency = vlx_sensor->getRangingFrequency();
        if (frequency > 0)
        {
        Serial.print("Ranging frequency set to ");
        Serial.print(frequency);
        Serial.println(" Hz.");
        }
        else
        Serial.println(F("Error recovering ranging frequency."));
    }
    else
    {
        return false;
    }


    // Set the ranging mode
    response = vlx_sensor->setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
    if (response == true)
    {
        SF_VL53L5CX_RANGING_MODE mode = vlx_sensor->getRangingMode();
        switch (mode)
        {
        case SF_VL53L5CX_RANGING_MODE::AUTONOMOUS:
            Serial.println(F("Ranging mode set to autonomous."));
            break;

        case SF_VL53L5CX_RANGING_MODE::CONTINUOUS:
            Serial.println(F("Ranging mode set to continuous."));
            break;

        default:
            Serial.println(F("Error recovering ranging mode."));
            break;
        }
    }
    else
    {
        Serial.println(F("Cannot set ranging mode requested. Freezing..."));
        return false;
    }

    vlx_sensor->startRanging();
    return true;
}


bool VLXSensor::update()
{
    switch_i2c(i2c_channel);
    // Poll sensor for new data
    if (vlx_sensor->isDataReady() == true)
    {
        if (vlx_sensor->getRangingData(&measurement_data)) //Read distance data into array
        {
            return true;
        }
    }
    return false;
}