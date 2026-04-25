#include <vlx_sensor_and_display.hpp>

// How many leds in your strip?
#define NUM_LEDS 64

#define DISTANCE_FAR 500
#define DISTANCE_MID 250
#define DISTANCE_NEAR 0

void switch_i2c(int channel) {

  // Si le numéro de voie est correct, alors on change "l'aiguillage interne du TCA9548" pour connecter cette sortie à l'entrée
  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

VLXSensorAndDisplay::VLXSensorAndDisplay(
    uint8_t i2c_channel_,
    uint8_t led_data_pin_
)
    : i2c_channel(i2c_channel_),
      led_data_pin(led_data_pin_),
      vlx_sensor(nullptr)
{
}

// Initialisation du capteur + ressources
bool VLXSensorAndDisplay::init()
{
    pinMode(led_data_pin, OUTPUT);
    vlx_sensor = new SparkFun_VL53L5CX();

    switch_i2c(i2c_channel);
    if (!vlx_sensor->begin(0x29, Wire))
    {
        Serial.println("VL53L5CX init failed");
        return false;
    }

    vlx_sensor->setResolution(8*8); //Enable all 64 pads
    
    imageResolution = vlx_sensor->getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
    imageWidth = sqrt(imageResolution); //Calculate printing width
    Serial.print("imageWidth: ");
    Serial.println(imageWidth);


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

    // Setup leds
    leds = new CRGB[NUM_LEDS];
    for (uint i=0; i<NUM_LEDS; i++)
    {
        leds[i] = CRGB::Black;
    }
    // switch(led_data_pin)
    // {
    //     case 1:
            FastLED.addLeds<WS2812, 1, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
    //         break;
    //     default:
    //         Serial.println("Pin LED non supporté");
    //         return false;
    // }

    return true;
}

// Mise à jour des mesures + affichage éventuel
bool VLXSensorAndDisplay::update()
{
//Poll sensor for new data
  if (vlx_sensor->isDataReady() == true)
  {
    if (vlx_sensor->getRangingData(&measurement_data)) //Read distance data into array
    {
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
            if (x+y >= NUM_LEDS)
            {
                Serial.print("Indice invalide: ");
                Serial.println(x+y);
                continue;
            }
          Serial.print("\t");
          if (measurement_data.target_status[x + y] == 5 || measurement_data.target_status[x + y] == 9)
          {
            int distance = measurement_data.distance_mm[x + y];
            Serial.print(distance);
            if (distance > DISTANCE_FAR)
            {
              leds[x + y] = CRGB::Green;
            }
            else if (distance > DISTANCE_MID)
            {
              leds[x + y] = CRGB::Orange;
            }
            else
            {
              leds[x + y] = CRGB::Red;
            }
            // else if (distance > DISTANCE_MID)
            // {
            //   leds[x + y] = (DISTANCE_FAR-distance) * CRGB::Orange + (distance) * CRGB::Green;
            // }
            // else
            // {
            //   leds[x + y] = (DISTANCE_MID-distance) * CRGB::Red + (distance) * CRGB::Orange;
            // }
          }
          else
          {
            Serial.print("*");
            leds[x + y] = CRGB::Black;
          }
          
        }
        Serial.println();
      }
      Serial.println();
    }
  }
  Serial.println("End update");
}