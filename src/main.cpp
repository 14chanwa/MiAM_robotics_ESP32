#include <L298NX2.h>

// #define LED_BUILTIN 2

#define IN1_A 25
#define IN2_A 26
#define IN1_B 32
#define IN2_B 33
#define EN_A 27
#define EN_B 14

#define ENCODER_A1 2
#define ENCODER_B1 4

#define BAT_READING 36
#define R1 20000.0
#define R2 4700.0

L298NX2 myMotors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);
unsigned char speed = 0;

float batReading_unfiltered = 0;
float batReading = 0;

// Moving average for battery reading
#define AN_Pot1     35
#define FILTER_LEN  15
uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
int AN_Pot1_i = 0;
int AN_Pot1_Raw = 0;
uint32_t readADC_Avg(int ADC_Raw);
bool MA_inited = false;

int encoder_count = 0;
bool oldB = 0;

// tasks

// void task_blink_led(void* parameters);
void task_monitor_battery(void* parameters);

// void IRAM_ATTR increment_encoder()
// {
//   encoder_count++;
// }

void IRAM_ATTR handleEncoder()
{
  // Get current status.
  bool currentA = digitalRead(ENCODER_A1);

  // The direction of the encoder is given by currentA xor oldB
  encoder_count += (oldB ^ currentA ? 1 : -1);
  oldB =  digitalRead(ENCODER_B1);

  // // Keep the number within 15 bits.
  // if(encoderCount[encoderNumber] > INT15_MAX)
  //   encoderCount[encoderNumber] = INT15_MIN ;
  // else if(encoderCount[encoderNumber] < INT15_MIN)
  //   encoderCount[encoderNumber] = INT15_MAX;

}



void setup() {
  Serial.begin(115200);
  // pinMode(LED_BUILTIN, OUTPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_6db);


  pinMode(ENCODER_A1, INPUT_PULLDOWN);
  attachInterrupt(ENCODER_A1, handleEncoder, RISING);

  // xTaskCreate(
  //   task_blink_led, 
  //   "task_blink_led",
  //   1000,
  //   NULL,
  //   1,
  //   NULL
  // ); 

  xTaskCreate(
    task_monitor_battery, 
    "task_monitor_battery",
    1000,
    NULL,
    1,
    NULL
  ); 
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.print("speed: ");
  Serial.println(speed);

  // forward
  myMotors.setSpeed(speed);
  myMotors.forward();
  vTaskDelay(100.0 / portTICK_PERIOD_MS);
  Serial.print("run bat reading: ");
  Serial.println(batReading);
  vTaskDelay(1000.0 / portTICK_PERIOD_MS);

  myMotors.stop();
  vTaskDelay(100.0 / portTICK_PERIOD_MS);
  Serial.print("stop bat reading: ");
  Serial.println(batReading);
  vTaskDelay(1000.0 / portTICK_PERIOD_MS);

  Serial.print("encoder count after forwward: ");
  Serial.println(encoder_count);

  // backward
  myMotors.setSpeed(speed);
  myMotors.backward();
  vTaskDelay(100.0 / portTICK_PERIOD_MS);
  Serial.print("run bat reading: ");
  Serial.println(batReading);
  vTaskDelay(1000.0 / portTICK_PERIOD_MS);

  myMotors.stop();
  vTaskDelay(100.0 / portTICK_PERIOD_MS);
  Serial.print("stop bat reading: ");
  Serial.println(batReading);
  vTaskDelay(1000.0 / portTICK_PERIOD_MS);
  
  Serial.print("encoder count after back: ");
  Serial.println(encoder_count);
  
  speed += 10;
}

/////////////////////////////////////////////
// Tasks
/////////////////////////////////////////////

// void task_blink_led(void* parameters)
// {
//   for(;;)
//   {
//     // Serial.print("task_blink_led is running on: ");
//     // Serial.println(xPortGetCoreID());
//     digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//     vTaskDelay(1000.0 / portTICK_PERIOD_MS);                       // wait for a second
//     digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
//     vTaskDelay(1000.0 / portTICK_PERIOD_MS);
//   }
// }

void task_monitor_battery(void* parameters)
{
  for(;;)
  {
    AN_Pot1_Raw = analogReadMilliVolts(BAT_READING);
    batReading_unfiltered = AN_Pot1_Raw * (R1 + R2) / R2 / 1000.0;
    batReading = readADC_Avg(AN_Pot1_Raw) * (R1 + R2) / R2 / 1000.0;
    vTaskDelay(10.0 / portTICK_PERIOD_MS);
  }    
}

/////////////////////////////////////////////
// Functions
/////////////////////////////////////////////

uint32_t readADC_Avg(int ADC_Raw)
{

  if (!MA_inited)
  {
    for (int i = 0; i < FILTER_LEN; i++)
      AN_Pot1_Buffer[i] = ADC_Raw;
    
    MA_inited = true;
  }

  int i = 0;
  uint32_t Sum = 0;
  
  AN_Pot1_Buffer[AN_Pot1_i++] = ADC_Raw;
  if(AN_Pot1_i == FILTER_LEN)
  {
    AN_Pot1_i = 0;
  }
  for(i=0; i<FILTER_LEN; i++)
  {
    Sum += AN_Pot1_Buffer[i];
  }
  return (Sum/FILTER_LEN);
}