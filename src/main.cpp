#include <Arduino.h>
#include "sbus.h"

// #include "typedef.hs"
// https://arduino-projekte.info/wp-content/uploads/2020/12/D1_mini_ESP32_pinout.jpg
/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial1);
/* SBUS data */
bfs::SbusData data;

#define MAX_MESSAGE_LENGTH 100

// Data Channel 0 = Zoom
// Data Channel 1 = Iris
// Data Channel 2 = Focus
#define DEFAULTS_BUTTON 2
#define FOCUS_KNOB_PIN 23
#define SERVO_PWM_PIN 22

int ZoomPWMValDesired = 17;
// #DEFINE PWM_8_BIT
#define PWM_16_BIT
// 8 int values for Zoom possitions
#ifdef PWM_8_BIT
int16_t Zoom12Val = 17;
int16_t Zoom14Val = 19;
int16_t Zoom15Val = 20;
int16_t Zoom18Val = 22;
int16_t Zoom25Val = 25;
int16_t Zoom30Val = 28;
int16_t Zoom35Val = 31;
int16_t Zoom40Val = 34;
#endif
#ifdef PWM_16_BIT
int16_t Zoom12Val = 2220;
int16_t Zoom14Val = 2472;
int16_t Zoom15Val = 2630;
int16_t Zoom18Val = 2855;
int16_t Zoom25Val = 3253;
int16_t Zoom30Val = 3624;
int16_t Zoom35Val = 3975;
int16_t Zoom40Val = 4400;
#endif


// ISR Functions to set the Zoom Values
void Zoom12()
{
  ZoomPWMValDesired = Zoom12Val;
}

void Zoom14()
{
  ZoomPWMValDesired = Zoom14Val;
}

void Zoom15()
{
  ZoomPWMValDesired = Zoom15Val;
}

void Zoom18()
{
  ZoomPWMValDesired = Zoom18Val;
}

void Zoom25()
{
  ZoomPWMValDesired = Zoom25Val;
}

void Zoom30()
{
  ZoomPWMValDesired = Zoom30Val;
}

void Zoom35()
{
  ZoomPWMValDesired = Zoom35Val;
}

void Zoom40()
{
  ZoomPWMValDesired = Zoom40Val;
}
// 10 int16_t values for Iris possitions
int16_t Iris_2_8_Val = 324;
int16_t Iris_3_2_Val = 449;
int16_t Iris_4_0_Val = 586;
int16_t Iris_4_5_Val = 649;
int16_t Iris_4_8_Val = 690;
int16_t Iris_5_6_Val = 807;
int16_t Iris_6_2_Val = 866;
int16_t Iris_6_7_Val = 930;
int16_t Iris_7_3_Val = 980;
int16_t Iris_8_7_Val = 1106;

// ISR Function to set the Iris Value

//debouncing function


void Iris_2_8()
{
  //run if havent run in


  data.ch[1] = Iris_2_8_Val;
  Serial.println("Iris 2.8");
}

void Iris_3_2()
{
  data.ch[1] = Iris_3_2_Val;
  Serial.println("Iris 3.2");
}

void Iris_4_0()
{
  data.ch[1] = Iris_4_0_Val;
  Serial.println("Iris 4.0");
}

void Iris_4_5()
{
  data.ch[1] = Iris_4_5_Val;
  Serial.println("Iris 4.5");
}

void Iris_4_8()
{
  data.ch[1] = Iris_4_8_Val;
  Serial.println("Iris 4.8");
}

void Iris_5_6()
{
  data.ch[1] = Iris_5_6_Val;
  Serial.println("Iris 5.6");
}

void Iris_6_2()
{
  data.ch[1] = Iris_6_2_Val;
  Serial.println("Iris 6.2");
}

void Iris_6_7()
{
  data.ch[1] = Iris_6_7_Val;
  Serial.println("Iris 6.7");
}

void Iris_7_3()
{
  data.ch[1] = Iris_7_3_Val;
  Serial.println("Iris 7.3");
}

void Iris_8_7()
{
  data.ch[1] = Iris_8_7_Val;
  Serial.println("Iris 8.7");
}
// Default values for the Zoom and Iris

// Defaults ISR
void Defaults()
{
  ZoomPWMValDesired= Zoom12Val;
  data.ch[1] = Iris_2_8_Val;
}

// Function to read values from Sbus and print to serial
void readSbus()
{
  if (sbus_rx.Read())
  {
    /* Grab the received data */
    data = sbus_rx.data();
    /* Display the received data */
    for (int8_t i = 0; i < 4; i++)
    {

      Serial.print(" Chanel: ");
      Serial.print(i);
      Serial.print(" Data: ");
      Serial.print(data.ch[i]);
    }
    Serial.println();
  }
}

void setup()
{
  // List of all ISR functions, enables attaching ISR's in a loop
  void (*isrList[])() = {Zoom12, Zoom14, Zoom15, Zoom18, Zoom25, Zoom30, Zoom35,
                         Zoom40, Iris_2_8, Iris_3_2, Iris_4_0, Iris_4_5, Iris_4_8,
                         Iris_5_6, Iris_6_2, Iris_6_7, Iris_7_3, Iris_8_7, Defaults};

  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial)
  {
  }
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();

  Serial.println("SBUS RX/TX started");
  // Define pins 2 thru 22 as inputs with pullup resistors
  for (int i = 2; i < 21; i++)
  {
    pinMode(i, INPUT_PULLDOWN);
  }
  Serial.println("Pins 2 thru 22 set as inputs with pullup resistors");
  // attach interupt to pin 2 thru 22
  for (int i = 2; i < 21; i++)
  {
    attachInterrupt(digitalPinToInterrupt(i), isrList[i - 2], RISING);
  }
  Serial.println("Interupts attached to pins 2 thru 22");

  // Set Focus know pin as analog input
  pinMode(FOCUS_KNOB_PIN, INPUT);

  // Set Servo PWM pin as output
  pinMode(SERVO_PWM_PIN, OUTPUT);
  // set PWM freq to 66Hz
  analogWriteFrequency(SERVO_PWM_PIN, 66);

  #ifdef PWM_16_BIT
  // set PWM resolution to 15 bits
  analogWriteResolution(15);
  #endif

  // Setup Complete
  Serial.println("Setup Complete");
}

void loop()
{

  static unsigned long previousMillis = 0;
  static int ZoomPWMVal = 17;

  // save the current time
  unsigned long currentMillis = millis();
  // if 15ms have passed since the last time the loop ran
  if (currentMillis - previousMillis >= 15)
  {
    // save the last time the loop ran
    previousMillis = currentMillis;
    // write the SBUS data
    sbus_tx.data(data);
    // Transmit SBUS data
    sbus_tx.Write();
  }

  // Focus Control
  // Read the focus know value
  int FocusRaw = analogRead(FOCUS_KNOB_PIN);
  // Map the focus know value to a SBUS value
  int FocusMapped = map(FocusRaw, 0, 1024, 462, 985);

      // analogWrite(SERVO_PWM_PIN, FocusMapped);
  // set FocusMapped to sbus channel 3
  data.ch[2] = FocusMapped;

  // Zoom Control
  // If the desired zoom value is differnt from the current zoom value slowly change the zoom value until they match
  // Only Check the zoom value every 500ms
  static unsigned long previousMillisZoom = 0;
  // save the current time
  unsigned long currentMillisZoom = millis();
  // if 500ms have passed since the last time the loop ran
  if (currentMillisZoom - previousMillisZoom >= 1)
  {
    // save the last time the loop ran
    previousMillisZoom = currentMillisZoom;
    // Check the zoom value
    if (ZoomPWMValDesired != ZoomPWMVal)
    {
      if (ZoomPWMValDesired > ZoomPWMVal)
      {
        ZoomPWMVal++;
      }
      else if (ZoomPWMValDesired < ZoomPWMVal)
      {
        ZoomPWMVal--;
      }
      //Update the servo PWM value
      analogWrite(SERVO_PWM_PIN, ZoomPWMVal);
    }
  }
}
