#include <Arduino.h>
#include "sbus.h"

// #include "typedef.hs"
// https://arduino-projekte.info/wp-content/uploads/2020/12/D1_mini_ESP32_pinout.jpg
/* SBUS object, reading SBUS */
// bfs::SbusRx sbus_rx(&Serial1);
/* SBUS object, writing SBUS */
// bfs::SbusTx sbus_tx(&Serial1);
/* SBUS data */
bfs::SbusData data;

#define MAX_MESSAGE_LENGTH 100

// Data Channel 0 = Zoom
// Data Channel 1 = Iris
// Data Channel 2 = Focus
#define DEFAULTS_BUTTON 2
#define FOCUS_KNOB_PIN 21
#define SERVO_PWM_PIN 22

#define HC_12_SETPIN 22

#define DEBOUNCE_DELAY 500

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
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    ZoomPWMValDesired = Zoom12Val;
    data.ch[0] = Zoom12Val;
    Serial.println("Zoom 12");
  }
}

void Zoom14()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    ZoomPWMValDesired = Zoom14Val;
    data.ch[0] = Zoom14Val;
    Serial.println("Zoom 14");
  }
}

void Zoom15()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    ZoomPWMValDesired = Zoom15Val;
    data.ch[0] = Zoom15Val;
    Serial.println("Zoom 15");
  }
}

void Zoom18()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    ZoomPWMValDesired = Zoom18Val;
    data.ch[0] = Zoom18Val;
    Serial.println("Zoom 18");
  }
}

void Zoom25()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    ZoomPWMValDesired = Zoom25Val;
    data.ch[0] = Zoom25Val;
    Serial.println("Zoom 25");
  }
}

void Zoom30()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    ZoomPWMValDesired = Zoom30Val;
    data.ch[0] = Zoom30Val;
    Serial.println("Zoom 30");
  }
}

void Zoom35()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    ZoomPWMValDesired = Zoom35Val;
    data.ch[0] = Zoom35Val;
    Serial.println("Zoom 35");
  }
}

void Zoom40()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    ZoomPWMValDesired = Zoom40Val;
    data.ch[0] = Zoom40Val;
    Serial.println("Zoom 40");
  }
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

// debouncing function

void Iris_2_8()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    data.ch[1] = Iris_2_8_Val;
    Serial.println("Iris 2.8");
  }
}

void Iris_3_2()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    data.ch[1] = Iris_3_2_Val;
    Serial.println("Iris 3.2");
  }
}

void Iris_4_0()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    data.ch[1] = Iris_4_0_Val;
    Serial.println("Iris 4.0");
  }
}

void Iris_4_5()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    data.ch[1] = Iris_4_5_Val;
    Serial.println("Iris 4.5");
  }
}

void Iris_4_8()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    data.ch[1] = Iris_4_8_Val;
    Serial.println("Iris 4.8");
  }
}

void Iris_5_6()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    data.ch[1] = Iris_5_6_Val;
    Serial.println("Iris 5.6");
  }
}

void Iris_6_2()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    data.ch[1] = Iris_6_2_Val;
    Serial.println("Iris 6.2");
  }
}

void Iris_6_7()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    data.ch[1] = Iris_6_7_Val;
    Serial.println("Iris 6.7");
  }
}

void Iris_7_3()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    data.ch[1] = Iris_7_3_Val;
    Serial.println("Iris 7.3");
  }
}

void Iris_8_7()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    data.ch[1] = Iris_8_7_Val;
    Serial.println("Iris 8.7");
  }
}
// Default values for the Zoom and Iris

// Defaults ISR
void Defaults()
{
  static long lastpressedTime = 0;
  static long debounceDelay = DEBOUNCE_DELAY;
  long pressedTime = millis();
  if (pressedTime - lastpressedTime > debounceDelay)
  {
    lastpressedTime = pressedTime;
    ZoomPWMValDesired = Zoom12Val;
    data.ch[0] = ZoomPWMValDesired;
    data.ch[1] = Iris_2_8_Val;
    Serial.println("Defaults");
  }
}

// Function to read values from Sbus and print to serial
// void readSbus()
// {
//   if (sbus_rx.Read())
//   {
//     /* Grab the received data */
//     data = sbus_rx.data();
//     /* Display the received data */
//     for (int8_t i = 0; i < 4; i++)
//     {

//       Serial.print(" Chanel: ");
//       Serial.print(i);
//       Serial.print(" Data: ");
//       Serial.print(data.ch[i]);
//     }
//     Serial.println();
//   }
// }
bool ChanelChangeMode = false;
void setup()
{
  delay(3000);
  // List of all ISR functions, enables attaching ISR's in a loop
  // This Sets the order that the buttons are on the board, starting with bottom left and going clockwise
  void (*isrList[])() = {Zoom12, Zoom14, Zoom15, Zoom18, Zoom25, Zoom30, Zoom35,
                         Zoom40, Defaults, Iris_8_7, Iris_7_3, Iris_6_7, Iris_6_2,
                         Iris_5_6, Iris_4_8, Iris_4_5, Iris_4_0, Iris_3_2, Iris_2_8};

  /* Serial to display data */
  Serial.begin(115200);

  /* Begin the SBUS communication */
  // sbus_rx.Begin();
  // sbus_tx.Begin();

  Serial.println("SBUS RX/TX started");
  // Define pins 2 thru 22 as inputs with pullup resistors
  for (int i = 2; i < 21; i++)
  {
    pinMode(i, INPUT_PULLDOWN);
  }
  Serial.println("Pins 2 thru 22 set as inputs with pullup resistors");
  if (digitalRead(11))
  {
    Serial.println("Channel Change Mode!");
    ChanelChangeMode = true;
  }
  else
  {
    Serial.println("Normal Mode");

    // attach interupt to pin 2 thru 22
    for (int i = 2; i < 21; i++)
    {
      attachInterrupt(digitalPinToInterrupt(i), isrList[i - 2], RISING);
      Serial.println("Interupts attached to pins 2 thru 22");
    }
  }
  // Set Focus know pin as analog input
  pinMode(FOCUS_KNOB_PIN, INPUT);

  // Set Servo PWM pin as output
  pinMode(SERVO_PWM_PIN, OUTPUT);
  // set PWM freq to 66Hz
  analogWriteFrequency(SERVO_PWM_PIN, 66);

  Serial1.begin(9600);
  pinMode(HC_12_SETPIN, OUTPUT);
  digitalWrite(HC_12_SETPIN, HIGH);

#ifdef PWM_16_BIT
  // set PWM resolution to 15 bits
  analogWriteResolution(15);
#endif

  // Setup Complete

  Serial.println("Setup Complete");
}

// rolling average function
// takes in a int and retunrs the average of the last 10 values
int rollingAverage(int val)
{
  static int rollingAverageArray[10];
  static int rollingAverageIndex = 0;
  static int rollingAverageSum = 0;
  static int rollingAverageCount = 0;
  static int rollingAverageVal = 0;

  rollingAverageSum -= rollingAverageArray[rollingAverageIndex];
  rollingAverageArray[rollingAverageIndex] = val;
  rollingAverageSum += rollingAverageArray[rollingAverageIndex];
  rollingAverageIndex++;
  if (rollingAverageIndex >= 10)
  {
    rollingAverageIndex = 0;
  }
  if (rollingAverageCount < 10)
  {
    rollingAverageCount++;
  }
  rollingAverageVal = rollingAverageSum / rollingAverageCount;
  return rollingAverageVal;
}

int readStablePot(int potPin, int threshold)
{
  static int potVal = 0;
  static int potLast = 0;

  potVal = analogRead(potPin);

  if (abs(potVal - potLast) > threshold)
  { // check for significant change
    potLast = potVal;
  }

  return potLast;
}
void ChangeChannel(int Channel)
{
  // Put HC-12 in to command mode
  digitalWrite(HC_12_SETPIN, LOW);
  delay(100);
  // Send command to change channel with extra leading zeros before channel number
  Serial1.print("AT+C");
  Serial1.print("0");
  Serial1.print("0");
  Serial1.println(Channel);

  // Wait for response
  delay(100);
  // Put HC-12 back in to normal mode
  digitalWrite(HC_12_SETPIN, HIGH);
  delay(100);
}
void loop()
{
  if (ChanelChangeMode)
  {
    char array[3];
    int buttonPressed = 0;
    // Read all buttons 2 thru 10
    for (int i = 2; i < 11; i++)
    {
      if (digitalRead(i) == HIGH)
      {
        buttonPressed = i - 1;
      }
    }
    if (buttonPressed != 0)
    {
      delay(100);
      Serial.print("C");
      Serial.println(buttonPressed);
      Serial1.print("CHANGE");
      Serial1.print(buttonPressed);
      Serial.println("Channel Change Requested");
      while (Serial1.available() == 0)
      {
        /* code */
      }

      while (Serial1.available())
      {
        byte temp = Serial1.read();
        Serial.println(temp);
        Serial.println(buttonPressed);
        if ((temp - '0') == buttonPressed) // Receiver verified it got the message to change channel
        {
          ChangeChannel(buttonPressed);
          ChanelChangeMode = false;
          Serial.println("Channel Change Accepted");
          while (true)
          {
          }
        }
      }
    }
    // Serial1.flush();
  }
  else
  {

    static bfs::SbusData lastData;
    static int lastFocusedMapped = 0;
    static long lastLoopTime = 0;
    static int focusHistory[6] = {0, 0, 0, 0, 0, 0};
    static int focusHistoryIndex = 0;
    // If 10 ms has passsed since last loop
    if (millis() - lastLoopTime > 20)
    {

      // Focus Control
      // Read the focus know value
      int FocusRaw = readStablePot(FOCUS_KNOB_PIN, 2);
      // FocusRaw = 0;
      // Map the focus know value to a SBUS value
      int FocusMapped = map(FocusRaw, 0, 1024, 462, 985);
      // if the last 6 values of focusmapped are changing between 2 values then set focusmapped to the last value
      focusHistory[focusHistoryIndex] = FocusMapped;
      focusHistoryIndex++;
      if (focusHistoryIndex >= 6)
      {
        focusHistoryIndex = 0;
      }
      if (focusHistory[0] == focusHistory[2])
      {
        FocusMapped = focusHistory[0];
      }

      // set the servo pwm pin to the mapped value

      // analogWrite(SERVO_PWM_PIN, FocusMapped);
      // set FocusMapped to sbus channel 3
      // if (lastFocusedMapped - FocusMapped > 1 || lastFocusedMapped - FocusMapped < -1)
      // {
      lastFocusedMapped = FocusMapped;
      data.ch[2] = FocusMapped;
      // }
    }

    // if data has changed transmit it over serial1
    if (data.ch[0] != lastData.ch[0] ||
        data.ch[1] != lastData.ch[1] ||
        data.ch[2] != lastData.ch[2] ||
        data.ch[3] != lastData.ch[3])
    {
      Serial1.print("START");
      Serial1.write(data.ch[0] & 0xFF);
      Serial1.write(data.ch[0] >> 8);

      Serial1.write(data.ch[1] & 0xFF);
      Serial1.write(data.ch[1] >> 8);

      Serial1.write(data.ch[2] & 0xFF);
      Serial1.write(data.ch[2] >> 8);
      Serial1.print("END");

      // Serial1.write(data.ch[1]);
      Serial.print(data.ch[0]);
      Serial.print(" ");
      Serial.print(data.ch[1]);
      Serial.print(" ");
      Serial.println(data.ch[2]);
      lastData = data;
    }

    //   static unsigned long previousMillis = 0;
    //   static int ZoomPWMVal = 17;

    //   // save the current time
    //   unsigned long currentMillis = millis();
    //   // if 15ms have passed since the last time the loop ran
    //   if (currentMillis - previousMillis >= 15)
    //   {
    //     // save the last time the loop ran
    //     previousMillis = currentMillis;
    //     // write the SBUS data
    //     sbus_tx.data(data);
    //     // Transmit SBUS data
    //     sbus_tx.Write();
    //   }

    //   // Zoom Control
    //   // If the desired zoom value is differnt from the current zoom value slowly change the zoom value until they match
    //   // Only Check the zoom value every 500ms
    //   static unsigned long previousMillisZoom = 0;
    //   // save the current time
    //   unsigned long currentMillisZoom = millis();
    //   // if 500ms have passed since the last time the loop ran
    //   if (currentMillisZoom - previousMillisZoom >= 1)
    //   {
    //     // save the last time the loop ran
    //     previousMillisZoom = currentMillisZoom;
    //     // Check the zoom value
    //     if (ZoomPWMValDesired != ZoomPWMVal)
    //     {
    //       if (ZoomPWMValDesired > ZoomPWMVal)
    //       {
    //         ZoomPWMVal++;
    //       }
    //       else if (ZoomPWMValDesired < ZoomPWMVal)
    //       {
    //         ZoomPWMVal--;
    //       }
    //       // Update the servo PWM value
    //       analogWrite(SERVO_PWM_PIN, ZoomPWMVal);
    //     }
    //   }
  }
}
