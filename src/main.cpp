#include <Arduino.h>
#include "sbus.h"
#include <ADC.h>

/* SBUS data */
bfs::SbusData data;

#define MAX_MESSAGE_LENGTH 100
ADC TeensyADC;
// Data Channel 0 = Zoom
// Data Channel 1 = Iris
// Data Channel 2 = Focus
#define DEFAULTS_BUTTON 2
#define FOCUS_KNOB_PIN 21
#define SERVO_PWM_PIN 22

#define HC_12_SETPIN 22

#define DEBOUNCE_DELAY 500

int16_t ZoomPWMValDesired = 17;
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
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
    data.ch[3]++; 
  }
}

bool ChanelChangeMode = false;
void setup()
{
  delay(3000);
  data.ch[0] = Zoom12Val;
  data.ch[1] = Iris_2_8_Val;
  data.ch[2] = 555;
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
  if (digitalRead(10))
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

  TeensyADC.setAveraging(16);
  TeensyADC.setResolution(12);
  Serial.println("Setup Complete");
}

int readStablePot(int potPin, int threshold)
{
  static int potVal = 0;
  static int potLast = 0;

  potVal = TeensyADC.analogRead(potPin);

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
int ErrorCount = 0;
void loop()
{
  if (ChanelChangeMode)
  {
    char array[3];
    int buttonPressed = 0;
    // Read all buttons 2 thru 10
    while (buttonPressed == 0)
    {

      for (int i = 2; i < 10; i++)
      {
        if (digitalRead(i) == HIGH)
        {
          buttonPressed = i - 1;
        }
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
  }
  else
  {

    static bfs::SbusData lastData;
    static int lastFocusedMapped = 0;
    static long lastLoopTime = 0;

    // If 20 ms has passsed since last loop
    if (millis() - lastLoopTime > 20)
    {

      // Focus Control

      int FocusRaw = readStablePot(FOCUS_KNOB_PIN, 5);


      int FocusMapped = map(FocusRaw, 0, 4095, 462, 985);

      data.ch[2] = FocusMapped;
    }

    // if data has changed transmit it over serial1
    if (data.ch[0] != lastData.ch[0] ||
        data.ch[1] != lastData.ch[1] ||
        data.ch[2] != lastData.ch[2] ||
        data.ch[3] != lastData.ch[3])
    {
      


      Serial1.flush();
      Serial1.print("START");
      // Serial1.write(data.ch[0] & 0xFF);
      // Serial1.write(data.ch[0] >> 8);
      Serial1.write(ZoomPWMValDesired &0xFF);
      Serial1.write(ZoomPWMValDesired >> 8 & 0xFF);

      Serial1.write(data.ch[1] & 0xFF);
      Serial1.write(data.ch[1] >> 8);

      Serial1.write(data.ch[2] & 0xFF);
      Serial1.write(data.ch[2] >> 8);

      //Generate Checksunm
      uint16_t checksum = 0;
      checksum += data.ch[0];
      checksum += data.ch[1];
      checksum += data.ch[2];
      // Write Checksum
      Serial1.write(checksum & 0xFF);
      Serial1.write(checksum >> 8);
      
      Serial1.print("END");
      lastData = data;
      bool searching = true;
      // temp array for 8 bytes
      byte temp[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
      while (searching)
      {
        delay(10);
        // for loop to move data down one slot in temp array
        for (auto i = 0; i < 5; i++)
        {
          temp[i] = temp[i + 1];
        }
        // read one byte from serial into back of temp array
        temp[4] = Serial1.read();
        // print out the temp array

        if (temp[0] == 'S' && temp[1] == 'T' && temp[2] == 'A' && temp[3] == 'R' && temp[4] == 'T')
        {
          // if start code found, break out of while loop
          searching = false;
          Serial1.readBytes(temp + 5, 6);
          // Serial.println("                                                    Start Code Found");
          if (data.ch[0] != (temp[5] | (temp[6] << 8)) &&
              data.ch[1] != (temp[7] | (temp[8] << 8)) &&
              data.ch[2] != (temp[9] | (temp[10] << 8)))
          {
            ErrorCount++;
            // data.ch[3]++; // Increment unused channel to force data to be resent
          }
        }else         if (Serial1.available() == 0)
        {
          searching = false;
          ErrorCount++;
          // data.ch[3]++; // Increment unused channel to force data to be resent
        }
        
      }
      // Serial1.write(data.ch[1]);
      Serial.print(data.ch[0]);
      Serial.print(" ");
      Serial.print(data.ch[1]);
      Serial.print(" ");
      Serial.print(data.ch[2]);
      Serial.print(" ");
      Serial.print(data.ch[3]);
      Serial.print(" Error Count: ");
      Serial.println(ErrorCount);
    }
  }
}
