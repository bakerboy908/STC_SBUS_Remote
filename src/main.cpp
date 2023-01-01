#include <Arduino.h>
#include "sbus.h"
#include "map.h"
// #include "typedef.hs"
// https://arduino-projekte.info/wp-content/uploads/2020/12/D1_mini_ESP32_pinout.jpg
/* SBUS object, reading SBUS */
bfs::SbusRx  sbus_rx(&Serial1);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial1);
/* SBUS data */
bfs::SbusData data;

#define MAX_MESSAGE_LENGTH 100

//Data Channel 0 = Zoom
//Data Channel 1 = Iris
//Data Channel 2 = Focus
#define DEFAULTS_BUTTON 2

// 8 int values for Zoom possitions
int16_t Zoom12Val = 0;
int16_t Zoom14Val = 0;
int16_t Zoom15Val = 0;
int16_t Zoom18Val = 0;
int16_t Zoom25Val = 0;
int16_t Zoom30Val = 0;
int16_t Zoom35Val = 0;
int16_t Zoom40Val = 0;


//ISR Functions to set the Zoom Values
void Zoom12() {
  data.ch[2] = Zoom12Val;
}

void Zoom14() {
  data.ch[2] = Zoom14Val;
}

void Zoom15() {
  data.ch[2] = Zoom15Val;
}

void Zoom18() {
  data.ch[2] = Zoom18Val;
}

void Zoom25() {
  data.ch[2] = Zoom25Val;
}

void Zoom30() {
  data.ch[2] = Zoom30Val;
}

void Zoom35() {
  data.ch[2] = Zoom35Val;
}

void Zoom40() {
  data.ch[2] = Zoom40Val;
}
// 10 int16_t values for Iris possitions
int16_t Iris_2_8_Val = 0;
int16_t Iris_3_2_Val = 0;
int16_t Iris_4_0_Val = 0;
int16_t Iris_4_5_Val = 0;
int16_t Iris_4_8_Val = 0;
int16_t Iris_5_6_Val = 0;
int16_t Iris_6_2_Val = 0;
int16_t Iris_6_7_Val = 0;
int16_t Iris_7_3_Val = 0;
int16_t Iris_8_7_Val = 0;


//ISR Function to set the Iris Value

void Iris_2_8() {
  data.ch[1] = Iris_2_8_Val;
}

void Iris_3_2() {
  data.ch[1] = Iris_3_2_Val;
}

void Iris_4_0() {
  data.ch[1] = Iris_4_0_Val;
}

void Iris_4_5() {
  data.ch[1] = Iris_4_5_Val;
}

void Iris_4_8() {
  data.ch[1] = Iris_4_8_Val;
}

void Iris_5_6() {
  data.ch[1] = Iris_5_6_Val;
}

void Iris_6_2() {
  data.ch[1] = Iris_6_2_Val;
}

void Iris_6_7() {
  data.ch[1] = Iris_6_7_Val;
}

void Iris_7_3() {
  data.ch[1] = Iris_7_3_Val;
}

void Iris_8_7() {
  data.ch[1] = Iris_8_7_Val;
}
// Default values for the Zoom and Iris

//Defaults ISR
void Defaults() {
  data.ch[0] = Zoom12Val;
  data.ch[1] = Iris_2_8_Val;
}


void setup() {
  //List of all ISR functions, enables attaching ISR's in a loop
  void (*isrList[])() = { Zoom12, Zoom14, Zoom15, Zoom18, Zoom25, Zoom30, Zoom35, 
                          Zoom40, Iris_2_8, Iris_3_2, Iris_4_0, Iris_4_5, Iris_4_8,
                          Iris_5_6, Iris_6_2, Iris_6_7, Iris_7_3, Iris_8_7, Defaults};
  
  /* Serial to display data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();

  Serial.println("SBUS RX/TX started");
// Define pins 2 thru 22 as inputs with pullup resistors
  for (int i = 2; i < 23; i++) {
    pinMode(i, INPUT_PULLUP);
  }
  Serial.println("Pins 2 thru 22 set as inputs with pullup resistors");
// attach interupt to pin 2 thru 22
  for (int i = 2; i < 23; i++) {
    attachInterrupt(digitalPinToInterrupt(i), isrList[i], FALLING);
  }
  Serial.println("Interupts attached to pins 2 thru 22");
  //Setup Complete
  Serial.println("Setup Complete");

}


void loop () {
 //Check to see if anything is available in the serial receive buffer
//  while (Serial.available() > 0)
//  {
//    static char message[MAX_MESSAGE_LENGTH];
//    static unsigned int message_pos = 0;
//    //Create a place to hold the incoming message

//    //Read the next available byte in the serial receive buffer
//    char inByte = Serial.read();

//    //Message coming in (check not terminating character) and guard for over message size
//    if ( inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1) )
//    {
//      //Add the incoming byte to our message
//      message[message_pos] = inByte;
//      message_pos++;
//    }
//    //Full message received...
//    else
//    {
//      //Add null character to string
//      message[message_pos] = '\0';

//      //Print the message (or do other things)
//      Serial.println(message);
//     test = message;
//      //Reset for the next message
//      message_pos = 0;
//    }
//  }
  
  // if (sbus_rx.Read()) {
  //   /* Grab the received data */
  //   data = sbus_rx.data();
  //   /* Display the received data */
  //   // for (int8_t i = 0; i < 4; i++) {
      
  //   //   Serial.print(" Chanel: ");
  //   //   Serial.print(i);
  //   //   Serial.print(" Data: ");
  //   //   Serial.print(data.ch[i]);
  //   // }
  //   //   Serial.println();

  //   // data.ch[1] += 1; 
  //   /* Set the SBUS TX data to the received data */
  //   data.ch[1] = test.toInt();
  //   sbus_tx.data(data);
  //   /* Write the data to the servos */
  //   sbus_tx.Write();

  // }
  }

