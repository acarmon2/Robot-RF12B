// Libraries for transceiver RF12B
#include <JeeLib.h>
#include <Ports.h>
#include <RF12.h>

// Libraries for IMU
#include "GY_85.h"
#include <Wire.h>

// Objects
GY_85 GY85;

// Global variables
byte OutData, Pending;
char cont;
byte Data[15] = {};
MilliTimer sendTimer;
char a;
// PWM controllers
int vR = 9;
int vL = 3;
int dirL1 = 10;

void setup(){
  // D7 = BIN2, D6 = BIN1, D5 = STBY, D4 = AIN1, D3 = AIN2
  DDRD = B11111000;
  DDRD = DDRD | B11111000;
  // B1 = vR, B2 = vL
  pinMode(vR, OUTPUT);
  pinMode(vL, OUTPUT);
  pinMode(dirL1, OUTPUT);
  // Begin frame
  Data[0] = 0x13;
  Data[1] = 0x17;
  Data[2] = 0x25;
  // Use the timer 1
  bitSet(TCCR1B, WGM12);
  rf12_initialize('T', RF12_433MHZ, 88);
  Serial.begin(9600);
  Wire.begin();
  GY85.init();
  cont = 1;
  delay(10);
}


// Extract IMU information
char * IMU(){
  // 6 bytes Accel, 6 bytes Compass, 12 bytes Gyros 
  static char Data[24];
  
  return Data;
}


// Move to right, velR is the velocity 0-255 in the PWM
// dir is clockwise or counter clockwise
void moveR(char velR, char dirR){
    //set PWM and in dir 1 = CW, 0 = CCW
    analogWrite(vR, velR);
    if(dirR == 0){
      PORTD &= ~_BV(PORTD6);
      PORTD |= _BV(PORTD7);
    }
    else if (dirR == 1){
      PORTD |= _BV(PORTD6);
      PORTD &= ~_BV(PORTD7);
    }
    // Enable stanby
    PORTD |= _BV(PORTD5);
}

// Move to left, velL is the velocity 0-255 in the PWM
// dir is clockwise or counter clockwise
void moveL(char velL, char dirL){
    //set PWM and in dir 1 = CW, 0 = CCW
    analogWrite(vL, velL);
    if(dirL == 0){
      PORTD &= ~_BV(PORTD4);
      PORTB |= _BV(PORTB2);
    }
    else if (dirL == 1){
      PORTD |= _BV(PORTD4);
      PORTB &= ~_BV(PORTB2);
    }
    // Enable stanby
    PORTD |= _BV(PORTD5);
}

// Function to stop, disable the stanby
void stopMov(){
  PORTD |= ~_BV(PORTD5);
  // Cancel the PWM
  analogWrite(vR, 0);
  analogWrite(vL, 0);
}

void loop(){
  // Acceleration in x
  int ax = GY85.accelerometer_x(GY85.readFromAccelerometer());
  Data[3] = (byte)((ax >> 8) & 0xFF);                  // MSV
  Data[4] = (byte)(ax & 0xFF);                         // LSV
  // Acceleration in y
  int ay = GY85.accelerometer_y(GY85.readFromAccelerometer());
  Data[5] = (byte)((ay >> 8) & 0xFF);                  // MSV
  Data[6] = (byte)(ay & 0xFF);                         // LSV
  // Acceleration in z
  int az = GY85.accelerometer_z(GY85.readFromAccelerometer());
  Data[7] = (byte)((az >> 8) & 0xFF);                  // MSV
  Data[8] = (byte)(az & 0xFF);                         // LSV
  
  // Compass
  int hx = GY85.compass_x(GY85.readFromCompass());
  Data[9] = (byte)((hx >> 8) & 0xFF);                  // MSV
  Data[10] = (byte)(hx & 0xFF);                         // LSV
  // Compass
  int hy = GY85.compass_y(GY85.readFromCompass());
  Data[11] = (byte)((hy >> 8) & 0xFF);                  // MSV
  Data[12] = (byte)(hy & 0xFF);                         // LSV
  // Compass
  int hz = GY85.compass_z(GY85.readFromCompass());
  Data[13] = (byte)((hz >> 8) & 0xFF);                  // MSV
  Data[14] = (byte)(hz & 0xFF);                         // LSV
  
  
  
  // Emitter 
  /*rf12_recvDone();
  if(rf12_canSend()){
    rf12_sendStart(0, &Data, sizeof Data);
  }*/
  
  moveR(50, 1);
  moveL(50, 0);
  delay(10);
  
  // Emitter 
  for (char cont = 0; cont < 2; cont++){
  rf12_recvDone();
  if(rf12_canSend()){
    Serial.print("Alive");
    rf12_sendStart(0, &Data, sizeof Data);
  }
  }
}
