#include <JeeLib.h>
#include <Ports.h>
#include <RF12.h>

// Global variables
int Val;

void setup () {
  rf12_initialize('R', RF12_433MHZ, 88);
  Serial.begin(9600);
}

void loop () {
  if (rf12_recvDone() & (rf12_crc == 0)) {
    for(int x = 3; x < rf12_len; x = x + 2){
      Val = rf12_data[x + 1] | rf12_data[x] << 8;
      Serial.print(Val);
      Serial.print(" ");
    }
  //delay(100);
  Serial.println();
  }
}
