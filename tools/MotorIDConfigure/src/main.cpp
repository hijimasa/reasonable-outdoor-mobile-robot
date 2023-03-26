#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_can.h"

const int SPI_CS_PIN = 9;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

void setup() {
  Serial.begin(9600);
  while(!Serial){};

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");
}

unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
void loop() {
  Serial.print("Enter motor ID :");
  while (Serial.available() == 0) {}     //wait for data available
  String motor_id_str = Serial.readString();
  if (motor_id_str[0] < '0' && motor_id_str[0] > '7')
  {
    Serial.println("Invalid ID entered! Please reset!");
    while (1)
    {
      delay(100);
    }
  }
  stmp[0] = motor_id_str[0] - '0'; 
  CAN.sendMsgBuf(0x108, 0, 8, stmp);
  delay(100);                       // send data per 100ms
  Serial.println("CAN BUS sendMsgBuf ok!");

  while (1)
  {
    delay(100);
  }
}