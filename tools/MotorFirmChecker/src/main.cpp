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

  unsigned char read_len;
  unsigned char read_buf[16];
  unsigned char registor_stmp[8] = {0x00, 0x00, 0, 0, 0, 0, 0, 0};
  CAN.sendMsgBuf(0x109, 0, 8, registor_stmp);
  delay(10);

  unsigned char feedback_stmp[8] = {0x80, 0x80, 0, 0, 0, 0, 0, 0};
  CAN.sendMsgBuf(0x106, 0, 8, feedback_stmp);
  delay(10);

  // flush can data
  while (1)
  {
    int timeout_count = 0;
    while (CAN_MSGAVAIL != CAN.checkReceive())
    {
      delay(10);
      timeout_count++;
      if (timeout_count > 100)
      {
        break;
      }
    }
    if (timeout_count > 100)
    {
      break;
    }

    CAN.readMsgBuf(&read_len, read_buf);
  }

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
  CAN.sendMsgBuf(0x10A, 0, 8, stmp);
  delay(100);                       // send data per 100ms
  Serial.println("CAN BUS sendMsgBuf ok!");

  // flush can data
  unsigned char read_len;
  unsigned char read_buf[16];
  while (1)
  {
    while (CAN_MSGAVAIL != CAN.checkReceive())
    {
      delay(10);
    }
    CAN.readMsgBuf(&read_len, read_buf);
    for (int i =0; i < read_len; i++)
    {
      Serial.println(read_buf[i]);
    }
  }
}