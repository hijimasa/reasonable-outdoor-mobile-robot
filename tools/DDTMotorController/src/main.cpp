#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_can.h"

const int SPI_CS_PIN = 9;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

const int EMERGENCY_PIN = 7;
const int FREE_ROTATION_PIN = 6;

const int motor_total_num = 2;
int motor_velocities[8]         = {0, 0, 0, 0, 0, 0, 0, 0};
int current_motor_velocities[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int current_motor_currents[8]   = {0, 0, 0, 0, 0, 0, 0, 0};
int current_motor_angles[8]     = {0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
  unsigned char read_len;
  unsigned char read_buf[16];

  pinMode(EMERGENCY_PIN, INPUT_PULLUP);
  pinMode(FREE_ROTATION_PIN, INPUT_PULLUP);

  Serial.begin(9600);

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
    delay(100);
  }

  unsigned char registor_stmp[8] = {0x01, 0x01, 0, 0, 0, 0, 0, 0};
  CAN.sendMsgBuf(0x109, 0, 8, registor_stmp);

  unsigned char feedback_stmp[8] = {0x80, 0x80, 0, 0, 0, 0, 0, 0};
  CAN.sendMsgBuf(0x106, 0, 8, feedback_stmp);

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

void loop() {
  unsigned char read_len;
  unsigned char read_buf[16];

  int free_rotation_pin_mode = digitalRead(FREE_ROTATION_PIN);
  int emergency_pin_mode = digitalRead(EMERGENCY_PIN);

  if (free_rotation_pin_mode == LOW) // free
  {
    const unsigned char disable_mode_stmp[8] = {0x09, 0x09, 0, 0, 0, 0, 0, 0};
    CAN.sendMsgBuf(0x105, 0, 8, disable_mode_stmp);
    for (int motor_num = 0; motor_num < motor_total_num; motor_num++)
    {
      while (CAN_MSGAVAIL != CAN.checkReceive())
      {
        delay(10);
      }
      CAN.readMsgBuf(&read_len, read_buf);
    }
  }
  else // normal
  {
    const unsigned char velocity_mode_stmp[8] = {0x02, 0x02, 0, 0, 0, 0, 0, 0};
    CAN.sendMsgBuf(0x105, 0, 8, velocity_mode_stmp);
    for (int motor_num = 0; motor_num < motor_total_num; motor_num++)
    {
      while (CAN_MSGAVAIL != CAN.checkReceive())
      {
        delay(10);
      }
      CAN.readMsgBuf(&read_len, read_buf);
    }

    unsigned char velocities_stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if (emergency_pin_mode == LOW) // not emergency
    {
      for (int motor_num = 0; motor_num < 4; motor_num++)
      {
        velocities_stmp[motor_num*2] = highByte(motor_velocities[motor_num]);
        velocities_stmp[motor_num*2+1] = lowByte(motor_velocities[motor_num]);
      }
    }
    CAN.sendMsgBuf(0x32, 0, 8, velocities_stmp);
    if (emergency_pin_mode == LOW) // not emergency
    {
      for (int motor_num = 4; motor_num < 8; motor_num++)
      {
        velocities_stmp[(motor_num-4)*2] = highByte(motor_velocities[motor_num]);
        velocities_stmp[(motor_num-4)*2+1] = lowByte(motor_velocities[motor_num]);
      }
    }
    CAN.sendMsgBuf(0x33, 0, 8, velocities_stmp);
  }

  // put your main code here, to run repeatedly:
  byte command[256];
  int key = Serial.available();

  if (key >= motor_total_num*2+2)
  {
    byte check_byte = 0;
    for (int i = 0; i < key; i++)
    {
      command[i] = Serial.read();

      if (i < motor_total_num*2+1)
      {
        check_byte += command[i];
      }
    }

    {
      Serial.write((byte)85);
      byte check_byte = 85;
      unsigned char get_status_stmp[8] = {0x01, 0x01, 0x02, 0x04, 0x55, 0, 0, 0};
      for (int motor_num = 0; motor_num < motor_total_num; motor_num++)
      {
        //get_status_stmp[0] = motor_num + 1;
        CAN.sendMsgBuf(0x107, 0, 8, get_status_stmp);
        CAN.readMsgBuf(&read_len, read_buf);
        Serial.println((char *)read_buf);
        Serial.write(highByte(current_motor_angles[motor_num]));
        Serial.write(lowByte(current_motor_angles[motor_num]));
        check_byte += highByte(current_motor_angles[motor_num]);
        check_byte += lowByte(current_motor_angles[motor_num]);
        Serial.write(highByte(current_motor_velocities[motor_num]));
        Serial.write(lowByte(current_motor_velocities[motor_num]));
        check_byte += highByte(current_motor_velocities[motor_num]);
        check_byte += lowByte(current_motor_velocities[motor_num]);
        Serial.write(highByte(current_motor_currents[motor_num]));
        Serial.write(lowByte(current_motor_currents[motor_num]));
        check_byte += highByte(current_motor_currents[motor_num]);
        check_byte += lowByte(current_motor_currents[motor_num]);
      }
      Serial.write(check_byte);
    }

    if (command[0] == 85 && check_byte == command[motor_total_num*2+1])
    {
      for (int motor_num = 0; motor_num < motor_total_num; motor_num++)
      {
        motor_velocities[motor_num] = (command[2*motor_num+1]  << 8)+ command[2*motor_num+2];
      }
    }
  }

  unsigned char get_status_stmp[8] = {0x00, 0x01, 0x02, 0x04, 0, 0, 0, 0};
  for (int motor_num = 0; motor_num < motor_total_num; motor_num++)
  {
    get_status_stmp[0] = motor_num + 1;
    CAN.sendMsgBuf(0x107, 0, 8, get_status_stmp);
    while (CAN_MSGAVAIL != CAN.checkReceive())
    {
      delay(10);
    }
    CAN.readMsgBuf(&read_len, read_buf);
    current_motor_velocities[motor_num] = (read_buf[0] << 8) + read_buf[1];
    current_motor_currents[motor_num] = (read_buf[2] << 8) + read_buf[3];
    current_motor_angles[motor_num] = (read_buf[4] << 8) + read_buf[5];
  }

  delay(10);
}