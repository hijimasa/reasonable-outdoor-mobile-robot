#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_can.h"

const int SPI_CS_PIN = 9;
mcp2515_can CAN(SPI_CS_PIN);

const int EMERGENCY_PIN = 7;
const int FREE_ROTATION_PIN = 6;

const int motor_total_num = 2;
const int motor_decelation = 2000;
int motor_velocities[8]         = {0, 0, 0, 0, 0, 0, 0, 0};
int current_motor_velocities[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int current_motor_currents[8]   = {0, 0, 0, 0, 0, 0, 0, 0};
int current_motor_angles[8]     = {0, 0, 0, 0, 0, 0, 0, 0};
int current_command[8]          = {0, 0, 0, 0, 0, 0, 0, 0};
int diff_vel[8][4]             = {{0}};
int diff_vel_index              = 0;
int k_p                         =  80;
int k_d                         =  10;
int k_i                         =  40;
int timeout_count               = 0;
bool is_drive_mode = false;

void setup() {
  unsigned char read_len;
  unsigned char read_buf[16];

  pinMode(EMERGENCY_PIN, INPUT_PULLUP);
  pinMode(FREE_ROTATION_PIN, INPUT_PULLUP);

  Serial.begin(9600);

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
    delay(100);
  }

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

    is_drive_mode = false;
  }
  else // normal
  {
    if (is_drive_mode == false)
    {
      is_drive_mode = true;
      const unsigned char open_loop_mode_stmp[8] = {0x00, 0x00, 0, 0, 0, 0, 0, 0};
      CAN.sendMsgBuf(0x105, 0, 8, open_loop_mode_stmp);
      for (int motor_num = 0; motor_num < motor_total_num; motor_num++)
      {
        while (CAN_MSGAVAIL != CAN.checkReceive())
        {
          delay(10);
        }
        CAN.readMsgBuf(&read_len, read_buf);
      }
      delay(10);
    }

    unsigned char velocities_stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if (emergency_pin_mode == LOW) // not emergency
    {
      for (int motor_num = 0; motor_num < 4; motor_num++)
      {
        int old_diff_index = diff_vel_index - 1;
        if (old_diff_index < 0)
        {
          old_diff_index += 4;
        }
        int old_diff_index2 = diff_vel_index - 2;
        if (old_diff_index2 < 0)
        {
          old_diff_index += 4;
        }
        int diff_control_amount = k_p * (diff_vel[motor_num][diff_vel_index] - diff_vel[motor_num][old_diff_index])
                                 + k_d * ((diff_vel[motor_num][diff_vel_index] - diff_vel[motor_num][old_diff_index]) - (diff_vel[motor_num][old_diff_index] - diff_vel[motor_num][old_diff_index2]))
                                 + k_i * diff_vel[motor_num][diff_vel_index];
        if ((long) diff_control_amount + (long) current_command[motor_num] > 32767)
        {
          current_command[motor_num] = 32767;
        }
        else if ((long) diff_control_amount + (long) current_command[motor_num] < -32767)
        {
          current_command[motor_num] = -32767;
        }
        else
        {
          current_command[motor_num] += diff_control_amount;
        }
        Serial.println(motor_num);
        Serial.println(current_command[motor_num]);
        Serial.println(diff_vel_index);
        Serial.println(diff_control_amount);
        Serial.println();
        velocities_stmp[motor_num*2] = highByte(current_command[motor_num]);
        velocities_stmp[motor_num*2+1] = lowByte(current_command[motor_num]);
      }
    }
    else
    {
      for (int motor_num = 0; motor_num < 4; motor_num++)
      {
        if (current_command[motor_num] > 0)
        {
          current_command[motor_num] -= motor_decelation;
          if (current_command[motor_num] < 0)
          {
            current_command[motor_num] = 0;
          }
        }
        if (current_command[motor_num] < 0)
        {
          current_command[motor_num] += motor_decelation;
          if (current_command[motor_num] > 0)
          {
            current_command[motor_num] = 0;
          }
        }
        velocities_stmp[motor_num*2] = highByte(current_command[motor_num]);
        velocities_stmp[motor_num*2+1] = lowByte(current_command[motor_num]);
      }
    }
    CAN.sendMsgBuf(0x32, 0, 8, velocities_stmp);
    if (emergency_pin_mode == LOW) // not emergency
    {
      for (int motor_num = 4; motor_num < 8; motor_num++)
      {
        int old_diff_index = diff_vel_index - 1;
        if (old_diff_index < 0)
        {
          old_diff_index += 4;
        }
        int old_diff_index2 = diff_vel_index - 2;
        if (old_diff_index2 < 0)
        {
          old_diff_index += 4;
        }
        int diff_control_amount = k_p * (diff_vel[motor_num][diff_vel_index] - diff_vel[motor_num][old_diff_index])
                                 + k_d * ((diff_vel[motor_num][diff_vel_index] - diff_vel[motor_num][old_diff_index]) - (diff_vel[motor_num][old_diff_index] - diff_vel[motor_num][old_diff_index2]))
                                 + k_i * diff_vel[motor_num][diff_vel_index];
        if ((long) diff_control_amount + (long) current_command[motor_num] > 32767)
        {
          current_command[motor_num] = 32767;
        }
        else if ((long) diff_control_amount + (long) current_command[motor_num] < -32767)
        {
          current_command[motor_num] = -32767;
        }
        else
        {
          current_command[motor_num] += diff_control_amount;
        }
        velocities_stmp[(motor_num-4)*2] = highByte(current_command[motor_num]);
        velocities_stmp[(motor_num-4)*2+1] = lowByte(current_command[motor_num]);
      }
    }
    else
    {
      for (int motor_num = 4; motor_num < 8; motor_num++)
      {
        if (current_command[motor_num] > 0)
        {
          current_command[motor_num] -= motor_decelation;
          if (current_command[motor_num] < 0)
          {
            current_command[motor_num] = 0;
          }
        }
        if (current_command[motor_num] < 0)
        {
          current_command[motor_num] += motor_decelation;
          if (current_command[motor_num] > 0)
          {
            current_command[motor_num] = 0;
          }
        }
        velocities_stmp[motor_num*2] = highByte(current_command[motor_num]);
        velocities_stmp[motor_num*2+1] = lowByte(current_command[motor_num]);
      }
    }
    CAN.sendMsgBuf(0x33, 0, 8, velocities_stmp);
  }

  // put your main code here, to run repeatedly:
  byte command[256];
  int key = Serial.available();

  if (key >= motor_total_num*2+2)
  {
    timeout_count = 0;

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
      for (int motor_num = 0; motor_num < motor_total_num; motor_num++)
      {
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

    if (command[0] == 85 && check_byte == command[motor_total_num*2+1] && emergency_pin_mode == LOW)
    {
      for (int motor_num = 0; motor_num < motor_total_num; motor_num++)
      {
        motor_velocities[motor_num] = (command[2*motor_num+1]  << 8)+ command[2*motor_num+2];
      }
    }
  }
  else
  {
    timeout_count++;

    if (timeout_count > 100)
    {
      timeout_count = 0;
      for (int motor_num = 0; motor_num < motor_total_num; motor_num++)
      {
        motor_velocities[motor_num] = 0;
      }
    }
  }

  unsigned char get_status_stmp[8] = {0x00, 0x01, 0x02, 0x04, 0xAA, 0, 0, 0};
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
    
    diff_vel_index++;
    if (diff_vel_index >= 4)
    {
      diff_vel_index = 0;
    }
    if (emergency_pin_mode == LOW && free_rotation_pin_mode == HIGH) // not emergency
    {
      diff_vel[motor_num][diff_vel_index] = motor_velocities[motor_num] - current_motor_velocities[motor_num];
    }
    else // command becomes zero if emergency is enable.
    {
      diff_vel[motor_num][diff_vel_index] = 0;
    }
  }

  delay(10);
}
