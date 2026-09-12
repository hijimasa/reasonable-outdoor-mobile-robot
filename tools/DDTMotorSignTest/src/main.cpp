/**
 * Which way does a positive duty turn the wheel, and which sign comes back?
 *
 * WHEELS OFF THE GROUND. This drives each motor open loop, one at a time, and
 * prints the status the motor returns, so the two mappings the controller
 * assumes can be read off instead of inferred:
 *
 *   duty sign  -> which way the wheel physically turns
 *   turn       -> the sign of the velocity and angle the motor reports
 *
 * The controller's PID needs those two to agree. If they do not, the error
 * grows instead of shrinking and the duty walks to the rail: commanded -1 rpm,
 * measured +88, which is what this robot does today.
 *
 * It runs one pass and then holds every motor at zero. Output is CSV at
 * 500000 baud, the same rate the real firmware uses:
 *
 *   ms,phase,motor,duty,vel0,vel1,cur0,cur1,ang0,ang1
 *
 * The emergency and free-rotation inputs are honoured: emergency forces duty
 * to zero, free rotation puts the motors in their disabled mode, same sense as
 * the real firmware.
 */
#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_can.h"

const int SPI_CS_PIN = 9;
mcp2515_can CAN(SPI_CS_PIN);

const int EMERGENCY_PIN = 7;
const int FREE_ROTATION_PIN = 6;

const int motor_total_num = 2;
const unsigned long HOST_BAUD = 500000;
const unsigned long CAN_REPLY_TIMEOUT_US = 20000;

// Ramp to this rather than stepping: if the wheel does not break away at a low
// duty the ramp shows where it does, and a step into a stiff drivetrain tells
// you nothing about the threshold.
const int TEST_DUTY = 4000;          // of 32767
const unsigned long RAMP_MS = 2500;
const unsigned long HOLD_MS = 1000;
const unsigned long REST_MS = 1500;

int reported_vel[4] = {0, 0, 0, 0};
int reported_cur[4] = {0, 0, 0, 0};
int reported_ang[4] = {0, 0, 0, 0};

static bool waitForCanFrame(unsigned long timeout_us)
{
  const unsigned long start = micros();
  while (CAN_MSGAVAIL != CAN.checkReceive())
  {
    if (micros() - start > timeout_us)
    {
      return false;
    }
  }
  return true;
}

static void sendDuty(int duty0, int duty1)
{
  unsigned char payload[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  payload[0] = highByte(duty0);
  payload[1] = lowByte(duty0);
  payload[2] = highByte(duty1);
  payload[3] = lowByte(duty1);
  CAN.sendMsgBuf(0x32, 0, 8, payload);
}

static void readStatus()
{
  unsigned char read_len;
  unsigned char read_buf[16];
  unsigned char request[8] = {0x00, 0x01, 0x02, 0x04, 0xAA, 0, 0, 0};
  for (int motor_num = 0; motor_num < motor_total_num; motor_num++)
  {
    request[0] = motor_num + 1;
    CAN.sendMsgBuf(0x107, 0, 8, request);
    if (waitForCanFrame(CAN_REPLY_TIMEOUT_US))
    {
      CAN.readMsgBuf(&read_len, read_buf);
      // Signed on purpose. The reply carries int16 two's complement; reading it
      // unsigned is one of the things this sketch exists to rule out.
      reported_vel[motor_num] = (int)((read_buf[0] << 8) | read_buf[1]);
      reported_cur[motor_num] = (int)((read_buf[2] << 8) | read_buf[3]);
      reported_ang[motor_num] = (int)((read_buf[4] << 8) | read_buf[5]);
    }
  }
}

void setup()
{
  pinMode(EMERGENCY_PIN, INPUT_PULLUP);
  pinMode(FREE_ROTATION_PIN, INPUT_PULLUP);
  Serial.begin(HOST_BAUD);

  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
    delay(100);
  }

  unsigned char registor_stmp[8] = {0x00, 0x00, 0, 0, 0, 0, 0, 0};
  CAN.sendMsgBuf(0x109, 0, 8, registor_stmp);
  delay(10);
  unsigned char feedback_stmp[8] = {0x80, 0x80, 0, 0, 0, 0, 0, 0};
  CAN.sendMsgBuf(0x106, 0, 8, feedback_stmp);
  delay(10);

  // Drain whatever the motors queued while all that went out.
  while (waitForCanFrame(50000))
  {
    unsigned char read_len;
    unsigned char read_buf[16];
    CAN.readMsgBuf(&read_len, read_buf);
  }

  const unsigned char open_loop_mode[8] = {0x00, 0x00, 0, 0, 0, 0, 0, 0};
  CAN.sendMsgBuf(0x105, 0, 8, open_loop_mode);
  delay(10);
  while (waitForCanFrame(50000))
  {
    unsigned char read_len;
    unsigned char read_buf[16];
    CAN.readMsgBuf(&read_len, read_buf);
  }

  Serial.println();
  Serial.println(F("ms,phase,motor,duty,vel0,vel1,cur0,cur1,ang0,ang1"));
}

// Four passes: motor 0 forward, motor 0 reverse, motor 1 forward, motor 1
// reverse. The other motor is held at zero throughout, so nothing has to be
// untangled afterwards.
struct Pass { int motor; int sign; const char *name; };
const Pass passes[] = {
  {0, +1, "m0+"}, {0, -1, "m0-"}, {1, +1, "m1+"}, {1, -1, "m1-"},
};
const int pass_count = sizeof(passes) / sizeof(passes[0]);

int pass_index = 0;
unsigned long phase_start = 0;
bool finished = false;

void loop()
{
  const int free_rotation = digitalRead(FREE_ROTATION_PIN);
  const int emergency = digitalRead(EMERGENCY_PIN);

  if (free_rotation == LOW)                       // free rotation requested
  {
    const unsigned char disable_mode[8] = {0x09, 0x09, 0, 0, 0, 0, 0, 0};
    CAN.sendMsgBuf(0x105, 0, 8, disable_mode);
    readStatus();
    Serial.print(millis()); Serial.println(F(",free,-,0,,,,,,"));
    delay(100);
    phase_start = millis();                       // do not count this against a pass
    return;
  }

  if (phase_start == 0)
  {
    phase_start = millis();
  }
  const unsigned long t = millis() - phase_start;
  const unsigned long pass_len = RAMP_MS + HOLD_MS + REST_MS;

  int duty = 0;
  const char *phase = "rest";
  if (!finished)
  {
    if (t < RAMP_MS)
    {
      duty = (int)((long)TEST_DUTY * t / RAMP_MS);
      phase = "ramp";
    }
    else if (t < RAMP_MS + HOLD_MS)
    {
      duty = TEST_DUTY;
      phase = "hold";
    }
    duty *= passes[pass_index].sign;
  }
  else
  {
    phase = "done";
  }

  if (emergency != LOW)                           // emergency engaged
  {
    duty = 0;
    phase = "estop";
  }

  const int motor = finished ? -1 : passes[pass_index].motor;
  sendDuty(motor == 0 ? duty : 0, motor == 1 ? duty : 0);
  readStatus();

  Serial.print(millis()); Serial.print(',');
  Serial.print(phase); Serial.print(',');
  Serial.print(motor); Serial.print(',');
  Serial.print(duty); Serial.print(',');
  Serial.print(reported_vel[0]); Serial.print(',');
  Serial.print(reported_vel[1]); Serial.print(',');
  Serial.print(reported_cur[0]); Serial.print(',');
  Serial.print(reported_cur[1]); Serial.print(',');
  Serial.print(reported_ang[0]); Serial.print(',');
  Serial.println(reported_ang[1]);

  if (!finished && t >= pass_len)
  {
    phase_start = millis();
    pass_index++;
    if (pass_index >= pass_count)
    {
      finished = true;
      pass_index = pass_count - 1;
    }
  }
  delay(100);
}
