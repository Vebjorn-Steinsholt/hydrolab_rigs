// LinMot serial-to-CAN bridge for the sloshing oscillator
// Arduino Uno R3 (an Uno R4 WiFi also works) with an MCP2515 CAN shield (CS pin 10, INT pin 2),
// LinMot drive on CANopen node 12 (0x0C) at 500 kbit/s. Based on the UDP2CAN-BUS controller by
// Robert Opland and Vebjorn Steinsholt (NTNU); the link to the computer is now USB serial instead
// of Wi-Fi/UDP, and the go-to-position command counts up the motion command count instead of
// sending an empty header and an extra SYNC first.
//
// Serial link: 115200 baud, 8N1, binary frames, little-endian, CRC-16/CCITT-FALSE (poly 0x1021,
// init 0xFFFF, no reflection, no final XOR; Python: binascii.crc_hqx(data, 0xFFFF)) over every
// byte after the two sync bytes and before the CRC.
//
// Computer -> Arduino, 22 bytes:
//   0  u8   0xA5            sync
//   1  u8   0x5A            sync
//   2  u8   command         see CMD_* below
//   3  u8   sequence        echoed in replies; the computer counts it up
//   4  i32  position        0.1 um (LinMot units); 1 mm = 10000
//   8  u32  max velocity    1e-6 m/s; 1 m/s = 1000000
//   12 u32  acceleration    1e-5 m/s^2; 1 m/s^2 = 100000
//   16 u32  deceleration    1e-5 m/s^2
//   20 u16  CRC
// Fields a command does not use are sent as 0.
//
// Arduino -> computer, 14 bytes:
//   0  u8   0xA5            sync
//   1  u8   0x5A            sync
//   2  u8   type            REPLY_READY or REPLY_FEEDBACK
//   3  u8   sequence        of the last valid command
//   4  i32  actual position 0.1 um (0 in REPLY_READY)
//   8  u16  status word     LinMot status word (0 in REPLY_READY)
//   10 u8   flags           FLAG_* below
//   11 u8   bad frames      frames dropped for a bad CRC or sync since start (wraps at 255)
//   12 u16  CRC
//
// REPLY_READY is sent at start and in reply to CMD_PING. Opening the port can reset the board, so
// the computer sends CMD_PING until it gets REPLY_READY before anything else.
// REPLY_FEEDBACK is sent for every position/status frame the drive sends (CAN ID 0x18C). The drive
// sends one after each SYNC if its TPDO1 is synchronous. Each command sends one SYNC and so gets
// one feedback, except CMD_ACK_ERROR, CMD_SWITCH_ON from a switched-off drive, and the first
// CMD_GO_TO_POS after an abort, homing, a switch-off or an error acknowledge, which get two.
//
// Watchdog: after a go-to-position command, the Arduino expects a valid frame at least every
// WATCHDOG_MS. If none arrives it aborts the running motion command once and sets
// FLAG_WATCHDOG_TRIPPED until the next go-to-position command. The drive stays switched on.
//
// Drive state machine (LinMot Motion Control SW manual, chapter 3): after power-up or an error the
// drive switches on only after control word bit 0 has been cleared, so CMD_SWITCH_ON clears it
// first unless the drive already reports operation enabled. CMD_ACK_ERROR acknowledges a drive
// error with a rising edge of bit 7 and leaves the drive switched off.
//
// Sync watchdog: the drive can stop itself when SYNC frames stop (Legacy Sync Watchdog in
// LinMot-Talk), but only in NMT state operational. CMD_OPERATIONAL starts it; CMD_PRE_OPERATIONAL
// ends it, so a switched-off drive does not go into error while nothing is sending. In
// pre-operational the drive ignores PDOs: no commands, no feedback, so no reply to that command.
//
// Removed from the UDP version: the drive's built-in sine command (old choice 8), which keeps
// running when the computer stops sending, and the force-control commands (old choices 7, 10, 11).

#include <SPI.h>
#include <mcp_can.h>
#include <string.h>

// Hardware
#define CAN0_INT 2
MCP_CAN CAN0(10);
const int NODE_ID = 0x0C;
const unsigned long SERIAL_BAUD = 115200;

// Commands (numbers as in the UDP version)
const uint8_t CMD_PING = 0;              // reply REPLY_READY
const uint8_t CMD_OPERATIONAL = 1;       // CANopen NMT: node to operational
const uint8_t CMD_HOME = 2;              // start homing
const uint8_t CMD_GO_TO_POS = 3;         // VAI go to position (position, velocity, acc, dec)
const uint8_t CMD_SWITCH_ON = 4;
const uint8_t CMD_SWITCH_OFF = 5;
const uint8_t CMD_REQUEST_STATUS = 6;    // SYNC, so the drive sends position and status
const uint8_t CMD_ACK_ERROR = 7;         // acknowledge a drive error (was force control in UDP)
const uint8_t CMD_PRE_OPERATIONAL = 8;   // CANopen NMT: node to pre-operational; no reply

// Replies
const uint8_t REPLY_READY = 0x10;
const uint8_t REPLY_FEEDBACK = 0x01;

// Flags in replies
const uint8_t FLAG_WATCHDOG_TRIPPED = 0x01;  // motion aborted, no frame within WATCHDOG_MS
const uint8_t FLAG_CAN_SEND_ERROR = 0x02;    // a CAN frame could not be sent since the last reply
const uint8_t FLAG_REJECTED = 0x04;          // last command rejected: unknown or out of range

// Last line of defence; the controller on the computer checks tighter limits first. The rig's
// physical range is 5-195 mm, at most 2 m/s and 1 g; velocity and acceleration here allow for
// the margin the computer adds to the sine's peak values.
const long POSITION_MIN = 50000L;           // 5 mm
const long POSITION_MAX = 1950000L;         // 195 mm
const unsigned long VELOCITY_MAX = 3000000UL;      // 3 m/s
const unsigned long ACCELERATION_MAX = 2000000UL;  // 20 m/s^2

const unsigned long WATCHDOG_MS = 200;

const uint8_t SYNC_1 = 0xA5;
const uint8_t SYNC_2 = 0x5A;
const int COMMAND_FRAME_SIZE = 22;
const int REPLY_FRAME_SIZE = 14;

uint8_t rxFrame[COMMAND_FRAME_SIZE];
int rxCount = 0;
uint8_t lastSequence = 0;
uint8_t badFrames = 0;
uint8_t flags = 0;
uint16_t lastStatusWord = 0;             // from the drive's last feedback; 0 until the first one
uint8_t motionCommandCount = 0;          // low 4 bits of the motion command header
uint16_t lastControlWord = 0;            // last control word sent to the drive
bool watchdogArmed = false;
unsigned long lastValidFrameMs = 0;

// CRC-16/CCITT-FALSE
uint16_t crc16(const uint8_t * data, int length)
{
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }
  return crc;
}

// Serial replies

void sendReply(uint8_t type, long position, uint16_t statusWord)
{
  uint8_t frame[REPLY_FRAME_SIZE];
  frame[0] = SYNC_1;
  frame[1] = SYNC_2;
  frame[2] = type;
  frame[3] = lastSequence;
  memcpy(&frame[4], &position, 4);
  memcpy(&frame[8], &statusWord, 2);
  frame[10] = flags;
  frame[11] = badFrames;
  uint16_t crc = crc16(&frame[2], REPLY_FRAME_SIZE - 4);
  memcpy(&frame[12], &crc, 2);
  Serial.write(frame, REPLY_FRAME_SIZE);
  // The CAN send error and rejection are reported once; the watchdog flag stays until the next
  // go-to-position command.
  flags &= ~(FLAG_CAN_SEND_ERROR | FLAG_REJECTED);
}

// CAN frames to the drive (byte layout as in the UDP version)

void sendCan(unsigned long id, uint8_t length, uint8_t * data)
{
  if (CAN0.sendMsgBuf(id, 0, length, data) != CAN_OK) {
    flags |= FLAG_CAN_SEND_ERROR;
  }
}

void sendSync()
{
  sendCan(0x80, 0, NULL);
}

// Sends the control word alone with an empty motion command header (No Operation).
void sendControlWord(uint8_t low, uint8_t high)
{
  uint8_t data[8] = {low, high, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendCan(0x200 + NODE_ID, 8, data);
  sendSync();
  lastControlWord = (uint16_t)low | ((uint16_t)high << 8);
}

void operationalMode()
{
  uint8_t data[2] = {0x01, (uint8_t)NODE_ID};  // NMT start remote node
  sendCan(0x000, 2, data);
  sendSync();
}

// No SYNC: a pre-operational drive sends no PDOs, so there would be no feedback.
void preOperationalMode()
{
  uint8_t data[2] = {0x80, (uint8_t)NODE_ID};  // NMT enter pre-operational
  sendCan(0x000, 2, data);
}

void switchOn()
{
  sendControlWord(0x3F, 0x00);
}

void switchOff()
{
  sendControlWord(0x3E, 0x00);
}

// Switch on from any state: unless the drive reports operation enabled (status bit 0), clear
// control word bit 0 first, which the drive needs after power-up, an error or a switch off.
void switchOnFromAnyState()
{
  if (!(lastStatusWord & 0x0001)) {
    switchOff();
  }
  switchOn();
}

// Rising edge of control word bit 7 with bit 0 cleared; the drive stays switched off.
void acknowledgeError()
{
  sendControlWord(0xBE, 0x00);
  sendControlWord(0x3E, 0x00);
}

void home()
{
  sendControlWord(0x3F, 0x08);  // control word 0x083F: switch on + home
}

// Clears /Abort (bit 4): the drive aborts the running motion command and stays switched on.
void abortMotion()
{
  sendControlWord(0x2F, 0x00);
}

void requestStatus()
{
  sendSync();
}

// VAI go to position: control word 0x003F (switch on), motion command header 0x010x and position
// in PDO1, velocity and acceleration in PDO2, deceleration in PDO3. The drive executes a motion
// command only when its header changes, so the command count x counts up every time.
void goToPosition(long position, unsigned long velocity, unsigned long acceleration,
                  unsigned long deceleration)
{
  // After an abort (or homing, a switch-off, an error acknowledge) the drive refuses a motion
  // command that arrives together with the control word that ends that state ("Motion Cmd In
  // Wrong State", then a quick stop). Send the switch-on control word on its own first; while
  // streaming it is already 0x003F and this costs nothing.
  if (lastControlWord != 0x003F) {
    switchOn();
  }
  motionCommandCount = (motionCommandCount + 1) & 0x0F;

  uint8_t pdo1[8] = {0x3F, 0x00, motionCommandCount, 0x01, 0, 0, 0, 0};
  memcpy(&pdo1[4], &position, 4);
  uint8_t pdo2[8];
  memcpy(&pdo2[0], &velocity, 4);
  memcpy(&pdo2[4], &acceleration, 4);
  uint8_t pdo3[8] = {0, 0, 0, 0, 0x00, 0x00, 0x00, 0x00};
  memcpy(&pdo3[0], &deceleration, 4);

  // PDO1 last: it carries the new header, and the parameters must be there when it arrives.
  sendCan(0x300 + NODE_ID, 8, pdo2);
  sendCan(0x400 + NODE_ID, 8, pdo3);
  sendCan(0x200 + NODE_ID, 8, pdo1);
  sendSync();
}

// Commands

void handleCommand(const uint8_t * frame)
{
  uint8_t command = frame[2];
  long position;
  unsigned long velocity, acceleration, deceleration;
  memcpy(&position, &frame[4], 4);
  memcpy(&velocity, &frame[8], 4);
  memcpy(&acceleration, &frame[12], 4);
  memcpy(&deceleration, &frame[16], 4);

  lastSequence = frame[3];
  lastValidFrameMs = millis();

  switch (command) {
    case CMD_PING:
      sendReply(REPLY_READY, 0, 0);
      break;
    case CMD_OPERATIONAL:
      operationalMode();
      break;
    case CMD_HOME:
      home();
      break;
    case CMD_GO_TO_POS:
      if (position < POSITION_MIN || position > POSITION_MAX || velocity > VELOCITY_MAX ||
        acceleration > ACCELERATION_MAX || deceleration > ACCELERATION_MAX)
      {
        flags |= FLAG_REJECTED;
        requestStatus();  // the drive's reply carries the flag back
        break;
      }
      flags &= ~FLAG_WATCHDOG_TRIPPED;
      digitalWrite(LED_BUILTIN, LOW);
      watchdogArmed = true;
      goToPosition(position, velocity, acceleration, deceleration);
      break;
    case CMD_SWITCH_ON:
      switchOnFromAnyState();
      break;
    case CMD_SWITCH_OFF:
      watchdogArmed = false;
      switchOff();
      break;
    case CMD_REQUEST_STATUS:
      requestStatus();
      break;
    case CMD_ACK_ERROR:
      watchdogArmed = false;
      acknowledgeError();
      break;
    case CMD_PRE_OPERATIONAL:
      watchdogArmed = false;
      preOperationalMode();
      break;
    default:
      flags |= FLAG_REJECTED;
      sendReply(REPLY_READY, 0, 0);
      break;
  }
}

// Collects bytes into frames. A frame with a bad CRC is dropped and the search for the next sync
// starts again, so one lost or garbled byte costs only the frames it touches.
void readSerial()
{
  while (Serial.available() > 0) {
    uint8_t byteIn = (uint8_t)Serial.read();
    if (rxCount == 0) {
      if (byteIn == SYNC_1) {
        rxFrame[rxCount++] = byteIn;
      }
      continue;
    }
    if (rxCount == 1) {
      if (byteIn == SYNC_2) {
        rxFrame[rxCount++] = byteIn;
      } else {
        badFrames++;
        rxCount = (byteIn == SYNC_1) ? 1 : 0;
      }
      continue;
    }
    rxFrame[rxCount++] = byteIn;
    if (rxCount == COMMAND_FRAME_SIZE) {
      rxCount = 0;
      uint16_t received;
      memcpy(&received, &rxFrame[COMMAND_FRAME_SIZE - 2], 2);
      if (crc16(&rxFrame[2], COMMAND_FRAME_SIZE - 4) == received) {
        handleCommand(rxFrame);
      } else {
        badFrames++;
      }
    }
  }
}

// Forwards the drive's position and status (TPDO1, CAN ID 0x180 + node).
void readCan()
{
  while (digitalRead(CAN0_INT) == LOW) {
    unsigned long id;
    uint8_t length = 0;
    uint8_t data[8];
    if (CAN0.readMsgBuf(&id, &length, data) != CAN_OK) {
      break;
    }
    if (id == 0x180 + NODE_ID && length == 8) {
      uint16_t statusWord = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
      lastStatusWord = statusWord;
      long position = (long)((uint32_t)data[4] | ((uint32_t)data[5] << 8) |
        ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24));
      sendReply(REPLY_FEEDBACK, position, statusWord);
    }
  }
}

void checkWatchdog()
{
  if (watchdogArmed && millis() - lastValidFrameMs > WATCHDOG_MS) {
    watchdogArmed = false;
    flags |= FLAG_WATCHDOG_TRIPPED;
    digitalWrite(LED_BUILTIN, HIGH);
    abortMotion();
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(CAN0_INT, INPUT);
  Serial.begin(SERIAL_BAUD);

  // Keep retrying: without CAN there is nothing to bridge. The LED blinks meanwhile.
  while (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(250);
  }
  digitalWrite(LED_BUILTIN, LOW);
  CAN0.setMode(MCP_NORMAL);

  sendReply(REPLY_READY, 0, 0);
}

void loop()
{
  readSerial();
  readCan();
  checkWatchdog();
}
