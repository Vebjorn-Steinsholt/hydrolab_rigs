// HC-SR04 ultrasonic wave probe: echo times over USB serial
//
// The RPi4 reads the lines and publishes sensor_msgs/Range. This sketch only measures: the
// conversion to metres (speed of sound from the air temperature), the min/max limits and the
// timestamps are done on the RPi4, so they change without reflashing.
//
// Based on the HC-SR04 example by Isaac100 (Arduino Project Hub).
//
// Serial link: 115200 baud, 8N1, one ASCII line per ping:
//
//   seq,t_us,echo_us\n
//
//   seq      ping number since boot, +1 per line; a gap means lines were lost
//   t_us     micros() at the trigger; wraps after about 71 minutes
//   echo_us  echo pulse length in microseconds; 0 = no echo within TIMEOUT_US
//
// Range on the RPi4: range_m = echo_us * 1e-6 * c / 2, with c = 331.3 + 0.606 * T_air_C m/s.
// Opening the port resets most Arduinos: the reader skips lines until one parses.

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

const unsigned long SERIAL_BAUD = 115200;
const unsigned long PERIOD_US = 50000;    // 20 Hz; the datasheet recommends at least 60 ms
const unsigned long TIMEOUT_US = 30000;   // longest echo waited for: about 5 m round trip

unsigned long next_us = 0;
unsigned long seq = 0;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(SERIAL_BAUD);
  next_us = micros();
}

void loop() {
  unsigned long now = micros();
  if ((long)(now - next_us) < 0) {
    return;                               // not time for the next ping yet
  }
  next_us += PERIOD_US;
  if ((long)(now - next_us) >= 0) {
    next_us = now + PERIOD_US;            // fell a whole period behind: restart the schedule
  }

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  unsigned long t_us = micros();
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long echo_us = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);   // 0 on timeout

  Serial.print(seq++);
  Serial.print(',');
  Serial.print(t_us);
  Serial.print(',');
  Serial.println(echo_us);
}