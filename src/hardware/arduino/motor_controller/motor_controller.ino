/*
 * motor_controller.ino
 *
 * Serial protocol for my_hardware.cpp (ROS2 hardware interface):
 *   ROS → Arduino: "left right\n"  where left/right are floats in [-1.0, 1.0]
 *   Arduino → ROS: "pos_left,pos_right,vel_left,vel_right\n"
 *                  positions in encoder counts, velocities in counts/sec
 *
 * Wiring (adjust pins to your board):
 *   Left motor:  PWM → PIN 5, DIR → PIN 4, ENC_A → PIN 2, ENC_B → PIN 3
 *   Right motor: PWM → PIN 6, DIR → PIN 7, ENC_A → PIN 8, ENC_B → PIN 9
 */

// ── Motor driver pins ─────────────────────────────────────────────────────────
#define LEFT_PWM   5
#define LEFT_DIR   4
#define RIGHT_PWM  6
#define RIGHT_DIR  7

// ── Encoder pins (interrupt-capable: 2, 3 on Uno; 2,3,18,19,20,21 on Mega) ──
#define LEFT_ENC_A  2
#define LEFT_ENC_B  3
#define RIGHT_ENC_A 18
#define RIGHT_ENC_B 19

// ── Max PWM output ────────────────────────────────────────────────────────────
#define MAX_PWM 255

// ── Encoder state ─────────────────────────────────────────────────────────────
volatile long enc_left  = 0;
volatile long enc_right = 0;

// Velocity tracking
long prev_left  = 0;
long prev_right = 0;
unsigned long prev_time = 0;
float vel_left  = 0.0;
float vel_right = 0.0;

// ── Interrupt handlers ────────────────────────────────────────────────────────
void leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B) == HIGH) enc_left++;
  else                                  enc_left--;
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) enc_right++;
  else                                   enc_right--;
}

// ── Motor drive helper ────────────────────────────────────────────────────────
void driveMotor(int pwm_pin, int dir_pin, float cmd) {
  // cmd in [-1.0, 1.0]
  int pwm = (int)(abs(cmd) * MAX_PWM);
  pwm = constrain(pwm, 0, MAX_PWM);
  digitalWrite(dir_pin, cmd >= 0 ? HIGH : LOW);
  analogWrite(pwm_pin, pwm);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(LEFT_PWM,  OUTPUT);
  pinMode(LEFT_DIR,  OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  pinMode(LEFT_ENC_A,  INPUT_PULLUP);
  pinMode(LEFT_ENC_B,  INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftEncoderISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  prev_time = millis();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // 1. Parse incoming velocity command: "left right\n"
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    int space = line.indexOf(' ');
    if (space > 0) {
      float cmd_left  = line.substring(0, space).toFloat();
      float cmd_right = line.substring(space + 1).toFloat();
      cmd_left  = constrain(cmd_left,  -1.0, 1.0);
      cmd_right = constrain(cmd_right, -1.0, 1.0);
      driveMotor(LEFT_PWM,  LEFT_DIR,  cmd_left);
      driveMotor(RIGHT_PWM, RIGHT_DIR, cmd_right);
    }
  }

  // 2. Publish encoder state at ~50 Hz
  unsigned long now = millis();
  if (now - prev_time >= 20) {
    float dt = (now - prev_time) / 1000.0;

    noInterrupts();
    long cur_left  = enc_left;
    long cur_right = enc_right;
    interrupts();

    vel_left  = (cur_left  - prev_left)  / dt;
    vel_right = (cur_right - prev_right) / dt;

    prev_left  = cur_left;
    prev_right = cur_right;
    prev_time  = now;

    // "pos_left,pos_right,vel_left,vel_right\n"
    Serial.print(cur_left);   Serial.print(',');
    Serial.print(cur_right);  Serial.print(',');
    Serial.print((long)vel_left);  Serial.print(',');
    Serial.print((long)vel_right); Serial.print('\n');
  }
}
