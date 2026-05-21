// Arduino code for ROS2 robot control
// This code receives velocity commands from ROS and sends back encoder data

// Motor pins (adjust according to your setup)
#define LEFT_MOTOR_PWM 5
#define LEFT_MOTOR_DIR 4
#define RIGHT_MOTOR_PWM 6
#define RIGHT_MOTOR_DIR 7

// Encoder pins (adjust according to your setup)
#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 3
#define RIGHT_ENCODER_A 8
#define RIGHT_ENCODER_B 9

// Variables
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;
float left_velocity = 0.0;
float right_velocity = 0.0;

// Encoder interrupt handlers
void leftEncoderISR() {
  if (digitalRead(LEFT_ENCODER_B) == HIGH) {
    left_encoder_count++;
  } else {
    left_encoder_count--;
  }
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) {
    right_encoder_count++;
  } else {
    right_encoder_count--;
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  // Motor pins
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);

  // Encoder pins
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);

  Serial.println("Arduino ready for ROS communication");
}

void loop() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Parse command (format: "left_vel,right_vel")
    int commaIndex = command.indexOf(',');
    if (commaIndex > 0) {
      float left_cmd = command.substring(0, commaIndex).toFloat();
      float right_cmd = command.substring(commaIndex + 1).toFloat();

      // Set motor velocities
      setMotorVelocity(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR, left_cmd);
      setMotorVelocity(RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR, right_cmd);

      // Log received commands
      Serial.print("Received commands - Left: ");
      Serial.print(left_cmd);
      Serial.print(", Right: ");
      Serial.println(right_cmd);
    }
  }

  // Send encoder data periodically
  static unsigned long last_send = 0;
  if (millis() - last_send > 100) {  // Send every 100ms
    // Calculate velocities (pulses per second)
    static long last_left_count = 0;
    static long last_right_count = 0;
    static unsigned long last_time = 0;

    unsigned long current_time = millis();
    long time_diff = current_time - last_time;

    if (time_diff > 0) {
      left_velocity = (left_encoder_count - last_left_count) * 1000.0 / time_diff;
      right_velocity = (right_encoder_count - last_right_count) * 1000.0 / time_diff;

      last_left_count = left_encoder_count;
      last_right_count = right_encoder_count;
      last_time = current_time;
    }

    // Send data in format: "pos_left,pos_right,vel_left,vel_right"
    Serial.print(left_encoder_count);
    Serial.print(",");
    Serial.print(right_encoder_count);
    Serial.print(",");
    Serial.print(left_velocity);
    Serial.print(",");
    Serial.println(right_velocity);

    last_send = millis();
  }
}

void setMotorVelocity(int pwm_pin, int dir_pin, float velocity) {
  // Convert velocity to PWM (adjust scaling as needed)
  int pwm_value = abs(velocity) * 255;  // Assuming velocity range -1 to 1
  pwm_value = constrain(pwm_value, 0, 255);

  // Set direction
  if (velocity >= 0) {
    digitalWrite(dir_pin, HIGH);
  } else {
    digitalWrite(dir_pin, LOW);
  }

  // Set PWM
  analogWrite(pwm_pin, pwm_value);
}