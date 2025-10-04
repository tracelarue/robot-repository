const unsigned long SERIAL_TIMEOUT = 50; // 50ms timeout

// Define pins for motor control
const int LEFT_MOTOR_PWM_PIN = 5;     // PWM pin for left motor speed
const int RIGHT_MOTOR_PWM_PIN = 6;    // PWM pin for right motor speed
const int LEFT_MOTOR_DIR_PIN = 12;    // Direction pin for left motor
const int RIGHT_MOTOR_DIR_PIN = 7;    // Direction pin for right motor
const int MOTOR_ENABLE_PIN = 8;       // Enable pin for both motors
const int LEFT_ENCODER_PIN = 2;       // Left motor encoder pin
const int RIGHT_ENCODER_PIN = 4;      // Right motor encoder pin

float left_motor_speed, right_motor_speed;
int left_motor_dir, right_motor_dir;

unsigned long lastCommandTime = 0;

long leftCount = 0;
long rightCount = 0;


void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  Serial.setTimeout(SERIAL_TIMEOUT);

  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);

  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  
  // Wait for the serial port to be ready (optional, depending on your board)
  while (!Serial) { }

  Serial.println("Arduino Ready");
}

void loop() {
  // Check if data is available on the Serial port
  if (Serial.available() >= 4) {
    left_motor_speed = Serial.parseFloat();
    right_motor_speed = Serial.parseFloat();
    left_motor_dir = Serial.parseInt();
    right_motor_dir = Serial.parseInt();

    // Clear any remaining data in buffer
    while (Serial.available() > 0) {
      Serial.read();
    }

    // Set motor speed and direction
    analogWrite(LEFT_MOTOR_PWM_PIN, abs(left_motor_speed));
    analogWrite(RIGHT_MOTOR_PWM_PIN, abs(right_motor_speed));
    digitalWrite(LEFT_MOTOR_DIR_PIN, left_motor_dir);
    digitalWrite(RIGHT_MOTOR_DIR_PIN, right_motor_dir);

    lastCommandTime = millis();
    

  }
  
  // Safety feature: stop motors if no commands received for 1 second
  if (millis() - lastCommandTime > 1000) {
    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
  }
}
