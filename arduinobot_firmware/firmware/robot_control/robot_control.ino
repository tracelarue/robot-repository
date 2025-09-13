#include <Servo.h>

// Declare the Arduino pin where each servo is connected
#define SERVO_BASE_PIN 3
#define SERVO_SHOULDER_PIN 5
#define SERVO_ELBOW_PIN 6
// ========== WRIST JOINT ADDITION START ==========
#define SERVO_WRIST_PIN 9
// ========== WRIST JOINT ADDITION END ==========
#define SERVO_GRIPPER_PIN 10

// Define the start configuration of the joints
#define BASE_START 90
#define SHOULDER_START 90
#define ELBOW_START 90
// ========== WRIST JOINT ADDITION START ==========
#define WRIST_START 90
// ========== WRIST JOINT ADDITION END ==========
#define GRIPPER_START 0

// Register the servo motors of each joint
Servo base;  
Servo shoulder;  
Servo elbow;  
// ========== WRIST JOINT ADDITION START ==========
Servo wrist;
// ========== WRIST JOINT ADDITION END ==========
Servo gripper; 

uint8_t idx = 0;
uint8_t val_idx = 0;
char value[4] = "000";


/*
 * This function moves a given servo smoothly from a given start position to a given end position.
 * The movement can be both clockwise or counterclockwise based on the values assigned to
 * the start position and end position
 */
void reach_goal(Servo& motor, int goal){
  if(goal>=motor.read()){
    // goes from the start point degrees to the end point degrees
    for (int pos = motor.read(); pos <= goal; pos += 1) { 
      motor.write(pos);     
      delay(5);                       
    }
  } else{
    // goes from the end point degrees to the start point degrees
    for (int pos = motor.read(); pos >= goal; pos -= 1) { 
      motor.write(pos);     
      delay(5);                       
    }
  }
}

void setup() {
  // Attach and Initialize each Servo to the Arduino pin where it is connected
  base.attach(SERVO_BASE_PIN);
  shoulder.attach(SERVO_SHOULDER_PIN);
  elbow.attach(SERVO_ELBOW_PIN);
  // ========== WRIST JOINT ADDITION START ==========
  wrist.attach(SERVO_WRIST_PIN);
  // ========== WRIST JOINT ADDITION END ==========
  gripper.attach(SERVO_GRIPPER_PIN); 

  // Set a common start point for each joint
  // This way, the start status of each joint is known
  base.write(BASE_START);
  shoulder.write(SHOULDER_START);
  elbow.write(ELBOW_START);
  // ========== WRIST JOINT ADDITION START ==========
  wrist.write(WRIST_START);
  // ========== WRIST JOINT ADDITION END ==========
  gripper.write(GRIPPER_START);

  // Start the Serial communication with ROS
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available())
  {
    char chr = Serial.read();

    // base motor
    if(chr == 'b')
    {
      idx = 0;
      val_idx = 0;
    }
    // shoulder motor
    else if(chr == 's')
    {
      idx = 1;
      val_idx = 0;
    }
    // elbow motor
    else if(chr == 'e')
    {
      idx = 2;
      val_idx = 0;
    }
    // ========== WRIST JOINT ADDITION START ==========
    // wrist motor
    else if(chr == 'w')
    {
      idx = 3;
      val_idx = 0;
    }
    // ========== WRIST JOINT ADDITION END ==========
    // gripper motor
    else if(chr == 'g')
    {
      idx = 4;  // ========== WRIST JOINT ADDITION: Changed from 3 to 4 ==========
      val_idx = 0;
    }
    // Separator
    else if(chr == ',')
    {
      int val = atoi(value);
      if(idx == 0)
      {
        reach_goal(base, val);
      }
      else if(idx == 1)
      {
        reach_goal(shoulder, val);
      }
      else if(idx == 2)
      {
        reach_goal(elbow, val);
      }
      // ========== WRIST JOINT ADDITION START ==========
      else if(idx == 3)
      {
        reach_goal(wrist, val);
      }
      // ========== WRIST JOINT ADDITION END ==========
      else if(idx == 4)  // ========== WRIST JOINT ADDITION: Changed from 3 to 4 ==========
      {
        reach_goal(gripper, val);
      }

      // reset the angle
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '0';
      value[4] = '\0'; // ========== WRIST JOINT ADDITION: Added to accommodate 4 digits ==========
    }
    // Plain number
    else
    {
      value[val_idx] = chr;
      val_idx++;
    }
  }
}
