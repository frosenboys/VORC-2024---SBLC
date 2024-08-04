 /*///////////////////////////////////////////////////////
------------------- DEFINE LIBRARIES --------------------
///////////////////////////////////////////////////////*/
#include <PS2X_lib.h>
#include <Adafruit_PWMServoDriver.h>
/*///////////////////////////////////////////////////////
------------------- DEFINE CONSTANTS --------------------
///////////////////////////////////////////////////////*/
#define MAX_PWM 3276
// PS2 Pins
#define PS2_DAT 12 // MISO
#define PS2_CMD 13 // MOSI
#define PS2_SEL 15 // SS
#define PS2_CLK 14 // SLK
// PS2 Modes :
#define pressures false // read analog values from buttons
#define rumble false // vibration

// Joystick Neutral Zones
#define JOY_NEUTRAL_HIGH 134
#define JOY_NEUTRAL_LOW 120

//---- Get Balls ----//
// Rulo Motor
#define MOTOR_CHANNEL_A1 8
#define MOTOR_CHANNEL_A2 9

//---- Drop Balls ----//
// Linear Slide Motor
#define MOTOR_CHANNEL_B1 10
#define MOTOR_CHANNEL_B2 11
// Gate Servo
#define SERVO_1 2
#define SERVO_2 3

//---- Wheels ----//
// Left Wheel Motor
#define MOTOR_CHANNEL_C1 13
#define MOTOR_CHANNEL_C2 12
// Right Wheel Motor
#define MOTOR_CHANNEL_D1 14
#define MOTOR_CHANNEL_D2 15


/*///////////////////////////////////////////////////////
------------------- DEFINE VARIABLES --------------------
///////////////////////////////////////////////////////*/
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Default I2C PCA9685 address: 0x40
int motorC1_speed, motorC2_speed, motorD1_speed, motorD2_speed; // Wheel's Motors speed
int getState = 0, dropState = 0; // States to switching modes
PS2X ps2x; // Init PS2x class
int ps2Error = -1, JoyRight, JoyLeft; // PS2

/*///////////////////////////////////////////////////////
----------------- MOVING MOTORS MODULE ------------------
///////////////////////////////////////////////////////*/
void movingMotors(int motorC1_speed, int motorC2_speed, int motorD1_speed, int motorD2_speed)
{
  // Pulse pwm for motor C (Left wheel)
  pwm.setPWM(MOTOR_CHANNEL_C1, 0, motorC1_speed); 
  pwm.setPWM(MOTOR_CHANNEL_C2, 0, motorC2_speed);

  // Pulse pwm for motor D (Right wheel)
  pwm.setPWM(MOTOR_CHANNEL_D1, 0, motorD1_speed); 
  pwm.setPWM(MOTOR_CHANNEL_D2, 0, motorD2_speed);
}

/*///////////////////////////////////////////////////////
----------------- GET BALLS MODULE ------------------
///////////////////////////////////////////////////////*/
// ---- Main Get Ball Function (Roll the rulo) ---- //
void GetBall(){
  // Pulse pwm for motor A (Rulo)
  if(getState) pwm.setPWM(MOTOR_CHANNEL_A1, 0, MAX_PWM);
  else pwm.setPWM(MOTOR_CHANNEL_A1, 0, 0);
}

/*///////////////////////////////////////////////////////
----------------- DROP BALLS MODULE ------------------
///////////////////////////////////////////////////////*/
// ---- Set Servo Degree ---- //
void WriteDeg(int pin,int deg){
  int pulse = map(deg, 0, 180, 90, 600);
  pwm.setPWM(pin, 0, pulse);
}

// ---- Main Drop Ball Function ---- //
void DropBall(){
  if(dropState){
    // Gate's Servo control
    if (ps2x.ButtonPressed(PSB_PINK)){
      // Open Gate (100°)
      WriteDeg(SERVO_1, 100);
      WriteDeg(SERVO_2, 100);
    }
    else if (ps2x.ButtonPressed(PSB_RED)){
      // Close Gate (0°)
      WriteDeg(SERVO_1, 0);
      WriteDeg(SERVO_2, 0);
    }

    // Linear Slide control
    if (ps2x.ButtonPressed(PSB_PAD_UP)){
      // Lift up
      pwm.setPWM(MOTOR_CHANNEL_B1, 0, MAX_PWM/2); 
      pwm.setPWM(MOTOR_CHANNEL_B2, 0, 0);
    }
    else if (ps2x.ButtonPressed(PSB_PAD_DOWN)){
      // Lift down
      pwm.setPWM(MOTOR_CHANNEL_B1, 0, 0); 
      pwm.setPWM(MOTOR_CHANNEL_B2, 0, MAX_PWM/2);
    }
  }
}

/*///////////////////////////////////////////////////////
----------------- PS3 CONTROLLER MODULE -----------------
///////////////////////////////////////////////////////*/
void PS2_Controller(){  
  //----Moving Wheels----//
  // Left wheel
  JoyLeft = ps2x.Analog(PSS_LY);
  if(JoyLeft > JOY_NEUTRAL_HIGH) {
    // Forward
    motorC1_speed = map(JoyLeft, JOY_NEUTRAL_HIGH, 255, 0, MAX_PWM);
    motorC2_speed = 0;  
  }
  if(JoyLeft < JOY_NEUTRAL_LOW) {
    // Backward
    motorC1_speed = 0;
    motorC2_speed = map(JoyLeft, JOY_NEUTRAL_LOW, 0, 0, MAX_PWM);
  }
  if(JoyLeft >= JOY_NEUTRAL_LOW && JoyLeft <= JOY_NEUTRAL_HIGH) {
    // Stop
    motorC1_speed = 0;
    motorC2_speed = 0;
  }

  // Right wheel
  JoyRight = ps2x.Analog(PSS_RY);
  if(JoyRight > JOY_NEUTRAL_HIGH) {
    // Forward
    motorD1_speed = 0;
    motorD2_speed = map(JoyRight, JOY_NEUTRAL_HIGH, 255, 0, MAX_PWM);
  }
  if(JoyRight < JOY_NEUTRAL_LOW) {
    // Backward
    motorD1_speed = map(JoyRight, JOY_NEUTRAL_LOW, 0, 0, MAX_PWM);
    motorD2_speed = 0;
  }
  if(JoyRight >= JOY_NEUTRAL_LOW && JoyRight <= JOY_NEUTRAL_HIGH) {
    // Stop
    motorD1_speed = 0;
    motorD2_speed = 0;
  }
  // Pulse PWM to motors
  movingMotors(motorC1_speed, motorC2_speed, motorD1_speed, motorD2_speed);


  //---- Change States----//
  // Get balls
  if (ps2x.ButtonPressed(PSB_L1)){
    getState = !getState;
    GetBall();
  }

  // Drop ball
  if (ps2x.ButtonPressed(PSB_R1)){
    dropState = !dropState;
    DropBall();
  }
}
/*///////////////////////////////////////////////////////
------------------- CONFIGURE MODULES -------------------
///////////////////////////////////////////////////////*/
void setup() {
  // ---- Setup PCA9685 ---- //
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);
  movingMotors(0,0,0,0);
  // ---- Connect to PS2 ---- //
  while (ps2Error != 0){
    delay(1000);
    ps2Error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  }
}

void loop() {
  ps2x.read_gamepad(pressures, rumble);
  PS2_Controller();
}