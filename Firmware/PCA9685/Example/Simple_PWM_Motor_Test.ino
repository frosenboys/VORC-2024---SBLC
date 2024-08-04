// --- Include libraries ---
#include <Adafruit_PWMServoDriver.h>

// --- Define pulse value variables ---
#define PWM_Pulse 4095                                      // 100% pulse

// --- Define frequency variables ---
#define MOTOR_FREQ 50

// --- Define pulse pins for channels ---
#define MOTOR_CHANNEL_A1 8          
#define MOTOR_CHANNEL_A2 9
#define MOTOR_CHANNEL_B1 10
#define MOTOR_CHANNEL_B2 11
#define MOTOR_CHANNEL_C1 12
#define MOTOR_CHANNEL_C2 13
#define MOTOR_CHANNEL_D1 14
#define MOTOR_CHANNEL_D2 15

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();   //  Default I2C address for PCA9685: 0x40

// --- Function to initialize, set up, and define tasks and variables ---
void setup()
{
    // --- Initialize Serial (Default baud rate for ESP32: 115200) ---
    Serial.begin(115200);

    // --- Initialize I2C PWM ---
    pwm.begin();
    // --- Set the oscillator frequency of the PCA9685 chip: 23MHz -> 27MHz ---
    pwm.setOscillatorFrequency(27000000);
    // --- Set the PWM frequency (Default: 50Hz) ---
    pwm.setPWMFreq(MOTOR_FREQ);
}

// --- Function to execute commands continuously in a loop ---
void loop()
{
    // --- Commands to test motors on each channel (Uncomment the line to use: Highlight and press CTRL + /) ---
    
    // Pulse on Channel A1 with 100% speed
    // pwm.setPWM(MOTOR_CHANNEL_A1, 0, PWM_Pulse);     // Uncomment here to use

    // Pulse on Channel A2 with 100% speed
    // pwm.setPWM(MOTOR_CHANNEL_A2, 0, PWM_Pulse);     // Uncomment here to use

    // Pulse on Channel B1 with 100% speed
    // pwm.setPWM(MOTOR_CHANNEL_B1, 0, PWM_Pulse);     // Uncomment here to use

    // Pulse on Channel B2 with 100% speed
    // pwm.setPWM(MOTOR_CHANNEL_B2, 0, PWM_Pulse);     // Uncomment here to use

    // Pulse on Channel C1 with 100% speed
    // pwm.setPWM(MOTOR_CHANNEL_C1, 0, PWM_Pulse);     // Uncomment here to use

    // Pulse on Channel C2 with 100% speed
    // pwm.setPWM(MOTOR_CHANNEL_C2, 0, PWM_Pulse);     // Uncomment here to use

    // Pulse on Channel D1 with 100% speed
    // pwm.setPWM(MOTOR_CHANNEL_D1, 0, PWM_Pulse);     // Uncomment here to use

    // Pulse on Channel D2 with 100% speed
    // pwm.setPWM(MOTOR_CHANNEL_D2, 0, PWM_Pulse);     // Uncomment here to use

    // --- Delay (in ms)
    delay(10000);
}
