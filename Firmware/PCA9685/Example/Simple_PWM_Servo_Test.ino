// --- Include libraries ---
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// --- Define and initialize variables ---
#define SERVOMIN  150   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600   // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600      // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400     // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50   // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;   // Variable to keep track of the servo number

void setup()
{
    // --- Initialize Serial (Default baud rate for ESP32: 115200) ---
    Serial.begin(115200);

    // --- Initialize I2C PWM ---
    pwm.begin();
    // --- Set the oscillator frequency of the PCA9685 chip: 23MHz -> 27MHz ---
    pwm.setOscillatorFrequency(27000000);
    // --- Set the PWM frequency (Default: 50Hz) ---
    pwm.setPWMFreq(SERVO_FREQ);

    // --- Print status if the above commands have been executed successfully ---
    Serial.println("Servo test program for 8 channels");

    // --- Delay (in ms) ---
    delay(10);
}

// --- Function to set the speed / direction of the servo (not time-based) ---
void servoSetPulseTesting()
{
    // Loop to increase pulse -> Servo will pulse with corresponding value
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
        pwm.setPWM(servonum, 0, pulselen);      
    }

    // --- Delay (in ms) ---
    delay(500);

    // Loop to decrease pulse -> Servo will pulse with corresponding value
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
        pwm.setPWM(servonum, 0, pulselen);      
    }

    // --- Delay (in ms) ---
    delay(500);
}

// --- Function to set the rotation cycle of the servo (time-based - unit: microseconds) ---
void servoWriteUsTesting()
{
    // Loop to increase time (unit: microseconds) -> Servo will pulse based on corresponding time
    for (uint16_t microsec = USMIN; microsec < USMAX; microsec++) {
        pwm.writeMicroseconds(servonum, microsec);
    }

    // --- Delay (in ms) ---
    delay(500);

    // Loop to decrease time (unit: microseconds) -> Servo will pulse based on corresponding time
    for (uint16_t microsec = USMAX; microsec > USMIN; microsec--) {
        pwm.writeMicroseconds(servonum, microsec);
    }

    // --- Delay (in ms) ---
    delay(500);
}

void loop()
{
    // --- Print the servo number to the screen ---
    Serial.println(servonum);
    // --- Test functions ---
    servoSetPulseTesting();
    servoWriteUsTesting();

    // --- Increment the servo number after executing the above commands ---
    servonum++;
    // --- Limit to 8 servos ---
    if (servonum > 7) servonum = 0;
}
