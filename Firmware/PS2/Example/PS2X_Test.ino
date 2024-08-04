#include <PS2X_lib.h>
/******************************************************************
 * Pin setup for the library:
 * - On the Motor Shield, there is a 6-pin header
 *   designed for the PS2 controller.
 * Header pinout and corresponding GPIO layout:
 *   MOSI | MISO | GND | 3.3V | CS | CLK
 *    12     13    GND   3.3V   15   14
 ******************************************************************/

#define PS2_DAT 12 // MISO
#define PS2_CMD 13 // MOSI
#define PS2_SEL 15 // SS
#define PS2_CLK 14 // CLK

/******************************************************************
 * Choose mode for the PS2 controller:
 *   - pressures = read analog values from buttons
 *   - rumble    = enable/disable vibration mode
 ******************************************************************/
#define pressures false
#define rumble false

PS2X ps2x; // create PS2x class

void setup()
{
  Serial.begin(115200);
  Serial.print("Connecting to PS2 controller:");

  int error = -1;
  for (int i = 0; i < 10; i++) // try to connect to the PS2 controller 10 times
  {
    delay(1000); // wait for 1 second
    // setup pins and modes: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
  }

  switch (error) // check for errors if unable to connect after 10 attempts
  {
  case 0:
    Serial.println(" Successfully connected to PS2 controller");
    break;
  case 1:
    Serial.println(" ERROR: No controller found, check connection");
    break;
  case 2:
    Serial.println(" ERROR: Command failed");
    break;
  case 3:
    Serial.println(" ERROR: Failed to enter pressures mode");
    break;
  }
}

void loop()
{
  ps2x.read_gamepad(false, false); // call function to read the controller

  // returns TRUE (1) when the button is held
  if (ps2x.Button(PSB_START)) // if Start button is held, print to Serial monitor
    Serial.println("Start is being held");
  if (ps2x.Button(PSB_SELECT)) // if Select button is held, print to Serial monitor
    Serial.println("Select is being held");

  if (ps2x.Button(PSB_PAD_UP)) // similarly check the Up button (PAD UP)
  {
    Serial.print("Up held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC); // read analog value from this button, see how hard the button is pressed
  }
  if (ps2x.Button(PSB_PAD_RIGHT))
  {
    Serial.print("Right held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
  }
  if (ps2x.Button(PSB_PAD_LEFT))
  {
    Serial.print("LEFT held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
  }
  if (ps2x.Button(PSB_PAD_DOWN))
  {
    Serial.print("DOWN held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
  }

  if (ps2x.NewButtonState())
  { // Returns TRUE when a button changes state (from on to off, or off to on)
    if (ps2x.Button(PSB_L3))
      Serial.println("L3 pressed");
    if (ps2x.Button(PSB_R3))
      Serial.println("R3 pressed");
    if (ps2x.Button(PSB_L2))
      Serial.println("L2 pressed");
    if (ps2x.Button(PSB_R2))
      Serial.println("R2 pressed");
    if (ps2x.Button(PSB_TRIANGLE))
      Serial.println("△ pressed");
  }
  //△□○×
  if (ps2x.ButtonPressed(PSB_CIRCLE)) // Returns TRUE when the button is pressed (from off to on)
    Serial.println("○ just pressed");
  if (ps2x.NewButtonState(PSB_CROSS)) // Returns TRUE when the button changes state
    Serial.println("× just changed");
  if (ps2x.ButtonReleased(PSB_SQUARE)) // Returns TRUE when the button is released (from on to off)
    Serial.println("□ just released");

  if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) // returns TRUE when the button is held
  {                                               // Read the joystick values when L1 or R1 is held
    Serial.print("Stick Values:");
    Serial.print(ps2x.Analog(PSS_LY)); // read Y-axis of left joystick. Other options: LX, RY, RX
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_LX), DEC);
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_RY), DEC);
    Serial.print(",");
    Serial.println(ps2x.Analog(PSS_RX), DEC);
  }
  delay(50);
}
