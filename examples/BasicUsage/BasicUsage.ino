/* 

  This is the most basic way of using the XPT2046 Driver library.
  The library is initialized and then checks if a touch event occurred.
  If a touch event is captured, the touch position is printed to the Serial Monitor.
  The X and Y positions are a 12-Bit integer between 0 and 4095.
  For X and Y coordinates that are mapped to the pixel grid of a display, please look at:
  Examples > XPT2046 Driver > TouchscreenUsage
  Examples > XPT2046 Driver > 3PointCalibration

  Usage:
  1. Change the User configuration (#define) to your needs
  2. Compile and upload the sketch
  3. Touch the touch area

  Median Dispersion 2025
  https://github.com/median-dispersion/XPT2046-Driver

*/

#include "Arduino.h"
#include "XPT2046.h"

//-------------------------------------------------------------------------------------------------
// User configuration

// Define the pins the XPT2046 is connected to
// For MOSI, MISO and SCK the hardware pins are used
#define TOUCH_CS_PIN  2
#define TOUCH_IRQ_PIN 3

//-------------------------------------------------------------------------------------------------
// Global variables

// Create the XPT2046 touch object
XPT2046 touch(TOUCH_CS_PIN, TOUCH_IRQ_PIN);

// ================================================================================================
// Setup
// ================================================================================================
void setup() {

  // Initialize serial communication
  Serial.begin(115200);

  // Initialize
  touch.begin();

  // Set the rotation (optional, default = 0, values = 0, 1, 2, 3)
  // 0 = 0°, 1 = 90°, 2 = 180°, 3 = 270°
  touch.setRotation(0);

}

// ================================================================================================
// Main loop
// ================================================================================================
void loop() {

  // Check if touch area is being touched
  if (touch.touched()) {

    // Get the touch position
    XPT2046::Point position = touch.getTouchPosition();
    
    // Check if the touch position is valid
    // The touch position can become invalid if the touch event was lifted before or during the sampling process
    // This will result in a value of 65535 for both X and Y, indicating that position is invalid
    if (touch.valid(position)) {

      // Print the position to the Serial Monitor
      // The position is a 12-Bit integer between 0 and 4096
      // For X and Y coordinates that are mapped to the pixel grid of a display, please look at the other examples
      Serial.println("X: " + String(position.x) + ", Y: " + String(position.y));

    }

  }

}