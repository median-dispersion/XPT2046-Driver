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

  Median Dispersion 2024
  https://github.com/median-dispersion/XPT2046-Driver

*/

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

  // The number of samples to average over before returning the touch position (optional, default = 20, range = 1-255)
  // A higher number of samples will result in a more consistent touch position
  // But as a consequence will take a longer time to process
  touch.setSampleCount(20);

  // Set the rotation (optional, default = 0, values = 0, 1, 2, 3)
  // 0 = 0°, 1 = 90°, 2 = 180°, 3 = 270°
  touch.setRotation(0);

  // Set the debounce timeout (optional, default = 10, time = milliseconds)
  // This can help mitigate erroneous double touch events when touching or lifting from the touch area
  // Keep in mind that this is a TIMEOUT meaning if set to 1000 ms only after waiting for 1 second another touch event is registered
  // For something like a paint program, this value should be as low as possible, or even set to 0
  touch.setDebounceTimeout(10);

}

// ================================================================================================
// Main loop
// ================================================================================================
void loop() {

  // Check if touch event is occurring
  if (touch.touched()) {

    // Get the touch position
    XPT2046::Point position = touch.getTouchPosition();

    // Print the position to the serial console
    // The position is a 12-Bit integer between 0 and 4096
    // For X and Y coordinates that are mapped to the pixel grid of a display, please look at the other examples
    Serial.println("X: " + String(position.x) + ", Y: " + String(position.y));

  }

}