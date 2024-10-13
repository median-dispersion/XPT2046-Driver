#include "XPT2046.h"

// Define the pins the touchscreen is connected to
// For MOSI, MISO and SCK the hardware pins are used
#define TOUCH_CS_PIN  8
#define TOUCH_IRQ_PIN 2

// Create the XPT2046 touchscreen object
XPT2046 touchscreen(TOUCH_CS_PIN, TOUCH_IRQ_PIN);

void setup() {

  // Initialize serial communication
  Serial.begin(115200);

  // Initialize the touchscreen
  touchscreen.begin();

  // Set the amounts of samples (optional, default = 50)
  touchscreen.setSampleCount(50);

  // Set the touchscreen rotation (optional, default = 0)
  touchscreen.setRotation(0);

}

void loop() {

  // Check if the touchscreen is being touched
  if (touchscreen.touched()) {

    // Get the touch position
    Point position = touchscreen.getTouchPosition();

    // Print the position to the serial console
    Serial.println("X: " + String(position.x) + ", Y: " + String(position.y));

  }

}
