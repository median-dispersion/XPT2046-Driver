/*

  This sketch is an example of how to use the XPT2046 Driver library with a touchscreen.
  The touchscreen used in this case is an ILI9341 TFT display driven by the Adafruit GFX & ILI9341 library.
  It maps the touch X and Y position to the pixel grid of the ILI9341 TFT using a calibration matrix.
  This calibration matrix is unique for every display and needs to be determent by performing a 3 point calibration.
  To perform a 3 point calibration, use the sketch: Examples > XPT2046 Driver > 3PointCalibration.

  This sketch initializes the touchscreen and display and then checks if a touch event occurred.
  It takes the touch position mapped to the pixel grid of the display and draws a crosshair at that point.
  It also draws the pixel X and Y coordinates on screen.

  Usage:
  1. Change the User configuration (#define) to your needs
  2. Compile and upload the sketch
  3. Touch the touchscreen
  
  Median Dispersion 2025
  https://github.com/median-dispersion/XPT2046-Driver

*/

#include "Arduino.h"
#include "XPT2046.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

//-------------------------------------------------------------------------------------------------
// User configuration

// Define the pins the touchscreen is connected to
// For MOSI, MISO and SCK the hardware pins are used
#define TOUCH_CS_PIN  2
#define TOUCH_IRQ_PIN 3

// Define the pins the display is connected to
// For MOSI, MISO and SCK the hardware pins are used
#define DISPLAY_DC_PIN  9
#define DISPLAY_CS_PIN  10
#define DISPLAY_RST_PIN 11 // Some displays might not have this pin, -1 disables this option
#define DISPLAY_LED_PIN 12 // Some displays might not have this pin, -1 disables this option

// Define the display's width and height in px
// If the display is rotated by 90° or 270°, these values will automatically be swapped
#define DISPLAY_WIDTH  240
#define DISPLAY_HEIGHT 320

// Define display rotation
// 0 = 0°, 1 = 90°, 2 = 180°, 3 = 270°
#define DISPLAY_ROTATION 0

// Touchscreen calibration
// These values most likely will work for any display, but might not be accurate
// To get an accurate calibration matrix, a 3-point calibration needs to be performed
// To perform a 3-point calibration, use the sketch: Examples > XPT2046 Driver > 3PointCalibration
// Change the matrix coefficients according to that result
XPT2046::Calibration CALIBRATION = {
  
  /* A */ 0.0644197, 
  /* B */ -0.0003023, 
  /* C */ -11.8842955, 
  /* D */ -0.0002274, 
  /* E */ 0.0898521, 
  /* F */ -14.7727499, 
  /* W */ 240,
  /* H */ 320,
  /* R */ 0
  
};

//-------------------------------------------------------------------------------------------------
// Global variables

// Create touchscreen object
XPT2046 touchscreen(TOUCH_CS_PIN, TOUCH_IRQ_PIN);

// Create display object depending on if the display has the DISPLAY_RST_PIN
#if DISPLAY_RST_PIN == -1
  Adafruit_ILI9341 display(DISPLAY_CS_PIN, DISPLAY_DC_PIN);
#else
  Adafruit_ILI9341 display(DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN);
#endif

// Depending on screen rotation, swap width and height
#if (DISPLAY_ROTATION == 0) || (DISPLAY_ROTATION == 2)

  const uint16_t displayWidth  = DISPLAY_WIDTH;
  const uint16_t displayHeight = DISPLAY_HEIGHT;

#elif (DISPLAY_ROTATION == 1) || (DISPLAY_ROTATION == 3)

  const uint16_t displayWidth  = DISPLAY_HEIGHT;
  const uint16_t displayHeight = DISPLAY_WIDTH;

#endif

// Create frame buffer object
GFXcanvas16 canvas(displayWidth, displayHeight);

// ================================================================================================
// Draw the touch position on screen
// ================================================================================================
void drawTouchPosition(XPT2046::Point position) {

  // Create a position string
  String positionString = "X: " + String(position.x) + "\n" +
                          "Y: " + String(position.y);

  // Clear the frame buffer
  canvas.fillScreen(ILI9341_BLACK);

  // Set the text color and cursor
  canvas.setTextColor(ILI9341_WHITE);
  canvas.setCursor(0, 0);

  // Draw the position string to the frame buffer
  canvas.print(positionString);

  // Draw a crosshair at the touch position
  canvas.drawLine(0, position.y, displayWidth, position.y, ILI9341_RED);
  canvas.drawLine(position.x, 0, position.x, displayHeight, ILI9341_RED);
  canvas.drawCircle(position.x, position.y, 5, ILI9341_RED);

  // Write frame buffer to the display
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());

}

// ================================================================================================
// Setup
// ================================================================================================
void setup() {

  // Initialize serial communication
  Serial.begin(115200);

  // Initialize touchscreen
  touchscreen.begin();

  // Set the touchscreen rotation (optional, default = 0, values = 0, 1, 2, 3)
  // 0 = 0°, 1 = 90°, 2 = 180°, 3 = 270°
  // On some screens, one or both axes might be flipped, this depends on how that specific touchscreen was manufactured
  // The flipped axes should be fixed when using the calibration matrix generated by performing a 3-point calibration
  touchscreen.setRotation(DISPLAY_ROTATION);
  
  // Set the display calibration (optional, default = unset)
  // This will map the touch position to the display's pixel grid
  // i.e., touchX = pixelX and touchY = pixelY
  // To get the calibration matrix, use the sketch: Examples > XPT2046 Driver > 3PointCalibration
  touchscreen.setCalibration(CALIBRATION);

  // The number of samples to average over before returning the touch position (optional, default = 20, range = 1-255)
  // A higher number of samples will result in a more consistent touch position
  // But as a consequence, it will take a longer time to process
  touchscreen.setSampleCount(20);

  // Set the debounce timeout (optional, default = 100, time = milliseconds)
  // This can help mitigate erroneous double touch events when lifting from the touch area
  // Keep in mind that this is a TIMEOUT meaning if set to 1000 ms only after waiting for 1 second another touch event is registered
  // For something like a paint program, this value should be as low as possible, or even set to 0
  touchscreen.setDebounceTimeout(100);

  // Set the pressure at which a touch event will be registered (optional, default = 3.5)
  // A lower value means higher pressure needs to be applied before counting as a touch event
  // Too high of a value might make touch readings unstable because there is not enough pressure applied to make them settle down
  touchscreen.setTouchPressure(3.5);

  // Set the dead zone (optional, default = 50, range = 0-2047)
  // All touch values below this value will be disregarded as invalid
  // Most touchscreens have a dead zone of around 200, so disregarding all values below that should be fine
  touchscreen.setDeadZone(50);

  // Set the power-down state (optional, default = true, values = true, false)
  // Enabling this will send an extra command to the XPT2046 to power down its reference and ADCs after each SPI transaction
  // Reducing overall power draw but sending an extra 8 bits over the SPI bus
  touchscreen.setPowerDown(true);

  // Initialize the display
  display.begin();

  // Set the display rotation
  display.setRotation(DISPLAY_ROTATION);

  // Clear the screen
  display.fillScreen(ILI9341_BLACK);

  // Only do this if the display has the DISPLAY_LED_PIN
  #if DISPLAY_LED_PIN != -1

    // Set the display LED pin mode to output
    pinMode(DISPLAY_LED_PIN, OUTPUT);

    // Turn on the display LED
    digitalWrite(DISPLAY_LED_PIN, HIGH);

  #endif

  // Draw a crosshair at the center of the screen
  drawTouchPosition({displayWidth / 2, displayHeight / 2});

}

// ================================================================================================
// Main loop
// ================================================================================================
void loop() {

  // If the touchscreen is being touched
  if (touchscreen.touched()) {

    // An alternative test case could be:
    // if (touchscreen.touched() && touchscreen.released()) {}
    // This would check if the touchscreen is currently being touched
    // But only continue if the previous touch event was released, i.e., the pen or finger was lifted from the touchscreen
    // This can be useful for single button presses

    // Get the touch position
    XPT2046::Point position = touchscreen.getTouchPosition();

    // Check if the touch position is valid
    // The touch position can become invalid if the touch event was lifted before or during the sampling process
    // This will result in a value of 65535 for both X and Y, indicating that position is invalid
    if (touchscreen.valid(position)) {

      // Draw the touch position on screen
      drawTouchPosition(position);

    }

  }

}