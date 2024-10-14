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
  
  Median Dispersion 2024
  https://github.com/median-dispersion/XPT2046-Driver

*/

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
// If the display is rotated by 90° or 270°, switch the width and height value
#define DISPLAY_WIDTH  240
#define DISPLAY_HEIGHT 320

// Define display rotation
// 0 = 0°, 1 = 90°, 2 = 180°, 3 = 270°
#define DISPLAY_ROTATION 0

// Touchscreen calibration
// These values most likely will work for any display, but might not be accurate
// To get an accurate calibration matrix a 3 point calibration needs to be performed
// To perform a 3 point calibration, use the sketch: Examples > XPT2046 Driver > 3PointCalibration
// Change the matrix coefficients according to that result
// Keep in mind when changing the screen rotation a need calibration matrix is also required
XPT2046::CalibrationMatrix CALIBRATION = {
  
  /* A */ 0.0654288, 
  /* B */ 0.0000242, 
  /* C */ -12.5270405, 
  /* D */ -0.0008066, 
  /* E */ 0.0913295, 
  /* F */ -17.0230808, 
  /* W */ DISPLAY_WIDTH, // This must be the display's width in pixels
  /* H */ DISPLAY_HEIGHT // This must be the display's height in pixels
  
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

// Create frame buffer object
GFXcanvas16 canvas(DISPLAY_WIDTH, DISPLAY_HEIGHT);

// ================================================================================================
// Setup
// ================================================================================================
void setup() {

  // Initialize serial communication
  Serial.begin(115200);

  // Initialize touchscreen
  touchscreen.begin();

  // The number of samples to average over before returning the touch position (optional, default = 20, range = 1-255)
  // A higher number of samples will result in a more consistent touch position
  // But as a consequence will take a longer time to process
  touchscreen.setSampleCount(20);

  // Set the touchscreen rotation (optional, default = 0, values = 0, 1, 2, 3)
  // 0 = 0°, 1 = 90°, 2 = 180°, 3 = 270°
  touchscreen.setRotation(DISPLAY_ROTATION);
  
  // Set the display calibration (optional, default = unset)
  // This will map the touch position to the display's pixel grid
  // i.e., touchX = pixelX and touchY = pixelY
  // To get the calibration matrix, use the sketch: Examples > XPT2046 Driver > 3PointCalibration
  touchscreen.setCalibration(CALIBRATION);

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

}

// ================================================================================================
// Main loop
// ================================================================================================
void loop() {

  // If the touchscreen is being touched
  if (touchscreen.touched()) {

    // Get the touch position
    XPT2046::Point position = touchscreen.getTouchPosition();

    // Get the touch position
    canvas.fillScreen(ILI9341_BLACK);

    // Create a string containing the current touch position
    String positionString = "X: " + String(position.x) + "\nY:" + String(position.y);

    // Set text color and position
    canvas.setTextColor(ILI9341_WHITE);
    canvas.setCursor(0, 0);

    // Draw position string to the frame buffer
    canvas.print(positionString);

    // Draw crosshair to the frame buffer
    canvas.drawLine(0, position.y, DISPLAY_WIDTH, position.y, ILI9341_RED);
    canvas.drawLine(position.x, 0, position.x, DISPLAY_HEIGHT, ILI9341_RED);
    canvas.drawCircle(position.x, position.y, 5, ILI9341_RED);

    // Write frame buffer to the display
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());

  }

}