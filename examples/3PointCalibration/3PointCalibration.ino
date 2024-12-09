/*

  This sketch performs a 3 point calibration on a touchscreen that uses the XPT2046 controller.
  The touchscreen used in this case is an ILI9341 TFT display driven by the Adafruit GFX & ILI9341 library.
  The resulting calibration matrix can then be used in other sketches to calibrate and map the touch positions to the display's pixel grid.

  Usage:
  1. Change the User configuration (#define) to your needs
  2. Compile and upload the sketch
  3. Tap the touch targets
  4. Copy the calculated calibration matrix from the Serial Monitor (115200 Baud)
  5. Check the calibration with the crosshair on screen

  For an example of how to use the resulting calibration matrix in your sketch, look at:
  Examples > XPT2046 Driver > TouchscreenUsage

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

// Define how many calibration samples should be taken before averaging them
// i.e., how often to touch the 3 touch targets before calculating the calibration matrix
#define CALIBRATION_SAMPLES 3

//-------------------------------------------------------------------------------------------------
// Global variables

// Create touchscreen object
XPT2046 touchscreen(TOUCH_CS_PIN, TOUCH_IRQ_PIN);

// Create display object depending on whether the display has the DISPLAY_RST_PIN
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

  // Set the number of samples to the maximum value to get the most accurate reading
  touchscreen.setSampleCount(255);

  // Set the touchscreen rotation
  touchscreen.setRotation(DISPLAY_ROTATION);

  // Set the debounce timeout to a fairly high value to negate erroneous double touch events
  touchscreen.setDebounceTimeout(250);

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

  // Calibrate the touchscreen
  calibrateTouchscreen();

  // Rest debounce timeout to make the touchscreen more responsive
  touchscreen.setDebounceTimeout(0);

  // Draw the crosshair in the center of the screen
  drawTouchPosition({DISPLAY_WIDTH / 2, DISPLAY_HEIGHT / 2});

}

// ================================================================================================
// Main loop
// ================================================================================================
void loop() {

  // Check if the touchscreen is being touched
  if (touchscreen.touched()) {

    // Get the touch position
    XPT2046::Point position = touchscreen.getTouchPosition();

    // Draw touch position on screen
    drawTouchPosition(position);

  }

}

// ================================================================================================
// Calibrate the touchscreen
// ================================================================================================
void calibrateTouchscreen() {

  // Array of touch targets
  XPT2046::Point targets[3] = {

    {DISPLAY_WIDTH * 0.43, DISPLAY_HEIGHT * 0.05},
    {DISPLAY_WIDTH * 0.94, DISPLAY_HEIGHT * 0.54},
    {DISPLAY_WIDTH * 0.04, DISPLAY_HEIGHT * 0.96}

  };
  
  // Array of touch measurements
  XPT2046::Point measurements[3] = {

    0,
    0,
    0

  };

  // Calibration matrix
  XPT2046::Calibration calibration;

  // Measure the touch positions of the touch targets
  measureTouchTargets(targets, measurements);

  // Calculate the calibration matrix
  calculateCalibrationMatrix(targets, measurements, calibration);

  // Print the calibration matrix result to the Serial Monitor
  printCalibrationMatrix(calibration);
  
  // Apply the touchscreen calibration matrix
  touchscreen.setCalibration(calibration);

}

// ================================================================================================
// Measure the touch positions of the touch targets
// ================================================================================================
void measureTouchTargets(XPT2046::Point *targets, XPT2046::Point (&measurements)[3]) {

  // An array of 64 bit points containing the sum of the measurement samples
  struct Point64 {uint64_t x; uint64_t y;} accumulativeMeasurements[3] = {};

  // For the number of samples
  for (uint8_t sample = 0; sample < CALIBRATION_SAMPLES; sample++) {

    // For all targets
    for (uint8_t target = 0; target < 3; target++) {

      // Draw a touch target
      drawTouchTarget(targets[target]);

      // Wait for a touch event
      while (true) {
        
        // If a touch event occurred and the previous touch event was released
        // Stop waiting and continue execution
        if (touchscreen.touched() && touchscreen.released()) { break; }

        // Keep watchdog happy :)
        yield();

      }

      // Get the touch position
      XPT2046::Point position = touchscreen.getTouchPosition();

      // Add the touch position to the sum of a touch measurement
      accumulativeMeasurements[target].x += position.x;
      accumulativeMeasurements[target].y += position.y;

    }

  }

  // For all measurements
  for (uint8_t measurement = 0; measurement < 3; measurement++) {

    // Average the measurement sums and store them in the array of measurements
    measurements[measurement].x = accumulativeMeasurements[measurement].x / CALIBRATION_SAMPLES;
    measurements[measurement].y = accumulativeMeasurements[measurement].y / CALIBRATION_SAMPLES;

  }

}

// ================================================================================================
// Draw a touch target on screen
// ================================================================================================
void drawTouchTarget(XPT2046::Point position) {

  // Clear the frame buffer
  canvas.fillScreen(ILI9341_BLACK);

  // Draw a touch circle
  canvas.drawLine(position.x - 5, position.y, position.x + 5, position.y, ILI9341_WHITE);
  canvas.drawLine(position.x, position.y - 5, position.x, position.y + 5, ILI9341_WHITE);
  canvas.drawCircle(position.x, position.y, 5, ILI9341_WHITE);

  // Create message
  String message = "Please touch the target...";

  // Message bounding box values
  int16_t textX, textY;
  uint16_t textWidth, textHeight;

  // Get message bounding box
  canvas.getTextBounds(message, 0, 0, &textX, &textY, &textWidth, &textHeight);

  // Calculate center of screen for message
  int16_t cursorX = (DISPLAY_WIDTH - textWidth) / 2;
  int16_t cursorY = (DISPLAY_HEIGHT - textHeight) / 2;

  // Set text color and cursor position
  canvas.setTextColor(ILI9341_WHITE);
  canvas.setCursor(cursorX, cursorY);

  // Draw the message to the frame buffer
  canvas.print(message);

  // Write frame buffer to the display
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());

}

// ================================================================================================
// Calculate the calibration matrix
// ================================================================================================
void calculateCalibrationMatrix(XPT2046::Point *targets, XPT2046::Point *measurements, XPT2046::Calibration &calibration) {

  // Calculate the determinant
  double determinant = (measurements[0].x - measurements[2].x) * (measurements[1].y - measurements[2].y) - (measurements[1].x - measurements[2].x) * (measurements[0].y - measurements[2].y);

  // Prevent division by 0
  if (determinant == 0) {

    // Print error message
    Serial.println("[ERROR] Division by zero, calibration cannot be performed with these targets!");

    // Halt execution
    while(true) { yield(); }

  }

  // Calculate the calibration matrix coefficients
  calibration.A = ((targets[0].x - targets[2].x) * (measurements[1].y - measurements[2].y) - (targets[1].x - targets[2].x) * (measurements[0].y - measurements[2].y)) / determinant;
  calibration.B = ((targets[1].x - targets[2].x) * (measurements[0].x - measurements[2].x) - (targets[0].x - targets[2].x) * (measurements[1].x - measurements[2].x)) / determinant;
  calibration.C = (targets[0].x * (measurements[1].x * measurements[2].y - measurements[2].x * measurements[1].y) + targets[1].x * (measurements[2].x * measurements[0].y - measurements[0].x * measurements[2].y) + targets[2].x * (measurements[0].x * measurements[1].y - measurements[1].x * measurements[0].y)) / determinant;
  calibration.D = ((targets[0].y - targets[2].y) * (measurements[1].y - measurements[2].y) - (targets[1].y - targets[2].y) * (measurements[0].y - measurements[2].y)) / determinant;
  calibration.E = ((targets[1].y - targets[2].y) * (measurements[0].x - measurements[2].x) - (targets[0].y - targets[2].y) * (measurements[1].x - measurements[2].x)) / determinant;
  calibration.F = (targets[0].y * (measurements[1].x * measurements[2].y - measurements[2].x * measurements[1].y) + targets[1].y * (measurements[2].x * measurements[0].y - measurements[0].x * measurements[2].y) + targets[2].y * (measurements[0].x * measurements[1].y - measurements[1].x * measurements[0].y)) / determinant;
  calibration.width = DISPLAY_WIDTH;
  calibration.height = DISPLAY_HEIGHT;

}

// ================================================================================================
// Print the calibration matrix result to the Serial Monitor
// ================================================================================================
void printCalibrationMatrix(XPT2046::Calibration calibration) {

  Serial.println("\n\nCalibration matrix coefficients");
  Serial.println("===============================\n");

  Serial.println("A: " + String(calibration.A, 7));
  Serial.println("B: " + String(calibration.B, 7));
  Serial.println("C: " + String(calibration.C, 7));
  Serial.println("D: " + String(calibration.D, 7));
  Serial.println("E: " + String(calibration.E, 7));
  Serial.println("F: " + String(calibration.F, 7));
  Serial.println("W: " + String(calibration.width));
  Serial.println("H: " + String(calibration.height));

  Serial.println("\nUse this line to calibrate the touchscreen in your sketch:\n");

  Serial.println("XPT2046::Calibration calibration = {" + 
    String(calibration.A, 7)   + "," + 
    String(calibration.B, 7)   + "," + 
    String(calibration.C, 7)   + "," +
    String(calibration.D, 7)   + "," +
    String(calibration.E, 7)   + "," +
    String(calibration.F, 7)   + "," +
    String(calibration.width)  + "," +
    String(calibration.height) +
  "};\n");

}

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
  canvas.drawLine(0, position.y, DISPLAY_WIDTH, position.y, ILI9341_RED);
  canvas.drawLine(position.x, 0, position.x, DISPLAY_HEIGHT, ILI9341_RED);
  canvas.drawCircle(position.x, position.y, 5, ILI9341_RED);

  // Write frame buffer to the display
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());

}