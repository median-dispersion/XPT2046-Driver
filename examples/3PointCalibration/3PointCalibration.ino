/*

  This sketch performs a 3 point calibration on a touchscreen that uses the XPT2046 controller.
  The touchscreen used in this case is an ILI9341 TFT display driven by the Adafruit GFX & ILI9341 library.
  The resulting calibration matrix can then be used in other sketches to calibrate and map the touch positions to the display's pixel grid.

  Usage:
  1. Change the User configuration (#define) to your needs
  2. Compile and upload the sketch
  3. Tap the touch targets
  4. Copy the calculated calibration matrix from the serial console
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
// i.e., how often to touch the 3 touch target for before calculating the calibration matrix
#define CALIBRATION_SAMPLES 3

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

bool    calibrated = false; // Flag for checking if touchscreen is calibrated
uint8_t samples    = 0;     // Current sample being taken
uint8_t target     = 0;     // Current target being displayed
bool    released   = true;  // Flag for checking if touchscreen has been released

// Calibration matrix
XPT2046::Calibration calibration; 

// Touch targets
XPT2046::Point targets[3] = {

  {DISPLAY_WIDTH * 0.7, 15},
  {DISPLAY_WIDTH - 15, DISPLAY_HEIGHT- 15},
  {15, DISPLAY_HEIGHT * 0.5}

};

// Measurements of the touch positions
struct Point64 {uint64_t x; uint64_t y;} measurements[3*CALIBRATION_SAMPLES];

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

  // Set the calibration matrix width and height to the display's width and height
  calibration.width  = DISPLAY_WIDTH;
  calibration.height = DISPLAY_HEIGHT;

  // Draw the first touch target on screen
  drawTarget(targets[target]);

}

// ================================================================================================
// Main loop
// ================================================================================================
void loop() {

  // If touchscreen is not calibrated
  if (!calibrated) {

    // Check if samples taken are less than the number of total calibration samples
    if (samples < CALIBRATION_SAMPLES) {

      // If current target is in the list of targets
      if (target < sizeof(targets) / sizeof(targets[0])) {

        // If the touchscreen is being touched and the previous touch event has been released
        if (touchscreen.touched() && touchscreen.released()) {
          
          // Get the touch position
          XPT2046::Point position = touchscreen.getTouchPosition();

          // Add the X and Y position to the accumulative X and Y position for that measurement
          measurements[target].x += position.x;
          measurements[target].y += position.y;

          // Increase to the next target
          target++;

          // If still in the list of targets
          if (target < sizeof(targets) / sizeof(targets[0])) {
            
            // Draw the next target
            drawTarget(targets[target]);

          }

        }
      
      // If current target number is not in the list of targets
      } else {
        
        // Reset to the first target
        target = 0;

        // Draw the first target
        drawTarget(targets[target]);

        // Increase the number of samples taken
        samples++;

      }

    // If sampling process has finished
    } else {
      
      // Calculate the calibration matrix
      calculateCalibrationMatrix();

      // Print the calibration matrix result to the serial console
      printCalibrationMatrix();

      // Set the touchscreen calibration to the calculated matrix
      touchscreen.setCalibration(calibration);

      // Rest debounce timeout to make the touchscreen more responsive
      touchscreen.setDebounceTimeout(0);

      // Set the calibration flag to true
      calibrated = true;

      // Draw the info screen
      drawCalibratedScreen({DISPLAY_WIDTH / 2, DISPLAY_HEIGHT / 2});

    }

  // If touchscreen is calibrated
  } else {

    // Check if the touchscreen is being touched
    if (touchscreen.touched()) {

      // Draw the info screen and crosshair
      drawCalibratedScreen(touchscreen.getTouchPosition());

    }

  }

}

// ================================================================================================
// Draw a touch target on screen
// ================================================================================================
void drawTarget(XPT2046::Point position) {

  // Clear the frame buffer
  canvas.fillScreen(ILI9341_BLACK);

  // Draw circle
  canvas.drawLine(position.x - 5, position.y, position.x + 5, position.y, ILI9341_WHITE);
  canvas.drawLine(position.x, position.y - 5, position.x, position.y + 5, ILI9341_WHITE);
  canvas.drawCircle(position.x, position.y, 5, ILI9341_WHITE);

  // Create message
  String message = "Please touch the target...";

  // Message bounding box values
  int16_t textX;
  int16_t textY;
  uint16_t textWidth;
  uint16_t textHeight;

  // Get message bounding box
  canvas.getTextBounds(message, 0, 0, &textX, &textY, &textWidth, &textHeight);

  // Calculate center of screen for message
  int16_t cursorX = (DISPLAY_WIDTH - textWidth) / 2;
  int16_t cursorY = (DISPLAY_HEIGHT - textHeight) / 2;

  // Set text color
  canvas.setTextColor(ILI9341_WHITE);

  // Set cursor position
  canvas.setCursor(cursorX, cursorY);

  // Draw the message to the frame buffer
  canvas.print(message);

  // Write frame buffer to the display
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());

}

// ================================================================================================
// Draw the info screen and crosshair
// ================================================================================================
void drawCalibratedScreen(XPT2046::Point position) {

  // Clear the frame buffer
  canvas.fillScreen(ILI9341_BLACK);

  // Create the status message
  String status = "A: " + String(calibration.A, 7)   + "\n" +
                  "B: " + String(calibration.B, 7)   + "\n" +
                  "C: " + String(calibration.C, 7)   + "\n" +
                  "D: " + String(calibration.D, 7)   + "\n" +
                  "E: " + String(calibration.E, 7)   + "\n" +
                  "F: " + String(calibration.F, 7)   + "\n" +
                  "W: " + String(calibration.width)  + "\n" +
                  "H: " + String(calibration.height) + "\n" +
                  "X: " + String(position.x)               + "\n" +
                  "Y: " + String(position.y);

  // Set the text color to white
  canvas.setTextColor(ILI9341_WHITE);

  // Set the cursor to the top left position
  canvas.setCursor(0, 0);

  // Press the status massage
  canvas.print(status);

  // Draw crosshair
  canvas.drawLine(0, position.y, DISPLAY_WIDTH, position.y, ILI9341_RED);
  canvas.drawLine(position.x, 0, position.x, DISPLAY_HEIGHT, ILI9341_RED);
  canvas.drawCircle(position.x, position.y, 5, ILI9341_RED);

  // Write frame buffer to the display
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());

}

// ================================================================================================
// Calculate the calibration matrix
// ================================================================================================
void calculateCalibrationMatrix() {

  // Get the summed up measurement results and average them
  float measurementX1 = measurements[0].x / CALIBRATION_SAMPLES;
  float measurementX2 = measurements[1].x / CALIBRATION_SAMPLES;
  float measurementX3 = measurements[2].x / CALIBRATION_SAMPLES;
  float measurementY1 = measurements[0].y / CALIBRATION_SAMPLES;
  float measurementY2 = measurements[1].y / CALIBRATION_SAMPLES;
  float measurementY3 = measurements[2].y / CALIBRATION_SAMPLES;

  // Get the target pixel coordinates
  float targetX1 = targets[0].x;
  float targetX2 = targets[1].x;
  float targetX3 = targets[2].x;
  float targetY1 = targets[0].y;
  float targetY2 = targets[1].y;
  float targetY3 = targets[2].y;

  // Calculate the determinant
  float determinant = (measurementX1 - measurementX3) * (measurementY2 - measurementY3) - (measurementX2 - measurementX3) * (measurementY1 - measurementY3);

  // Prevent division by 0
  if (determinant == 0) {

    // Print error message
    Serial.println("[ERROR] Division by zero, calibration cannot be performed with these points!");

    // Halt execution
    while(true) { yield(); }

  }

  // Calculate the calibration matrix coefficients
  calibration.A = ((targetX1 - targetX3) * (measurementY2 - measurementY3) - (targetX2 - targetX3) * (measurementY1 - measurementY3)) / determinant;
  calibration.B = ((targetX2 - targetX3) * (measurementX1 - measurementX3) - (targetX1 - targetX3) * (measurementX2 - measurementX3)) / determinant;
  calibration.C = (targetX1 * (measurementX2 * measurementY3 - measurementX3 * measurementY2) + targetX2 * (measurementX3 * measurementY1 - measurementX1 * measurementY3) + targetX3 * (measurementX1 * measurementY2 - measurementX2 * measurementY1)) / determinant;
  calibration.D = ((targetY1 - targetY3) * (measurementY2 - measurementY3) - (targetY2 - targetY3) * (measurementY1 - measurementY3)) / determinant;
  calibration.E = ((targetY2 - targetY3) * (measurementX1 - measurementX3) - (targetY1 - targetY3) * (measurementX2 - measurementX3)) / determinant;
  calibration.F = (targetY1 * (measurementX2 * measurementY3 - measurementX3 * measurementY2) + targetY2 * (measurementX3 * measurementY1 - measurementX1 * measurementY3) + targetY3 * (measurementX1 * measurementY2 - measurementX2 * measurementY1)) / determinant;

}

// ================================================================================================
// Print the calibration matrix result to the serial console
// ================================================================================================
void printCalibrationMatrix() {

  Serial.println("\n\nCalibration matrix coefficients");
  Serial.println("===============================\n");

  Serial.println("A: " + String(calibration.A, 7));
  Serial.println("B: " + String(calibration.B, 7));
  Serial.println("C: " + String(calibration.C, 7));
  Serial.println("D: " + String(calibration.D, 7));
  Serial.println("E: " + String(calibration.E, 7));
  Serial.println("F: " + String(calibration.F, 7));

  Serial.println("");

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