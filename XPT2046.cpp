#include "XPT2046.h"

// Command bytes for the XPT2046
#define POSITION_X  0xD1
#define POSITION_Y  0x91
#define POSITION_Z1 0xB1
#define POSITION_Z2 0xC1
#define READ_VALUE  0x00
#define POWER_DOWN  0x80

// SPI settings
#define SPI_SETTINGS SPISettings(2000000, MSBFIRST, SPI_MODE0)

//-------------------------------------------------------------------------------------------------
// Generic functions

// ================================================================================================
// Read a value from the XPT2046 via SPI
// ================================================================================================
uint16_t readValue(uint8_t command, SPIClass *spi) {

  // Send the command byte
  spi->transfer(command);

  // Read the upper and lower bits of the response
  uint8_t upperBits = spi->transfer(READ_VALUE);
  uint8_t lowerBits = spi->transfer(READ_VALUE);

  // Combine the two parts into a 12-bit value
  return (upperBits << 5) | (lowerBits >> 3);

}

// ================================================================================================
// Average samples
// ================================================================================================
XPT2046::Point averageSamples(XPT2046::Point *samples, uint8_t numberOfSamples) {

  // TODO
  // This is the most basic averaging approach
  // A more complex algorithm that removes outliers should be used

  float x = 0.0;
  float y = 0.0;

  for (uint8_t sample = 0; sample < numberOfSamples; sample++) {

    x += samples[sample].x;
    y += samples[sample].y;

  }

  return {x / numberOfSamples, y / numberOfSamples};

}

// ================================================================================================
// Read the touch position
// ================================================================================================
XPT2046::Point readTouchPosition(uint8_t numberOfSamples, uint8_t csPin, SPIClass *spi) {

  // Begin an SPI transaction
  spi->beginTransaction(SPI_SETTINGS);

  // Select the XPT2046 via the chip select pin
  digitalWrite(csPin, LOW);

  // Create an array of samples
  XPT2046::Point samples[numberOfSamples];

  // For the number of specified samples
  for (uint8_t sample = 0; sample < numberOfSamples; sample++) {

    // Request the touch X and Y position from XPT2046 via SPI
    // Store the result in the sample array
    samples[sample].x = readValue(POSITION_X, spi);
    samples[sample].y = readValue(POSITION_Y, spi);

  }

  // Send a power down command to the XPT2046
  spi->transfer(POWER_DOWN);

  // Deselect the XPT2046
  digitalWrite(csPin, HIGH);

  // End the SPI transaction
  spi->endTransaction();

  // Average the samples return the resulting position
  return averageSamples(samples, numberOfSamples);

}

// ================================================================================================
// Rotate position
// ================================================================================================
XPT2046::Point rotatePosition(XPT2046::Point position, uint8_t rotation) {

  XPT2046::Point rotatedPosition;

  // Depending on the selected rotation, rotate the X and Y coordinates
  // These rotation values will give the same result as rotating an ILI9341 TFT screen with the Adafruit ILI9341 library
  switch (rotation) {

    case 0:
      rotatedPosition.x = position.x;
      rotatedPosition.y = 4095 - position.y;
    break;

    case 1:
      rotatedPosition.x = 4095 - position.y;
      rotatedPosition.y = 4095 - position.x;
    break;

    case 2:
      rotatedPosition.x = 4095 - position.x;
      rotatedPosition.y = position.y;
    break;

    case 3:
      rotatedPosition.x = position.y;
      rotatedPosition.y = position.x;
    break;

  }

  // Return rotated position
  return rotatedPosition;

}

// ================================================================================================
// Map the position based on the calibration matrix
// ================================================================================================
XPT2046::Point mapPosition(XPT2046::Point position, XPT2046::CalibrationMatrix calibration) {

  // Calculated mapped X and Y position
  int16_t x = calibration.A * position.x + calibration.B * position.y + calibration.C;
  int16_t y = calibration.D * position.y + calibration.E * position.y + calibration.F;

  // Prevent underflow
  if (x < 0) { x = 0; }
  if (y < 0) { y = 0; }

  // Prevent overflow
  if (x > calibration.width)  { x = calibration.width;  }
  if (y > calibration.height) { y = calibration.height; }

  // Return mapped positon
  return {x, y};  

}

//-------------------------------------------------------------------------------------------------
// XPT2046 Public

// ================================================================================================
// Constructor
// ================================================================================================
XPT2046::XPT2046(uint8_t csPin, uint8_t irqPin): _csPin(csPin), _irqPin(irqPin), _spi(&SPI), _samples(20), _rotation(0), _calibrated(false) {}

// ================================================================================================
// Initialize everything
// ================================================================================================
void XPT2046::begin() {

  // Set the appropriate pin modes for the CS and IRQ pin
  pinMode(_csPin, OUTPUT);
  pinMode(_irqPin, INPUT);

  // SPI begin
  _spi->begin();

  // Deselect the XPT2046 by setting chip select pin to HIGH
  // LOW = selected, HIGH = unselected
  digitalWrite(_csPin, HIGH);

}

// ================================================================================================
// Set the number of samples to average over
// ================================================================================================
void XPT2046::setSampleCount(uint8_t samples) {

  _samples = samples;

}

// ================================================================================================
// Set the rotation
// ================================================================================================
void XPT2046::setRotation(uint8_t rotation) {

  _rotation = rotation;

}

// ================================================================================================
// Set the calibration matrix
// ================================================================================================
void  XPT2046::setCalibration(CalibrationMatrix calibration) {

  _calibration = calibration;

  // Set the calibration flag to true
  _calibrated  = true;

}

// ================================================================================================
// Return the touch status
// ================================================================================================
bool XPT2046::touched() {

  // If the IRQ pin is low, a touch event is occurring
  if (digitalRead(_irqPin) == LOW) {

    // TODO
    // Add a check for a pressure threshold using the Z1 and Z2 values

    // Return true
    return true;

  }

  // If no touch event return false
  return false;

}

// ================================================================================================
// Return touch position
// ================================================================================================
XPT2046::Point XPT2046::getTouchPosition() {

  // Read the touch position
  Point position = readTouchPosition(_samples, _csPin, _spi);

  // Rotate the position
  position = rotatePosition(position, _rotation);

  // If the calibration matrix is set, map the position
  if (_calibrated) { position = mapPosition(position, _calibration); }

  // Return the position
  return position;

}