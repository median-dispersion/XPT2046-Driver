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
Point averageSamples(Point *samples, uint8_t numberOfSamples) {

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

//-------------------------------------------------------------------------------------------------
// XPT2046 Public

// ================================================================================================
// Constructor
// ================================================================================================
XPT2046::XPT2046(uint8_t csPin, uint8_t irqPin): _csPin(csPin), _irqPin(irqPin), _spi(&SPI), _samples(50), _rotation(0) {}

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
// Update the touch position
// ================================================================================================
void XPT2046::update() {

  // Begin an SPI transaction
  _spi->beginTransaction(SPI_SETTINGS);

  // Select the XPT2046 via the chip select pin
  digitalWrite(_csPin, LOW);

  // Create an array of samples
  Point samples[_samples];

  // For the number of specified samples
  for (uint8_t sample = 0; sample < _samples; sample++) {

    // Request the touch X and Y position from XPT2046 via SPI
    // Store the result in the sample array
    samples[sample].x = readValue(POSITION_X, _spi);
    samples[sample].y = readValue(POSITION_Y, _spi);

  }

  // Send a power down command to the XPT2046
  _spi->transfer(POWER_DOWN);

  // Deselect the XPT2046
  digitalWrite(_csPin, HIGH);

  // End the SPI transaction
  _spi->endTransaction();

  // Average the samples and store the result in the _position variable
  _position = averageSamples(samples, _samples);

}

// ================================================================================================
// Return the rotated touch position
// ================================================================================================
Point XPT2046::getTouchPosition() {

  // Update the touch position
  update();

  // Create a new touch position
  Point position;

  // Depending on the selected rotation, rotate the X and Y coordinates
  // These rotation values will give the same result as rotating an ILI9341 TFT screen with the Adafruit ILI9341 library
  switch (_rotation) {

    case 0:
      position.x = _position.x;
      position.y = 4095 - _position.y;
    break;

    case 1:
      position.x = 4095 - _position.y;
      position.y = 4095 - _position.x;
    break;

    case 2:
      position.x = 4095 - _position.x;
      position.y = _position.y;
    break;

    case 3:
      position.x = _position.y;
      position.y = _position.x;
    break;

  }

  // Return the rotated position
  return position;

}