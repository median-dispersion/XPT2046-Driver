#include "XPT2046.h"

// Default initialization values
#define DEFAULT_SAMPLE_COUNT     20
#define DEFAUTL_ROTATION         0
#define DEFAULT_DEBOUNCE_TIMEOUT 10

// Command bytes for the XPT2046
#define POSITION_X  0xD0 // 1 101 0 0 00
#define POSITION_Y  0x90 // 1 001 0 0 00
#define POSITION_Z1 0xB0 // 1 011 0 0 00
#define POSITION_Z2 0xC0 // 1 100 0 0 00
#define READ_VALUE  0x00 // 0 000 0 0 00
#define POWER_DOWN  0x80 // 1 000 0 0 00

// SPI settings
#define SPI_SETTINGS SPISettings(2000000, MSBFIRST, SPI_MODE0)

// Touchscreen dead zone
// The raw minimum and maximum values of the touch area are around 200 and 3900
// So a dead zone disregarding all samples below 50 should be safe
#define DEAD_ZONE 50

//-------------------------------------------------------------------------------------------------
// XPT2046 Public

// ================================================================================================
// Constructor
// ================================================================================================
XPT2046::XPT2046(uint8_t csPin, uint8_t irqPin):

  // Set private members to their default states
  _csPin(csPin),
  _irqPin(irqPin),
  _spi(&SPI),
  _sampleCount(DEFAULT_SAMPLE_COUNT),
  _rotation(DEFAUTL_ROTATION),
  _calibrated(false),
  _debounceTimeoutMilliseconds(DEFAULT_DEBOUNCE_TIMEOUT),
  _lastTouchMilliseconds(0),
  _released(true)

{}

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

  _sampleCount = samples;

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
void XPT2046::setCalibration(Calibration calibration) {

  _calibration = calibration;

  // Set the calibration flag to true
  _calibrated  = true;

}

// ================================================================================================
// Set the debounce timeout in milliseconds
// ================================================================================================
void XPT2046::setDebounceTimeout(uint16_t timeoutMilliseconds) {

  _debounceTimeoutMilliseconds = timeoutMilliseconds;

}

// ================================================================================================
// Returns if the touchscreen is being touched
// ================================================================================================
bool XPT2046::touched() {

  // If the debounce timeout has passed
  if (millis() - _lastTouchMilliseconds >= _debounceTimeoutMilliseconds) {

    // Check if touch event is occurring
    if (_touched()) {

      // Update the last touch time
      _lastTouchMilliseconds = millis();

      // Return true
      return true;

    }

  }

  // If no touch event or still in debounce timeout, return false
  return false;

}

// ================================================================================================
// Returns if a touch event has been released
// ================================================================================================
bool XPT2046::released() {

  // Get the current touch status
  // This will also reset the released flag if no touch is occurring
  bool touched = _touched();

  // If currently touching but the released flag still true
  // Meaning this is the initial touch event
  if (touched && _released) {

    // Set released to false for subsequent tests
    _released = false;

    // But return true because this is the initial touch event
    return true;

  }

  // If it is a subsequent touch event, simply return the released flag
  return _released;

}

// ================================================================================================
// Return touch position
// ================================================================================================
XPT2046::Point XPT2046::getTouchPosition() {

  // Read the touch position
  Point position = _readTouchPosition();

  // Rotate the position
  position = _rotatePosition(position);

  // If the calibration matrix is set, map the position
  if (_calibrated) { position = _calibratePosition(position); }

  // Return the position
  return position;

}

//-------------------------------------------------------------------------------------------------
// XPT2046 Private

// ================================================================================================
// Read a value from the XPT2046 via SPI
// ================================================================================================
uint16_t XPT2046::_readValue(uint8_t command) {

  // Send the command byte
  _spi->transfer(command);

  // Read the upper and lower bits of the response
  uint8_t upperBits = _spi->transfer(READ_VALUE);
  uint8_t lowerBits = _spi->transfer(READ_VALUE);

  // Combine the two parts into a 12-bit value
  return (upperBits << 5) | (lowerBits >> 3);

}

// ================================================================================================
// Average samples
// ================================================================================================
XPT2046::Point XPT2046::_averageSamples(XPT2046::Point *samples) {

  float x = 0.0;
  float y = 0.0;
  uint8_t validSamples = 0;

  // For all samples
  for (uint8_t sample = 0; sample < _sampleCount; sample++) {

    // Check if sample is outside of dead zone meaning it is a valid sample
    if (samples[sample].x >= DEAD_ZONE && samples[sample].y >= DEAD_ZONE) {

      // Add the X and Y position to the sum of X and Y positions
      x += samples[sample].x;
      y += samples[sample].y;

      // Increase the number of valid samples
      validSamples++;

    }

  }

  // If there were any valid samples
  if (validSamples) {

    // Average by the number of valid samples
    return {x / validSamples, y / validSamples};

  // If there were no valid samples
  // i.e., the touch was released before any samples were taken
  } else {

    // Return a touch position outside the maximum range
    return {UINT16_MAX, UINT16_MAX};

  }

}

// ================================================================================================
// Read the touch position via SPI
// ================================================================================================
XPT2046::Point XPT2046::_readTouchPosition() {

  // Begin an SPI transaction
  _spi->beginTransaction(SPI_SETTINGS);

  // Select the XPT2046 via the chip select pin
  digitalWrite(_csPin, LOW);

  // Create an array of samples
  XPT2046::Point samples[_sampleCount] = {};

  // For the number of specified samples
  for (uint8_t sample = 0; sample < _sampleCount; sample++) {

    // Only add the sample if touch event is still occurring
    if (_touched()) {

      // Request the touch X and Y position from XPT2046 via SPI
      // Store the result in the sample array
      samples[sample].x = _readValue(POSITION_X);
      samples[sample].y = _readValue(POSITION_Y);

    }

  }

  // Send a power down command to the XPT2046 using the _readValue function
  _readValue(POWER_DOWN);

  // Deselect the XPT2046
  digitalWrite(_csPin, HIGH);

  // End the SPI transaction
  _spi->endTransaction();

  // Average the samples return the resulting position
  return _averageSamples(samples);

}

// ================================================================================================
// Rotate a position
// ================================================================================================
XPT2046::Point XPT2046::_rotatePosition(XPT2046::Point position) {

  XPT2046::Point rotatedPosition;

  // Depending on the selected rotation, rotate the X and Y coordinates
  // These rotation values will give the same result as rotating an ILI9341 TFT screen with the Adafruit ILI9341 library
  switch (_rotation) {

    case 0: // 0°
      rotatedPosition.x = position.x;
      rotatedPosition.y = 4095 - position.y;
    break;

    case 1: // 90°
      rotatedPosition.x = 4095 - position.y;
      rotatedPosition.y = 4095 - position.x;
    break;

    case 2: // 180°
      rotatedPosition.x = 4095 - position.x;
      rotatedPosition.y = position.y;
    break;

    case 3: // 270°
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
XPT2046::Point XPT2046::_calibratePosition(XPT2046::Point position) {

  // Calculated mapped X and Y position
  int16_t x = _calibration.A * position.x + _calibration.B * position.y + _calibration.C;
  int16_t y = _calibration.D * position.y + _calibration.E * position.y + _calibration.F;

  // Prevent underflow
  if (x < 0) { x = 0; }
  if (y < 0) { y = 0; }

  // Prevent overflow
  if (x > _calibration.width)  { x = _calibration.width;  }
  if (y > _calibration.height) { y = _calibration.height; }

  // Return mapped positon
  return {x, y};  

}

// ================================================================================================
// Return the touch status
// ================================================================================================
bool XPT2046::_touched() {

  // If the IRQ pin is low, a touch event is occurring
  if (digitalRead(_irqPin) == LOW) {

    // TODO
    // Add a check for a pressure threshold using the Z1 and Z2 values

    // Return true
    return true;

  }

  // Always reset the released flag if not touch event is captured
  _released = true;

  // If no touch event return false
  return false;

}