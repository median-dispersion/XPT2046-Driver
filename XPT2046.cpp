// https://github.com/median-dispersion/XPT2046-Driver

#include "XPT2046.h"

// Default initialization values
#define DEFAUTL_ROTATION         0
#define DEFAULT_SAMPLE_COUNT     20
#define DEFAULT_DEBOUNCE_TIMEOUT 100
#define DEFAULT_TOUCH_PRESSURE   3.5
#define DEFAULT_DEAD_ZONE        50
#define DEFAULT_POWER_DOWN       true

// Command bits for the XPT2046
// See datasheet for bit values
#define XPT2046_POSITION_X  0b11010000
#define XPT2046_POSITION_Y  0b10010000
#define XPT2046_POSITION_Z1 0b10110000
#define XPT2046_POSITION_Z2 0b11000000
#define XPT2046_READ_VALUE  0b00000000
#define XPT2046_POWER_DOWN  0b10000000

// SPI settings
#define SPI_SETTINGS SPISettings(2000000, MSBFIRST, SPI_MODE0)

// Maximum value of a 12-bit unsigned integer
#define UINT12_MAX 4095

//-------------------------------------------------------------------------------------------------
// XPT2046 Public

// ================================================================================================
// Constructor
// ================================================================================================
XPT2046::XPT2046(uint8_t csPin, uint8_t irqPin):

  // Set private members to their default states
  _csPin(csPin),
  _irqPin(irqPin),
  _rotation(DEFAUTL_ROTATION),
  _calibrated(false),
  _sampleCount(DEFAULT_SAMPLE_COUNT),
  _debounceTimeoutMilliseconds(DEFAULT_DEBOUNCE_TIMEOUT),
  _lastTouchMilliseconds(0),
  _touchPressure(DEFAULT_TOUCH_PRESSURE),
  _deadZone(DEFAULT_DEAD_ZONE),
  _powerDown(DEFAULT_POWER_DOWN),
  _touched(false),
  _released(true),
  _updated(false),
  _transferEnabled(false)

{}

// ================================================================================================
// Initialize everything
// ================================================================================================
void XPT2046::begin() {

  // Set the appropriate pin modes for the CS and IRQ pin
  pinMode(_csPin, OUTPUT);
  pinMode(_irqPin, INPUT);

  // SPI begin
  SPI.begin();

  // Deselect the XPT2046 by setting chip select pin to HIGH
  // LOW = selected, HIGH = unselected
  digitalWrite(_csPin, HIGH);

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
// Set the number of samples to average over
// ================================================================================================
void XPT2046::setSampleCount(uint8_t samples) {

  _sampleCount = samples;

}

// ================================================================================================
// Set the debounce timeout in milliseconds
// ================================================================================================
void XPT2046::setDebounceTimeout(uint16_t timeoutMilliseconds) {

  _debounceTimeoutMilliseconds = timeoutMilliseconds;

}

// ================================================================================================
// Set the touch pressure
// ================================================================================================
void XPT2046::setTouchPressure(float pressure) {

  _touchPressure = pressure;

}

// ================================================================================================
// Set the touch area dead zone
// ================================================================================================
void XPT2046::setDeadZone(uint16_t deadZone) {

  _deadZone = deadZone;

}

// ================================================================================================
// Set power-down state after SPI transaction
// ================================================================================================
void XPT2046::setPowerDown(bool status) {

  _powerDown = status;

}

// ================================================================================================
// Returns if the touchscreen is being touched
// ================================================================================================
bool XPT2046::touched() {

  // Always update touch status, i.e., the _touched and _released flags
  _updateTouchStatus();

  // Set the update flag to true
  _updated = true;

  // Return touch status
  return _touched;

}

// ================================================================================================
// Returns if a touch event has been released
// ================================================================================================
bool XPT2046::released() {

  // Check if touch status was previously updated
  if(!_updated) { 
    
    // If not, update touch status
    _updateTouchStatus();
  
  // If it was previously updated
  } else {

    // Reset the update status
    _updated = false;

  }

  // Return the release status
  return _released;

}

// ================================================================================================
// Return touch position
// ================================================================================================
XPT2046::Point XPT2046::getTouchPosition() {

  // Touch position
  XPT2046::Point position;

  // Read the touch position via SPI
  position = _readTouchPosition();

  // If it is a valid raw touch position, i.e., touch position not outside touch bounds
  if (valid(position)) {

    // Rotate raw touch position
    position = _rotateTouchPosition(position);

    // If calibration is set
    if (_calibrated) {

      // Map the touch position to the display pixel grid
      position = _mapTouchPosition(position);

      // Rotate the mapped position
      position = _rotateMappedPosition(position);

    }

  }

  // Return touch position
  return position;

}

// ================================================================================================
// Check if a given positon is a valid touch position
// ================================================================================================
bool XPT2046::valid(XPT2046::Point position) {

  // In the case that a touch event was lifted after touched() was checked and before the first sample was taken, the touch position will become invalid
  // The getTouchPosition() function will return a position with X and Y at the maximum values for an unsigned 16-bit integer
  // This function will check if a touch position is invalid

  // Check if position is not outside the touch bounds
  if (position.x != UINT16_MAX && position.y != UINT16_MAX) {

    // Return valid position
    return true;

  }

  // Return invalid position
  return false;

}

//-------------------------------------------------------------------------------------------------
// XPT2046 Private

// ================================================================================================
// Enable SPI data transfer
// ================================================================================================
bool XPT2046::_enableDataTransfer() {

  // If data transfer is disabled
  if (!_transferEnabled) {

    // Begin an SPI transaction
    SPI.beginTransaction(SPI_SETTINGS);

    // Select the XPT2046 via the chip select pin
    digitalWrite(_csPin, LOW);

    // Set the transfer flag to true
    _transferEnabled = true;

    // Return that data transfer was enabled
    return true;

  // If data transfer was already enabled
  } else {

    // Return that data transfer was not enabled
    return false;

  }

}

// ================================================================================================
// Disable SPI data transfer
// ================================================================================================
bool XPT2046::_disableDataTransfer() {

  // If data transfer is enabled
  if (_transferEnabled) {

    // If power-down is enabled
    if (_powerDown) {

      // Send a power-down command to the XPT2046
      SPI.transfer(XPT2046_POWER_DOWN);

    }

    // Deselect the XPT2046
    digitalWrite(_csPin, HIGH);

    // End the SPI transaction
    SPI.endTransaction();

    // Set the transfer flag to false
    _transferEnabled = false;

    // Return that data transfer was disabled
    return true;

  } else {

    // Return that data transfer was not disabled
    return false;

  }

}

// ================================================================================================
// Read data via SPI
// ================================================================================================
uint16_t XPT2046::_readData(uint8_t data) {

  // Send the command bits
  SPI.transfer(data);

  // Read the upper and lower bits of the response
  uint8_t upperBits = SPI.transfer(XPT2046_READ_VALUE);
  uint8_t lowerBits = SPI.transfer(XPT2046_READ_VALUE);

  // Combine the two parts into a 12-bit value
  // In reality, the XPT2046 seems to return a 13-Bit value where the first bit is a junk bit always set to 0
  // That is why the bit shift operation is the way it is. If there is no junk bit, the bit shift would be:
  // return (upperBits << 4) | (lowerBits >> 4);
  return (upperBits << 5) | (lowerBits >> 3);

}

// ================================================================================================
// Return the touch status
// ================================================================================================
bool XPT2046::_getTouchStatus() {

  // If the IRQ pin is low, a touch event is occurring
  if (digitalRead(_irqPin) == LOW) {

    // Enable SPI data transfer
    bool enabled = _enableDataTransfer();

    // Read X, Z1 and Z2 values
    float x  = _readData(XPT2046_POSITION_X);
    float z1 = _readData(XPT2046_POSITION_Z1);
    float z2 = _readData(XPT2046_POSITION_Z2);

    // If data transfer was enabled, disable SPI data transfer
    if (enabled) { _disableDataTransfer(); }

    // Calculate approximate pressure
    // This formula results in values that decreases with pressure
    // This value is not very accurate or stable, but good enough to judge if the pressure for a proper touchdown has been reached
    float pressure = (x / UINT12_MAX) * ((z2 / z1) - 1.0);

    // If the touch pressure threshold has been reached
    if (pressure <= _touchPressure) {
      
      // Return true
      return true;

    }

  }

  // If no touch event return false
  return false;

}

// ================================================================================================
// Update the touched and released flags
// ================================================================================================
void XPT2046::_updateTouchStatus() {

  // Check if the debounce timeout has passed
  if (millis() - _lastTouchMilliseconds >= _debounceTimeoutMilliseconds) {

    // Get the current touch status
    bool touched = _getTouchStatus();

    // If currently touching and touch flag was already set to true
    // Meaning this is the second pass over this check
    if (touched && _touched) {

      // Set the released flag to false
      _released = false;

    }

    // If currently touching and touched flag has not been set
    // Meaning this is the first pass over this check
    if (touched && !_touched) {

      // Set the touched flag to true
      _touched = true;

    }

    // If not touching and touched flag is still set
    // Meaning the touch event was released
    if (!touched && _touched) {
      
      // Reset touched flag
      _touched = false;

      // Reset released flag
      _released = true;

      // Start debounce timeout
      _lastTouchMilliseconds = millis();

    }

  }

}

// ================================================================================================
// Read the touch position via SPI
// ================================================================================================
XPT2046::Point XPT2046::_readTouchPosition() {

  // Create an array of samples
  XPT2046::Point samples[_sampleCount] = {};

  // Enable SPI data transfer
  bool enabled = _enableDataTransfer();

  // For the number of specified samples
  for (uint8_t sample = 0; sample < _sampleCount; sample++) {

    // Check if still touching
    if (_getTouchStatus()) {

      // Read X and Y positions and store them in the sample array
      samples[sample].x = _readData(XPT2046_POSITION_X);
      samples[sample].y = _readData(XPT2046_POSITION_Y);

    }

  }

  // If data transfer was enabled, disable SPI data transfer
  if (enabled) { _disableDataTransfer(); }

  // Average the samples
  XPT2046::Point position = _averageTouchSamples(samples);

  // Return touch position
  return position;

}

// ================================================================================================
// Average touch samples
// ================================================================================================
XPT2046::Point XPT2046::_averageTouchSamples(XPT2046::Point *samples) {

  // A standard deviation averaging approach was previously tested
  // But it seemed like it had no effect on touch position stability

  // Create a point outside the touch bounds
  // If no valid samples were taken, this will result in a position outside the touch bounds
  // It can then be used to reject a touch position
  XPT2046::Point position = {UINT16_MAX, UINT16_MAX};

  // Number of samples in the valid touch area
  uint8_t validSamples = 0;

  // Sum of valid X and Y positions
  uint32_t sumX = 0;
  uint32_t sumY = 0;

  // For every sample
  for (uint8_t sample = 0; sample < _sampleCount; sample++) {

    // Check if the sample is inside the valid touch area
    // i.e. if sample is outside of dead zone
    if (samples[sample].x > _deadZone && samples[sample].x < UINT12_MAX - _deadZone && samples[sample].y > _deadZone && samples[sample].y < UINT12_MAX - _deadZone) {

      // Add sample component to corresponding sum
      sumX += samples[sample].x;
      sumY += samples[sample].y;

      // Increase the number of valid samples
      validSamples++;

    }

  }

  // If there are any valid samples
  if (validSamples) {

    // Average valid X and Y positions
    position.x = sumX / validSamples;
    position.y = sumY / validSamples;

  }

  // Return an averaged position
  // Or if no valid samples, a position outside the touch bounds
  return position;

}

// ================================================================================================
// Rotate the raw touch position
// ================================================================================================
XPT2046::Point XPT2046::_rotateTouchPosition(XPT2046::Point position) {

  // Set local rotation variable
  uint8_t rotation = _rotation;

  // If calibration is set, use the calibration rotation instead by overwriting the local rotation
  // The mapped position will then be rotated in a later step according to the rotation that was applied during calibration
  if (_calibrated) { rotation = _calibration.rotation; }

  // Rotated position
  XPT2046::Point rotatedPosition;

  // Depending on the selected rotation, rotate the X and Y coordinates
  // These rotation values will give the same result as rotating an ILI9341 TFT screen with the Adafruit ILI9341 library
  switch (rotation) {

    case 0: // 0°
      rotatedPosition.x = position.x;
      rotatedPosition.y = UINT12_MAX - position.y;
    break;

    case 1: // 90°
      rotatedPosition.x = UINT12_MAX - position.y;
      rotatedPosition.y = UINT12_MAX - position.x;
    break;

    case 2: // 180°
      rotatedPosition.x = UINT12_MAX - position.x;
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
XPT2046::Point XPT2046::_mapTouchPosition(XPT2046::Point position) {

  // Calculated mapped X and Y position
  int16_t x = _calibration.A * position.x + _calibration.B * position.y + _calibration.C;
  int16_t y = _calibration.D * position.x + _calibration.E * position.y + _calibration.F;

  // Prevent underflow
  if (x < 0) { x = 0; }
  if (y < 0) { y = 0; }

  // Prevent overflow
  if (x > _calibration.width)  { x = _calibration.width;  }
  if (y > _calibration.height) { y = _calibration.height; }

  // Return mapped positon
  return {(uint16_t)(x), (uint16_t)(y)};

}

// ================================================================================================
// Rotate the mapped position
// ================================================================================================
XPT2046::Point XPT2046::_rotateMappedPosition(XPT2046::Point position) {

  // Rotated position
  XPT2046::Point rotatedPosition = position;

  // If the calibration rotation is not the same as the set rotation
  if (_calibration.rotation != _rotation) {

    // Set "virtual" rotation depending on permutation of the currently set rotation and the rotation that was applied during matrix calculation
    uint8_t rotation = (_calibration.rotation + _rotation + ((_calibration.rotation % 2) * 2)) % 4;

    // Set width and height variables
    uint16_t width  = _calibration.width;
    uint16_t height = _calibration.height;

    // Swap width and height if necessary
    if ((_calibration.rotation % 2) != (_rotation % 2)) {

      width  = _calibration.height;
      height = _calibration.width;

    }
    
    // Apply rotation
    switch (rotation) {

      case 0: // 0°
        rotatedPosition.x = position.x;
        rotatedPosition.y = position.y;
      break;

      case 1: // 90°
        rotatedPosition.x = position.y;
        rotatedPosition.y = height - position.x;
      break;

      case 2: // 180°
        rotatedPosition.x = width - position.x;
        rotatedPosition.y = height - position.y;
      break;

      case 3: // 270°
        rotatedPosition.x = width - position.y;
        rotatedPosition.y = position.x;
      break;

    }

  }

  // Return rotated position
  return rotatedPosition;

}