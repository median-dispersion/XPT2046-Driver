#include "Arduino.h"
#include "SPI.h"

class XPT2046 {

  //-----------------------------------------------------------------------------------------------
  // Public

  public:

    // Point structure
    struct Point {

      uint16_t x;
      uint16_t y;

    };

    // Calibration structure
    struct Calibration {

      float A;
      float B;
      float C;
      float D;
      float E;
      float F;
      uint16_t width;
      uint16_t height;

    };

    XPT2046(uint8_t csPin, uint8_t irqPin);        // Constructor

    void  begin();                                 // Initialize everything
    void  setSampleCount(uint8_t samples);         // Set the number of samples to average over
    void  setRotation(uint8_t rotation);           // Set the rotation
    void  setCalibration(Calibration calibration); // Set the calibration matrix
    bool  touched();                               // Returns if the touchscreen is being touched
    Point getTouchPosition();                      // Return the touch position

  //-----------------------------------------------------------------------------------------------
  // Private

  private:

    uint8_t     _csPin;       // CS pin
    uint8_t     _irqPin;      // IRQ pin
    SPIClass    *_spi;        // SPI pointer
    uint8_t     _sampleCount; // Current number of samples to average over
    uint8_t     _rotation;    // Current rotation
    Calibration _calibration; // Calibration matrix
    bool        _calibrated;  // Flag for checking if calibration was set

    uint16_t _readValue(uint8_t command);        // Read a value from the XPT2046 via SPI
    Point    _averageSamples(Point *samples);    // Average samples
    Point    _readTouchPosition();               // Read the touch position via SPI
    Point    _rotatePosition(Point position);    // Rotate a position
    Point    _calibratePosition(Point position); // Map the position based on the calibration matrix
    bool     _touched();                         // Return the touch status

};