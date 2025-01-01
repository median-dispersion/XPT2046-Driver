// https://github.com/median-dispersion/XPT2046-Driver

#ifndef _XPT2046_H
#define _XPT2046_H

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
      uint8_t rotation;

    };

    XPT2046(uint8_t csPin, uint8_t irqPin);                 // Constructor

    void  begin();                                          // Initialize everything
    void  setRotation(uint8_t rotation);                    // Set the rotation
    void  setCalibration(Calibration calibration);          // Set the calibration matrix
    void  setSampleCount(uint8_t samples);                  // Set the number of samples to average over
    void  setDebounceTimeout(uint16_t timeoutMilliseconds); // Set the debounce timeout in milliseconds
    void  setTouchPressure(float pressure);                 // Set the touch pressure
    void  setDeadZone(uint16_t deadZone);                   // Set the touch area dead zone
    void  setPowerDown(bool state);                         // Set power-down state after SPI transaction
    bool  touched();                                        // Returns if the touchscreen is being touched
    bool  released();                                       // Returns if a touch event has been released
    Point getTouchPosition();                               // Return the touch position
    bool  valid(Point position);                            // Check if a given position is a valid touch position

  //-----------------------------------------------------------------------------------------------
  // Private

  private:

    uint8_t     _csPin;                       // CS pin
    uint8_t     _irqPin;                      // IRQ pin
    uint8_t     _rotation;                    // Current rotation
    Calibration _calibration;                 // Calibration matrix
    bool        _calibrated;                  // Flag for checking if calibration was set
    uint8_t     _sampleCount;                 // Current number of samples to average over
    uint16_t    _debounceTimeoutMilliseconds; // Debounce timeout in milliseconds
    uint64_t    _lastTouchMilliseconds;       // Last touch event time in milliseconds
    float       _touchPressure;               // Touch pressure
    uint16_t    _deadZone;                    // Dead zone
    bool        _powerDown;                   // Flag for setting power-down state after SPI communication
    bool        _touched;                     // Flag for if touch area is being touched
    bool        _released;                    // Flag for if a touch event has been released
    bool        _updated;                     // Flag for checking update status of touched and released flags
    bool        _transferEnabled;             // Flag for if SPI data transfer is enabled

    bool     _enableDataTransfer();                 // Enable SPI data transfer
    bool     _disableDataTransfer();                // Disable SPI data transfer
    uint16_t _readData(uint8_t data);               // Read data via SPI
    bool     _getTouchStatus();                     // Return the touch status
    void     _updateTouchStatus();                  // Update the touched and released flags
    Point    _readTouchPosition();                  // Read the touch position via SPI
    Point    _averageTouchSamples(Point *samples);  // Average touch samples
    Point    _rotateTouchPosition(Point position);  // Rotate the raw touch position
    Point    _mapTouchPosition(Point position);     // Map the position based on the calibration matrix
    Point    _rotateMappedPosition(Point position); // Rotate the mapped position

};

#endif