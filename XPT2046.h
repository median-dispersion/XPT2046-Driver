#include "Arduino.h"
#include "SPI.h"

struct Point {

  uint16_t x;
  uint16_t y;

};

class XPT2046 {

  public:

    XPT2046(uint8_t csPin, uint8_t irqPin); // Constructor

    void  begin();                          // Initialize everything
    void  setSampleCount(uint8_t samples);  // Set the number of samples to average over
    void  setRotation(uint8_t rotation);    // Set the rotation
    bool  touched();                        // Return the touch status
    void  update();                         // Update the touch position
    Point getTouchPosition();               // Return the rotated touch position

  private:

    uint8_t  _csPin;    // CS pin
    uint8_t  _irqPin;   // IRQ pin
    SPIClass _spi;      // SPI class
    uint8_t  _samples;  // Current number of samples to average over
    uint8_t  _rotation; // Current rotation
    Point    _position; // Touch position

};