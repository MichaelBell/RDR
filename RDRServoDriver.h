/*!
 *  This is a driver for the Adafruit PWM Servo driver based on 
 *  the Adafruit driver.  License notice for the original Adafruit 
 *  library below.  This version is also under the BSD license.
 * 
 *
 *  This is a library for our Adafruit 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit 16-channel PWM & Servo
 * driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These driver use I2C to communicate, 2 pins are required to interface.
 *  For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

/*!
 *  @brief  Class that stores state and functions for interacting with PCA9685
 * PWM chip
 */
class RDRServoDriver {
public:
  RDRServoDriver(const uint8_t addr, TwoWire &i2c, uint8_t baseServo, uint8_t numServos);
  ~RDRServoDriver();

  void begin(uint8_t prescale = 0);
  void reset();
  void sleep();
  void wakeup();
  void setExtClk(uint8_t prescale);
  void setPWMFreq(float freq);
  void setOutputMode(bool totempole);
  uint8_t readPrescale(void);

  void setPWM(uint8_t num, uint16_t on, uint16_t off);
  void commitPWM();

  void setOscillatorFrequency(uint32_t freq);
  uint32_t getOscillatorFrequency(void);

private:
  uint8_t m_i2cAddr;
  TwoWire* m_i2c;

  uint8_t m_baseServo;
  uint8_t m_numServos;
  bool m_bAnyChanged;

  uint32_t m_oscillator_freq;
  uint8_t* m_pwmRegisterState;

  uint8_t read8(uint8_t addr);
  void write8(uint8_t addr, uint8_t d);  
};
