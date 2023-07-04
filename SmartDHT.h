#ifndef _SMART_DHT_H
#define _SMART_DHT_H
/**
 * FILE: SmartDHT.h
 * AUTHOR: Mirko Falchetto
 * VERSION: 1.0.0
 * PURPOSE: DHT Temperature & Humidity Sensor library for Arduino
 *      URL: https://github.com/mirkoflchtt/SmartDHT


 * DHT PIN layout from left to right
 * =================================
 * FRONT  :  DESCRIPTION
 *   pin 1  :  VCC
 *   pin 2  :  DATA
 *   pin 3  :  Not Connected
 *   pin 4  :  GND
**/

#include <Arduino.h>

#define DHT11                               (11)
#define DHT22                               (22)
#define SONOFF_SI7021                       (70)

#define SMARTDHT_LIB_VERSION                (F("1.0.0"))

//  optionally detect out of range values.
//  occurs seldom so not enabled by default.
//  #define SMARTDHT_VALUE_OUT_OF_RANGE
#define SMARTDHT_HUMIDITY_OUT_OF_RANGE      (-100)
#define SMARTDHT_TEMPERATURE_OUT_OF_RANGE   (-101)

//  allows to overrule SMARTDHT_INVALID_VALUE e.g. to prevent spike in graphs.
#define SMARTDHT_INVALID_VALUE              (NAN)

typedef enum {
  SMARTDHT_OK                               = 0,
  SMARTDHT_ERROR_CHECKSUM                   = -1,
  SMARTDHT_ERROR_BIT_SHIFT                  = -2,
  SMARTDHT_ERROR_SENSOR_NOT_READY           = -3,
  SMARTDHT_ERROR_TIMEOUT_A                  = -4,
  SMARTDHT_ERROR_TIMEOUT_C                  = -5,
  SMARTDHT_ERROR_TIMEOUT_D                  = -6,
  SMARTDHT_ERROR_TIMEOUT_B                  = -7,
  SMARTDHT_WAITING_FOR_READ                 = -8,
} SmartDHTError;


class SmartDHT
{
public:

  SmartDHT(const uint8_t pin, const uint8_t type=0);

  uint8_t  getType(void);
  void     setType(const uint8_t type);

  //  resets all internals to construction time
  //  might help to reset a sensor behaving badly..
  void     reset(void);
  int      read(void);

  float    convertCtoF(const float c);
  float    convertFtoC(const float f);

  float    computeHeatIndex(const float t, const float h, const bool isFahrenheit=false);

  //  preferred interface
  float    readHumidity(void);
  float    readTemperature(const bool isFahrenheit=false);

  //  adding offsets works well in normal range
  //  might introduce under- or overflow at the ends of the sensor range
  void     setHumOffset(const float offset)         { _humOffset = offset; };
  void     setTempOffset(const float offset)        { _tempOffset = offset; };
  float    getHumOffset(void)                       { return _humOffset; };
  float    getTempOffset(void)                      { return _tempOffset; };
  
  bool     getWaitForReading(void)                  { return _waitForRead; };
  void     setWaitForReading(const bool b)          { _waitForRead = b; };

  //  set readDelay to 0 will reset to datasheet values
  uint16_t getReadDelay(void)                       { return _readDelay; };
  void     setReadDelay(const uint16_t rd = 0)      { _readDelay = rd; };

  //  minimal support for low power applications.
  //  after powerUp one must wait up to two seconds.
  void     powerUp(void);
  void     powerDown(void);

  //  suppress error values of -999 => check return value of read() instead
  bool     getSuppressError(void)                   { return _suppressError; };
  void     setSuppressError(const bool b)           { _suppressError = b; };

  bool     loop(void);

private:
  const uint8_t  _dataPin;
  uint8_t  _type;
  uint32_t _wakeupDelay;
  float    _humOffset;
  float    _tempOffset;
  float    _humidity;
  float    _temperature;
  uint32_t _lastRead;
  bool     _waitForRead;
  bool     _suppressError;
  uint16_t _readDelay;

  SmartDHTError _read(void);
  SmartDHTError _readSensor(uint8_t* data);
  SmartDHTError _readExit(const SmartDHTError error);

  bool     _waitFor(const uint8_t state, const uint32_t timeout);
};


# endif   /* _SMART_DHT_H */

