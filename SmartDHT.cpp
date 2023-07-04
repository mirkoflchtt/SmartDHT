/**
 * FILE: SmartDHT.cpp
 * AUTHOR: Mirko Falchetto
 * VERSION: 1.0.0
 * PURPOSE: DHT Temperature & Humidity Sensor library for Arduino
 *       URL: https://github.com/mirkoflchtt/SmartDHT
 *
 * HISTORY: see changelog.md
**/
#include "SmartDHT.h"

#define SMARTDHT_MAX_CYCLES                   (90)

//  bits are timing based (datasheet)
//  26-28us ==> 0
//  70 us   ==> 1
//  See https://github.com/RobTillaart/DHTNew/issues/11
#define SMARTDHT_BIT_THRESHOLD                (50)

/* these defines are not for user to adjust (microseconds) */
#define SMARTDHT_DHT11_WAKEUP                 (18 * 1100UL)
#define SMARTDHT_DHT_WAKEUP                   (1 * 1100UL)
#define SMARTDHT_SI7021_WAKEUP                (500UL)

/*
 * READ_DELAY for blocking read
 *   datasheet: DHT11 = 1000 and DHT22 = 2000
 *   use setReadDelay() to overrule (at own risk)
 *   as individual sensors can be read faster.
*/
#define SMARTDHT_DHT11_READ_DELAY             (1000)
#define SMARTDHT_DHT22_READ_DELAY             (2000)
#define SMARTDHT_SI7021_READ_DELAY            (2000)

#if defined(__AVR__)
#define SMARTDHT_DISABLE_IRQ                  /* SMARTDHT_DISABLE_IRQ */
#define SMARTDHT_ENABLE_IRQ                   /* SMARTDHT_ENABLE_IRQ */
#elif defined (ESP32)
#define SMARTDHT_DISABLE_IRQ                  portDISABLE_INTERRUPTS();
#define SMARTDHT_ENABLE_IRQ                   portENABLE_INTERRUPTS();
#else
#define SMARTDHT_DISABLE_IRQ                  noInterrupts();
#define SMARTDHT_ENABLE_IRQ                   interrupts();
#endif


/*****************************************************************************/
SmartDHT::SmartDHT(uint8_t pin, uint8_t type):
_dataPin(pin),
_type(type),
_wakeupDelay(0),
_humOffset(0.0f),
_tempOffset(0.0f),
_humidity(0.0f),
_temperature(0.0f),
_lastRead(0),
_waitForRead(false),
_suppressError(false),
_readDelay(0)
{
  setType(type);
  reset();
};


void SmartDHT::reset(void)
{
  //  Data-bus's free status is high voltage level.
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);

  _wakeupDelay   = 0;
  _type          = 0;
  _humOffset     = 0.0f;
  _tempOffset    = 0.0f;
  _humidity      = 0.0f;
  _temperature   = 0.0f;
  _lastRead      = 0;
  _waitForRead   = false;
  _suppressError = false;
}


uint8_t SmartDHT::getType(void)
{
  if (_type == 0) {
    read();
  };
  return _type;
}


void SmartDHT::setType(const uint8_t type)
{
  switch (type) {
    case 0:
    case DHT11:
      _type = type;
      _wakeupDelay = SMARTDHT_DHT11_WAKEUP;
      _readDelay = SMARTDHT_DHT11_READ_DELAY;
      break;
    case DHT22:
      _type = type;
      _wakeupDelay = SMARTDHT_DHT_WAKEUP;
      _readDelay = SMARTDHT_DHT22_READ_DELAY;
      break;
    //  experimental
    case SONOFF_SI7021:
      _type = type;
      _wakeupDelay = SMARTDHT_SI7021_WAKEUP;
      _readDelay = SMARTDHT_SI7021_READ_DELAY;
      break;
    default:
      break;
  }
}


float SmartDHT::convertCtoF(const float c)
{
  return c * 1.8f + 32.0f;
}


float SmartDHT::convertFtoC(const float f)
{
  return (f - 32.0f) * 0.55555f;
}


float SmartDHT::computeHeatIndex(const float t, const float h, const bool isFahrenheit)
{
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  const float temp = (isFahrenheit) ? t : convertCtoF(t);
  float hi = 0.5f * (temp + 61.0f + ((temp - 68.0f) * 1.2f) + (h * 0.094f));

  if (hi > 79) {
    hi = -42.379f +
             2.04901523f * temp +
            10.14333127f * h +
            -0.22475541f * temp*h +
            -0.00683783f * pow(temp, 2) +
            -0.05481717f * pow(h, 2) +
             0.00122874f * pow(temp, 2) * h +
             0.00085282f * temp*pow(h, 2) +
            -0.00000199f * pow(temp, 2) * pow(h, 2);

    if ((h < 13.0f) && (temp >= 80.0f) && (temp <= 112.0f)) {
      hi -= ((13.0f - h) * 0.25f) * sqrt((17.0f - abs(temp - 95.0f)) * 0.05882f);
    }
    else if ((h > 85.0f) && (temp >= 80.0f) && (temp <= 87.0f)) {
      hi += ((h - 85.0f) * 0.1f) * ((87.0f - temp) * 0.2f);
    }
  }

  return (isFahrenheit) ? hi : convertFtoC(hi);
}


float SmartDHT::readHumidity(void) {
  return _humidity;
}


float SmartDHT::readTemperature(const bool isFahrenheit)
{
  return (isFahrenheit) ? convertCtoF(_temperature) : _temperature;
}


/*
//  return values:
//  SMARTDHT_OK
//  SMARTDHT_WAITING_FOR_READ
//  SMARTDHT_ERROR_CHECKSUM
//  SMARTDHT_ERROR_BIT_SHIFT
//  SMARTDHT_ERROR_SENSOR_NOT_READY
//  SMARTDHT_ERROR_TIMEOUT_A
//  SMARTDHT_ERROR_TIMEOUT_B
//  SMARTDHT_ERROR_TIMEOUT_C
//  SMARTDHT_ERROR_TIMEOUT_D
*/
int SmartDHT::read(void)
{
  while (millis() - _lastRead < _readDelay)
  {
    if (!_waitForRead) {
      return SMARTDHT_WAITING_FOR_READ;
    }
    yield();
  }
  if (_type != 0) {
    return _read();
  }

  /* AUTODETECT: make sure sensor had time to wake up. */
  // while (millis() < 1000);

  /* NOTE: cannot differentiate between type 23 and 22 */
  _type = DHT22;
  _wakeupDelay = SMARTDHT_DHT_WAKEUP;
  int rv = _read();
  if (rv == SMARTDHT_OK) return rv;

  _type = DHT11;
  _wakeupDelay = SMARTDHT_DHT11_WAKEUP;
  rv = _read();
  if (rv == SMARTDHT_OK) return rv;

  //  experimental 0.4.14
  _type = SONOFF_SI7021;
  _wakeupDelay = SMARTDHT_SI7021_WAKEUP;
  rv = _read();
  if (rv == SMARTDHT_OK) return rv;

  _type = 0; // retry next time
  return rv;
}


//  return values:
//  SMARTDHT_OK
//  SMARTDHT_ERROR_CHECKSUM
//  SMARTDHT_ERROR_BIT_SHIFT
//  SMARTDHT_ERROR_SENSOR_NOT_READY
//  SMARTDHT_ERROR_TIMEOUT_A
//  SMARTDHT_ERROR_TIMEOUT_B
//  SMARTDHT_ERROR_TIMEOUT_C
//  SMARTDHT_ERROR_TIMEOUT_D
SmartDHTError SmartDHT::_read(void)
{
  uint8_t _bits[5];

  //  READ VALUES
  const SmartDHTError rv = _readSensor(_bits);//  bits are timing based (datasheet)

  if (rv != SMARTDHT_OK) {
    if (_suppressError == false) {
      _humidity    = SMARTDHT_INVALID_VALUE;
      _temperature = SMARTDHT_INVALID_VALUE;
    }
    return rv;        //  propagate error value
  }
  _lastRead = millis();

  _humidity = _humOffset;
  _temperature = _tempOffset;

  switch (_type)
  {
    case DHT11:
    {
      _humidity    += _bits[0] + _bits[1] * 0.1f;
      _temperature += _bits[2] + _bits[3] * 0.1f;
    } break;
    //  DHT22, DHT33, DHT44, compatible + Si7021
    case DHT22:
    case SONOFF_SI7021:
    {
      const int16_t t = ((_bits[2] & 0x7F) * 256 + _bits[3]);
      _humidity += (((int16_t)_bits[0]) * 256 + _bits[1]) * 0.1f;
      _temperature += (t == 0) ? 0.0f : (t * ((_bits[2] & 0x80) ? -0.1f: 0.1f));
    } break;
    default:
    {
      _humidity = SMARTDHT_INVALID_VALUE;
      _temperature = SMARTDHT_INVALID_VALUE;
    } break;
  }

#ifdef SMARTDHT_VALUE_OUT_OF_RANGE
  if (_humidity > 100) {
    return SMARTDHT_HUMIDITY_OUT_OF_RANGE;
  } else if ((_temperature < -40) || (_temperature > 80)) {
    return SMARTDHT_TEMPERATURE_OUT_OF_RANGE;
  }
#endif

  return SMARTDHT_OK;
}


void SmartDHT::powerUp(void)
{
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  //  do a dummy read to sync the sensor
  read();
};


void SmartDHT::powerDown(void)
{
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, LOW);
}

bool SmartDHT::loop(void)
{
  return true;
}

/////////////////////////////////////////////////////
//
//  PRIVATE
//

SmartDHTError SmartDHT::_readExit(const SmartDHTError error)
{
  SMARTDHT_ENABLE_IRQ

  //  Data-bus's free status is high voltage level.
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);

  return error;
}

//  return values:
//  SMARTDHT_OK
//  SMARTDHT_ERROR_CHECKSUM
//  SMARTDHT_ERROR_BIT_SHIFT
//  SMARTDHT_ERROR_SENSOR_NOT_READY
//  SMARTDHT_ERROR_TIMEOUT_A
//  SMARTDHT_ERROR_TIMEOUT_B
//  SMARTDHT_ERROR_TIMEOUT_C
//  SMARTDHT_ERROR_TIMEOUT_D
SmartDHTError SmartDHT::_readSensor(uint8_t* data)
{
  //  INIT BUFFERVAR TO RECEIVE DATA
  uint8_t i;

  //  CLEAR DATA BUFFER
  for (i=0; i<5; i++) {
    data[i] = 0;
  }

  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  delayMicroseconds(10);

  //  REQUEST SAMPLE - SEND WAKEUP TO SENSOR
  digitalWrite(_dataPin, LOW);

  //  HANDLE SI7021 separately (see #79)
  if (_type == SONOFF_SI7021) {
    delayMicroseconds(SMARTDHT_SI7021_WAKEUP);  //  hardcoded for now
  } else {
    //  WAKE UP - add 10% extra for timing inaccuracies in sensor.
    const uint32_t startWakeup = micros();
    do
    {
      //  HANDLE PENDING IRQ
      yield();
      //  180 gives good wakeup delay on UNO for DHT22 / DHT11 (issue #72)
      delayMicroseconds(180UL);
    }
    while ((micros()-startWakeup) < _wakeupDelay);
  }

  //  DHT22  (and others including Si7021)
  //  SENSOR PULLS LOW after 20-40 us  => if stays HIGH ==> device not ready
  //  timeout is 20 us less due to delay() above
  //  DHT11
  //  SENSOR PULLS LOW after 6000-10000 us
  const uint32_t WAITFORSENSOR = (_type == DHT11) ? 15000UL : 50UL;

  /* HOST GIVES CONTROL TO SENSOR */
  digitalWrite(_dataPin, HIGH);
  delayMicroseconds(4);

  /* Entering critical section */
  pinMode(_dataPin, INPUT_PULLUP);
  SMARTDHT_DISABLE_IRQ

  if (_waitFor(LOW, WAITFORSENSOR)) {
    return _readExit(SMARTDHT_ERROR_SENSOR_NOT_READY);
  }
  //  SENSOR STAYS LOW for ~80 us => or TIMEOUT
  if (_waitFor(HIGH, SMARTDHT_MAX_CYCLES)) {
    return _readExit(SMARTDHT_ERROR_TIMEOUT_A);
  }
  //  SENSOR STAYS HIGH for ~80 us => or TIMEOUT
  if (_waitFor(LOW, SMARTDHT_MAX_CYCLES)) {
    return _readExit(SMARTDHT_ERROR_TIMEOUT_B);
  }

  //  SENSOR HAS NOW SEND ACKNOWLEDGE ON WAKEUP
  //  NOW IT SENDS THE BITS

  //  READ THE OUTPUT - 40 BITS => 5 BYTES
  for (i=0; i<5; i++) {
    for (uint8_t mask=0x80; mask; mask>>=1) {
      //  EACH BIT START WITH ~50 us LOW
      if (_waitFor(HIGH, SMARTDHT_MAX_CYCLES)) {
        /* Most critical timeout */
        return _readExit(SMARTDHT_ERROR_TIMEOUT_C);
      }

      /*
        DURATION OF HIGH DETERMINES 0 or 1
          26-28 us  ==>  0
          70 us  ==>  1
      */
      const uint32_t start = micros();
      if (_waitFor(LOW, SMARTDHT_MAX_CYCLES)) {
        return _readExit(SMARTDHT_ERROR_TIMEOUT_D);
      }

      data[i] |= ((micros() - start) > SMARTDHT_BIT_THRESHOLD) ? mask : 0x0;
    }
  }

  //  After 40 bits the sensor pulls the line LOW for 50 us
  //  No functional need to wait for this one
  //  if (_waitFor(HIGH, 60)) return SMARTDHT_ERROR_TIMEOUT_E;

  //  CATCH RIGHTSHIFT BUG ESP (only 1 single bit shift)
  //  humidity is maximum 1000 = 0x03E8 for DHT22 and 0x6400 for DHT11
  //  so most significant bit may never be set.
  const SmartDHTError ret = _readExit((data[0] & 0x80) ? SMARTDHT_ERROR_BIT_SHIFT : SMARTDHT_OK);

  if (ret == SMARTDHT_OK) {
    //  TEST CHECKSUM
    const uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if (data[4] != checksum) {
      return SMARTDHT_ERROR_CHECKSUM;
    }
  }

  return ret;
}

/**
 * returns true  if timeout has passed.
 * returns false if timeout is not reached and state is seen.
 **/
bool SmartDHT::_waitFor(const uint8_t state, const uint32_t timeout)
{
  const uint32_t start = micros();
  uint32_t  count = 0;
  while ((micros() - start) < timeout) {
    delayMicroseconds(1);         // less # reads ==> minimizes # glitch reads
    if (digitalRead(_dataPin) == state) {
      count++;
      if (count == 8) {
        return false;   //  requested state seen count times
      }
    }
  }
  return true;
}


#undef SMARTDHT_DISABLE_IRQ
#undef SMARTDHT_ENABLE_IRQ
#undef SMARTDHT_DHT11_WAKEUP
#undef SMARTDHT_DHT_WAKEUP
#undef SMARTDHT_SI7021_WAKEUP
#undef SMARTDHT_MAX_CYCLES
#undef SMARTDHT_BIT_THRESHOLD
#undef SMARTDHT_DHT11_READ_DELAY
#undef SMARTDHT_DHT22_READ_DELAY
#undef SMARTDHT_SI7021_READ_DELAY

