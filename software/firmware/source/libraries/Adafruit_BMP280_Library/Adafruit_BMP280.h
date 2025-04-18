/***************************************************************************
  This is a library for the BMP280 pressure sensor

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __BMP280_H__
#define __BMP280_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#if !(defined(ESP8266)             || defined(ESP32) || \
      defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2) || \
      defined(ARDUINO_ARCH_STM32)  || defined(__ASR6501__) || \
      defined(ARDUINO_ARCH_NRF52))
#include <Adafruit_Sensor.h>
#endif

#ifdef __AVR_ATtiny85__
 #include "TinyWireM.h"
 #define Wire TinyWireM
#else
 #include <Wire.h>
#endif

/*=========================================================================
    I2C ADDRESS/BITS/SETTINGS
    -----------------------------------------------------------------------*/
    #define BMP280_ADDRESS                (0x77)
    #define BMP280_CHIPID                 (0x58)
    #define BME280_CHIPID                 (0x60)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    enum
    {
      BMP280_REGISTER_DIG_T1              = 0x88,
      BMP280_REGISTER_DIG_T2              = 0x8A,
      BMP280_REGISTER_DIG_T3              = 0x8C,

      BMP280_REGISTER_DIG_P1              = 0x8E,
      BMP280_REGISTER_DIG_P2              = 0x90,
      BMP280_REGISTER_DIG_P3              = 0x92,
      BMP280_REGISTER_DIG_P4              = 0x94,
      BMP280_REGISTER_DIG_P5              = 0x96,
      BMP280_REGISTER_DIG_P6              = 0x98,
      BMP280_REGISTER_DIG_P7              = 0x9A,
      BMP280_REGISTER_DIG_P8              = 0x9C,
      BMP280_REGISTER_DIG_P9              = 0x9E,

      BMP280_REGISTER_CHIPID             = 0xD0,
      BMP280_REGISTER_VERSION            = 0xD1,
      BMP280_REGISTER_SOFTRESET          = 0xE0,

      BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

      BMP280_REGISTER_CONTROL            = 0xF4,
      BMP280_REGISTER_CONFIG             = 0xF5,
      BMP280_REGISTER_PRESSUREDATA       = 0xF7,
      BMP280_REGISTER_TEMPDATA           = 0xFA,
    };

/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct
    {
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;

      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;
    } bmp280_calib_data;
/*=========================================================================*/

/*
class Adafruit_BMP280_Unified : public Adafruit_Sensor
{
  public:
    Adafruit_BMP280_Unified(int32_t sensorID = -1);

    bool  begin(uint8_t addr = BMP280_ADDRESS, uint8_t chipid = BMP280_CHIPID);
    void  getTemperature(float *temp);
    void  getPressure(float *pressure);
    float pressureToAltitude(float seaLevel, float atmospheric, float temp);
    float seaLevelForAltitude(float altitude, float atmospheric, float temp);
    void  getEvent(sensors_event_t*);
    void  getSensor(sensor_t*);

  private:
    uint8_t   _i2c_addr;
    int32_t   _sensorID;
};

*/

class Adafruit_BMP280
{
  public:
    Adafruit_BMP280();
    Adafruit_BMP280(int8_t cspin);
    Adafruit_BMP280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);

    bool  setWire(TwoWire *twoWire = &Wire);
    bool  begin(uint8_t addr = BMP280_ADDRESS, uint8_t chipid = BMP280_CHIPID);
    float readTemperature(void);
    float readPressure(void);
    float readAltitude(float seaLevelhPa = 1013.25);

    



  //CK: enums for BMP280 settings
  /** Oversampling rate for the sensor. */
  enum sensor_sampling {
    /** No over-sampling. */
    SAMPLING_NONE = 0x00,
    /** 1x over-sampling. */
    SAMPLING_X1 = 0x01,
    /** 2x over-sampling. */
    SAMPLING_X2 = 0x02,
    /** 4x over-sampling. */
    SAMPLING_X4 = 0x03,
    /** 8x over-sampling. */
    SAMPLING_X8 = 0x04,
    /** 16x over-sampling. */
    SAMPLING_X16 = 0x05
  };

  /** Filtering level for sensor data. */
  enum sensor_filter {
    /** No filtering. */
    FILTER_OFF = 0x00,
    /** 2x filtering. */
    FILTER_X2 = 0x01,
    /** 4x filtering. */
    FILTER_X4 = 0x02,
    /** 8x filtering. */
    FILTER_X8 = 0x03,
    /** 16x filtering. */
    FILTER_X16 = 0x04
  };

  /** Standby duration in ms */
  enum standby_duration {
    /** 1 ms standby. */
    STANDBY_MS_1 = 0x00,
    /** 62.5 ms standby. */
    STANDBY_MS_63 = 0x01,
    /** 125 ms standby. */
    STANDBY_MS_125 = 0x02,
    /** 250 ms standby. */
    STANDBY_MS_250 = 0x03,
    /** 500 ms standby. */
    STANDBY_MS_500 = 0x04,
    /** 1000 ms standby. */
    STANDBY_MS_1000 = 0x05,
    /** 2000 ms standby. */
    STANDBY_MS_2000 = 0x06,
    /** 4000 ms standby. */
    STANDBY_MS_4000 = 0x07
  };

  /** Operating mode for the sensor. */
  enum sensor_mode {
    /** Sleep mode. */
    MODE_SLEEP = 0x00,
    /** Forced mode. */
    MODE_FORCED = 0x01,
    /** Normal mode. */
    MODE_NORMAL = 0x03,
    /** Software reset. */
    MODE_SOFT_RESET_CODE = 0xB6
  };

  void setSampling(sensor_mode mode = MODE_NORMAL,
    sensor_sampling tempSampling = SAMPLING_X1,
    sensor_sampling pressSampling = SAMPLING_X1,
    sensor_filter filter = FILTER_OFF,
    standby_duration duration = STANDBY_MS_1);



    
  private:

    void readCoefficients(void);
    uint8_t spixfer(uint8_t x);

    void      write8(byte reg, byte value);
    uint8_t   read8(byte reg);
    uint16_t  read16(byte reg);
    uint32_t  read24(byte reg);
    int16_t   readS16(byte reg);
    uint16_t  read16_LE(byte reg); // little endian
    int16_t   readS16_LE(byte reg); // little endian

    TwoWire *_i2c; // programmable I2C wire
    
    uint8_t   _i2caddr;
    int32_t   _sensorID;
    int32_t t_fine;

    int8_t _cs, _mosi, _miso, _sck;

    bmp280_calib_data _bmp280_calib;


    


    struct ctrl_meas {
      /** Initialize to power-on-reset state */
      ctrl_meas()
          : osrs_t(SAMPLING_X2), osrs_p(SAMPLING_X1), mode(MODE_NORMAL) {}
      /** Temperature oversampling. */
      unsigned int osrs_t : 3;
      /** Pressure oversampling. */
      unsigned int osrs_p : 3;
      /** Device mode */
      unsigned int mode : 2;
      /** Used to retrieve the assembled ctrl_meas register's byte value. */
      unsigned int get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
    };
    
    struct config {
      /** Initialize to power-on-reset state */
      config() : t_sb(STANDBY_MS_1), filter(FILTER_X16), none(0), spi3w_en(0) {}
      /** Inactive duration (standby time) in normal mode */
      unsigned int t_sb : 3;
      /** Filter settings */
      unsigned int filter : 3;
      /** Unused - don't set */
      unsigned int none : 1;
      /** Enables 3-wire SPI */
      unsigned int spi3w_en : 1;
      /** Used to retrieve the assembled config register's byte value. */
      unsigned int get() { return (t_sb << 5) | (filter << 2) | spi3w_en; }
    };


    //CK: private variables for BMP280 settings
    ctrl_meas _measReg;
    config _configReg;

    

};

#endif
