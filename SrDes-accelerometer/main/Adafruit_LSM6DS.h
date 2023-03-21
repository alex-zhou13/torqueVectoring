
//  #ifndef _ADAFRUIT_LSM6DS_H
//  #define _ADAFRUIT_LSM6DS_H
 
// //  #include "Arduino.h"
// //  #include <Adafruit_BusIO_Register.h>
// //  #include <Adafruit_I2CDevice.h>
// //  #include <Adafruit_Sensor.h>
// //  #include <Wire.h>
 
//  #define LSM6DS_I2CADDR_DEFAULT 0x6A 
 
//  #define LSM6DS_FUNC_CFG_ACCESS 0x1 
//  #define LSM6DS_INT1_CTRL 0x0D      
//  #define LSM6DS_INT2_CTRL 0x0E      
//  #define LSM6DS_WHOAMI 0x0F         
//  #define LSM6DS_CTRL1_XL 0x10       
//  #define LSM6DS_CTRL2_G 0x11        
//  #define LSM6DS_CTRL3_C 0x12        
//  #define LSM6DS_CTRL8_XL 0x17       
//  #define LSM6DS_CTRL10_C 0x19       
//  #define LSM6DS_WAKEUP_SRC 0x1B     
//  #define LSM6DS_STATUS_REG 0X1E     
//  #define LSM6DS_OUT_TEMP_L 0x20     
//  #define LSM6DS_OUTX_L_G 0x22       
//  #define LSM6DS_OUTX_L_A 0x28       
//  #define LSM6DS_STEPCOUNTER 0x4B    
//  #define LSM6DS_TAP_CFG 0x58        
//  #define LSM6DS_WAKEUP_THS                                                      
//    0x5B 
//  #define LSM6DS_WAKEUP_DUR                                                      
//    0x5C 
//  #define LSM6DS_MD1_CFG 0x5E 
 
 
//  typedef enum data_rate {
//    LSM6DS_RATE_SHUTDOWN,
//    LSM6DS_RATE_12_5_HZ,
//    LSM6DS_RATE_26_HZ,
//    LSM6DS_RATE_52_HZ,
//    LSM6DS_RATE_104_HZ,
//    LSM6DS_RATE_208_HZ,
//    LSM6DS_RATE_416_HZ,
//    LSM6DS_RATE_833_HZ,
//    LSM6DS_RATE_1_66K_HZ,
//    LSM6DS_RATE_3_33K_HZ,
//    LSM6DS_RATE_6_66K_HZ,
//  } lsm6ds_data_rate_t;
 
//  typedef enum accel_range {
//    LSM6DS_ACCEL_RANGE_2_G,
//    LSM6DS_ACCEL_RANGE_16_G,
//    LSM6DS_ACCEL_RANGE_4_G,
//    LSM6DS_ACCEL_RANGE_8_G
//  } lsm6ds_accel_range_t;
 
//  typedef enum gyro_range {
//    LSM6DS_GYRO_RANGE_125_DPS = 0b0010,
//    LSM6DS_GYRO_RANGE_250_DPS = 0b0000,
//    LSM6DS_GYRO_RANGE_500_DPS = 0b0100,
//    LSM6DS_GYRO_RANGE_1000_DPS = 0b1000,
//    LSM6DS_GYRO_RANGE_2000_DPS = 0b1100,
//    ISM330DHCX_GYRO_RANGE_4000_DPS = 0b0001
//  } lsm6ds_gyro_range_t;
 
//  typedef enum hpf_range {
//    LSM6DS_HPF_ODR_DIV_50 = 0,
//    LSM6DS_HPF_ODR_DIV_100 = 1,
//    LSM6DS_HPF_ODR_DIV_9 = 2,
//    LSM6DS_HPF_ODR_DIV_400 = 3,
//  } lsm6ds_hp_filter_t;
 
//  class Adafruit_LSM6DS;
 
//  class Adafruit_LSM6DS_Temp : public Adafruit_Sensor {
//  public:
//    Adafruit_LSM6DS_Temp(Adafruit_LSM6DS *parent) { _theLSM6DS = parent; }
//    bool getEvent(sensors_event_t *);
//    void getSensor(sensor_t *);
 
//  private:
//    int _sensorID = 0x6D0;
//    Adafruit_LSM6DS *_theLSM6DS = NULL;
//  };
 
//  class Adafruit_LSM6DS_Accelerometer : public Adafruit_Sensor {
//  public:
//    Adafruit_LSM6DS_Accelerometer(Adafruit_LSM6DS *parent) {
//      _theLSM6DS = parent;
//    }
//    bool getEvent(sensors_event_t *);
//    void getSensor(sensor_t *);
 
//  private:
//    int _sensorID = 0x6D1;
//    Adafruit_LSM6DS *_theLSM6DS = NULL;
//  };
 
//  class Adafruit_LSM6DS_Gyro : public Adafruit_Sensor {
//  public:
//    Adafruit_LSM6DS_Gyro(Adafruit_LSM6DS *parent) { _theLSM6DS = parent; }
//    bool getEvent(sensors_event_t *);
//    void getSensor(sensor_t *);
 
//  private:
//    int _sensorID = 0x6D2;
//    Adafruit_LSM6DS *_theLSM6DS = NULL;
//  };
 
//  class Adafruit_LSM6DS {
//  public:
//    Adafruit_LSM6DS();
//    virtual ~Adafruit_LSM6DS();
 
//    bool begin_I2C(uint8_t i2c_addr = LSM6DS_I2CADDR_DEFAULT,
//                   TwoWire *wire = &Wire, int32_t sensorID = 0);
 
//    bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI, int32_t sensorID = 0,
//                   uint32_t frequency = 1000000);
//    bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
//                   int8_t mosi_pin, int32_t sensorID = 0,
//                   uint32_t frequency = 1000000);
 
//    bool getEvent(sensors_event_t *accel, sensors_event_t *gyro,
//                  sensors_event_t *temp);
 
//    lsm6ds_data_rate_t getAccelDataRate(void);
//    void setAccelDataRate(lsm6ds_data_rate_t data_rate);
 
//    lsm6ds_accel_range_t getAccelRange(void);
//    void setAccelRange(lsm6ds_accel_range_t new_range);
 
//    lsm6ds_data_rate_t getGyroDataRate(void);
//    void setGyroDataRate(lsm6ds_data_rate_t data_rate);
 
//    lsm6ds_gyro_range_t getGyroRange(void);
//    void setGyroRange(lsm6ds_gyro_range_t new_range);
 
//    void reset(void);
//    void configIntOutputs(bool active_low, bool open_drain);
//    void configInt1(bool drdy_temp, bool drdy_g, bool drdy_xl,
//                    bool step_detect = false, bool wakeup = false);
//    void configInt2(bool drdy_temp, bool drdy_g, bool drdy_xl);
//    void highPassFilter(bool enabled, lsm6ds_hp_filter_t filter);
 
//    void enableWakeup(bool enable, uint8_t duration = 0, uint8_t thresh = 20);
//    bool awake(void);
//    bool shake(void);
 
//    void enablePedometer(bool enable);
//    void resetPedometer(void);
//    uint16_t readPedometer(void);
 
//    // Arduino compatible API
//    int readAcceleration(float &x, float &y, float &z);
//    float accelerationSampleRate(void);
//    int accelerationAvailable(void);
 
//    int readGyroscope(float &x, float &y, float &z);
//    float gyroscopeSampleRate(void);
//    int gyroscopeAvailable(void);
 
//    int16_t rawAccX, 
//        rawAccY,     
//        rawAccZ,     
//        rawTemp,     
//        rawGyroX,    
//        rawGyroY,    
//        rawGyroZ;    
 
//    float temperature, 
//        accX,          
//        accY,          
//        accZ,          
//        gyroX,         
//        gyroY,         
//        gyroZ;         
 
//    Adafruit_Sensor *getTemperatureSensor(void);
//    Adafruit_Sensor *getAccelerometerSensor(void);
//    Adafruit_Sensor *getGyroSensor(void);
 
//  protected:
//    uint8_t chipID(void);
//    uint8_t status(void);
//    virtual void _read(void);
//    virtual bool _init(int32_t sensor_id);
 
//    uint16_t _sensorid_accel, 
//        _sensorid_gyro,       
//        _sensorid_temp;       
 
//    Adafruit_I2CDevice *i2c_dev = NULL; 
//    Adafruit_SPIDevice *spi_dev = NULL; 
 
//    float temperature_sensitivity =
//        256.0; 
//    Adafruit_LSM6DS_Temp *temp_sensor = NULL; 
//    Adafruit_LSM6DS_Accelerometer *accel_sensor =
//        NULL;                                 
//    Adafruit_LSM6DS_Gyro *gyro_sensor = NULL; 
 
//    lsm6ds_accel_range_t accelRangeBuffered = LSM6DS_ACCEL_RANGE_2_G;
//    lsm6ds_gyro_range_t gyroRangeBuffered = LSM6DS_GYRO_RANGE_250_DPS;
 
//  private:
//    friend class Adafruit_LSM6DS_Temp; 
//    friend class Adafruit_LSM6DS_Accelerometer; 
//    friend class Adafruit_LSM6DS_Gyro; 
 
//    void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
//    void fillAccelEvent(sensors_event_t *accel, uint32_t timestamp);
//    void fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp);
//  };
 
//  #endif