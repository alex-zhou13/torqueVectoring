/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "spi_flash_mmap.h"
#include "./Adafruit_ISM330DHCX.h"

// static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)

#define UART UART_NUM_2

int num = 0;

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          9   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          8   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// ISM330DHC
#define SLAVE_ADDR                         0x35 // 110101xb
#define ACCEL_ADDR                         0x6A

 typedef enum accel_range {
   LSM6DS_ACCEL_RANGE_2_G,
   LSM6DS_ACCEL_RANGE_16_G,
   LSM6DS_ACCEL_RANGE_4_G,
   LSM6DS_ACCEL_RANGE_8_G
 } lsm6ds_accel_range_t;

  typedef enum data_rate {
   LSM6DS_RATE_SHUTDOWN,
   LSM6DS_RATE_12_5_HZ,
   LSM6DS_RATE_26_HZ,
   LSM6DS_RATE_52_HZ,
   LSM6DS_RATE_104_HZ,
   LSM6DS_RATE_208_HZ,
   LSM6DS_RATE_416_HZ,
   LSM6DS_RATE_833_HZ,
   LSM6DS_RATE_1_66K_HZ,
   LSM6DS_RATE_3_33K_HZ,
   LSM6DS_RATE_6_66K_HZ,
 } lsm6ds_data_rate_t;

// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  conf.clk_flags = 0;                                       // set to 0 to avoid flag clock/frequency error (https://github.com/espressif/esp-idf/issues/6293)
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK) {printf("- initialized: yes\n");}

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    //printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}

////////////////////////////////////////////////////////////////////////////////

// ISM330DHCX Functions ///////////////////////////////////////////////////////////


// initializes the accelerometer by writing to its configuration registers using I2C commands
// void accel_init()
// {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ACCEL_ADDR << 1 | WRITE_BIT, true);
//     i2c_master_write_byte(cmd, 0x10, true); // set accelerometer to continuous mode
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
// }

// // reads the X, Y, and Z axis acceleration data from the accelerometer by issuing a sequence of I2C read and write commands, and returns the data as 16-bit integers.
// void read_accel_data(int16_t *ax, int16_t *ay, int16_t *az)
// {
//     uint8_t data[6];
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ACCEL_ADDR << 1 | WRITE_BIT, true);
//     i2c_master_write_byte(cmd, 0x01, true); // set register to accelerometer X-axis low byte
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ACCEL_ADDR << 1 | READ_BIT, true);
//     i2c_master_read_byte(cmd, &data[0], true);
//     i2c_master_read_byte(cmd, &data[1], true);
//     i2c_master_read_byte(cmd, &data[2], true);
//     i2c_master_read_byte(cmd, &data[3], true);
//     i2c_master_read_byte(cmd, &data[4], true);
//     i2c_master_read_byte(cmd, &data[5], false);
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     *ax = (int16_t)(data[1] << 8 | data[0]);
//     *ay = (int16_t)(data[3] << 8 | data[2]);
//     *az = (int16_t)(data[5] << 8 | data[4]);
//     // *aa = (int64_t)(data[])
// }

// Write one byte to register; referenced the datasheet
int writeRegister(uint8_t reg, uint8_t data) {
  int ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);  // start
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, true);   // slave address + write
  i2c_master_write_byte(cmd, reg, true);  // register address
  
  i2c_master_write_byte(cmd, data, false); // data
  
  i2c_master_stop(cmd);   // stop
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);  // output the results
  i2c_cmd_link_delete(cmd);

  // printf("Write Reg: %d\t", ret);

  return ret;
}

// Read register; referenced the datasheet
uint8_t readRegister(uint8_t reg) {
  int ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);  // start
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, true);  // slave address + write
  i2c_master_write_byte(cmd, reg, true);  // register address

  i2c_master_start(cmd);  // start
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, true); // slave address + read
  i2c_master_read_byte(cmd, &ret, false);  // nack
  i2c_master_stop(cmd); // stop
  
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS); // outputs the results
  i2c_cmd_link_delete(cmd);

  // printf("Read Reg: %d\t", ret);

  return ret;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
  int ret;

  // can call the read 1-byte register function twice for 2 bytes
  uint8_t b1 = readRegister(reg);
  uint8_t b2 = readRegister(reg + 1);

  ret = ((b2 << 8) | b1);

  return ret;
}

void setRange(lsm6ds_accel_range_t range) {
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister(0x10); // CHECK THIS REGISTER

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(0x10, format); // CHECK THIS REGISTER

}

// lsm6ds_accel_range_t getRange(void) {
//   /* Red the data format register to preserve bits */
//   return (lsm6ds_accel_range_t)(readRegister(0x12) & 0x03);
// }

// lsm6ds_data_rate_t getDataRate(void) {
//   return (lsm6ds_data_rate_t)(readRegister(0x10) & 0x0F);
// }

// function to get acceleration
void getAccel(float * xp, float *yp, float *zp) {
  *xp = read16(0x28) * 10.0F; // or 0x79 for FIFO
  *yp = read16(0x2A) * 0.004F * 9.80665F; // or 0x7B for FIFO
  *zp = read16(0x2C) * 0.004F * 9.80665F; // or 0x7D for FIFO
  printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", *xp, *yp, *zp);
}

// // function to print roll and pitch
// void calcRP(float x, float y, float z){
//   // YOUR CODE HERE; found this from tilt sensing source
//   float roll = atan2(y , z) * 57.3;
//   float pitch = atan2((-1*x) , sqrt(y * y + z * z)) * 57.3;
//   printf("roll: %.2f \t pitch: %.2f \n", roll, pitch);
// }

// Task to continuously poll acceleration and calculate roll and pitch
static void test_ism330dhc() {
  printf("\n>> Polling ISM330DHC\n");
  while (1) {
    float xVal, yVal, zVal;
    getAccel(&xVal, &yVal, &zVal);
    // calcRP(xVal, yVal, zVal);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

////////////////////////////////////////////////////////////////////////////////

// static void accel_task(void *arg) 
// {
//   int16_t ax, ay, az;
//   while (1) {
//     read_accel_data(&ax, &ay, &az);
//     printf("Accelerometer data: X-axis: %d\n", ax);
//     printf("        Y-axis: %d\n", ay);
//     printf("        Z-axis: %d\n", az);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
//   }
// }

void app_main(void)
{
    // Routine
    i2c_master_init();
    i2c_scanner();

    // Disable interrupts
    writeRegister(0x0D, 0); // check

    // set range
    // setRange(LSM6DS_ACCEL_RANGE_2_G);
  //     printf  ("- Range:         +/- ");
  // switch(getRange()) {
  //   case LSM6DS_ACCEL_RANGE_16_G:
  //     printf  ("16 ");
  //     break;
  //   case LSM6DS_ACCEL_RANGE_8_G:
  //     printf  ("8 ");
  //     break;
  //   case LSM6DS_ACCEL_RANGE_4_G:
  //     printf  ("4 ");
  //     break;
  //   case LSM6DS_ACCEL_RANGE_2_G:
  //     printf  ("2 ");
  //     break;
  //   default:
  //     printf  ("?? ");
  //     break;
  // }
  // printf(" g\n");

    // Enable measurements
    writeRegister(0x07, 0x08);

    // accel_init();
    // xTaskCreate(accel_task, "accel_task", 1024*2, NULL, configMAX_PRIORITIES-3, NULL);

    xTaskCreate(test_ism330dhc,"testism330dhc", 4096, NULL, 1, NULL);
}
