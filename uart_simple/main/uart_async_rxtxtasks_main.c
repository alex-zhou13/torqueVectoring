/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include <stdio.h>
#include <math.h>

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)

#define UART UART_NUM_2

int num = 0;

// // Master I2C
// #define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
// #define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
// #define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
// #define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
// #define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
// #define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
// #define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
// #define READ_BIT                           I2C_MASTER_READ  // i2c master read
// #define ACK_CHECK_EN                       true // i2c master will check ack
// #define ACK_CHECK_DIS                      false// i2c master will not check ack
// #define ACK_VAL                            0x00 // i2c ack value
// #define NACK_VAL                           0xFF // i2c nack value

// // ISM330DHC
// #define SLAVE_ADDR                         ADXL343_ADDRESS // 0x53

// // Function to initiate i2c -- note the MSB declaration!
// static void i2c_master_init(){
//   // Debug
//   printf("\n>> i2c Config\n");
//   int err;

//   // Port configuration
//   int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

//   /// Define I2C configurations
//   i2c_config_t conf;
//   conf.mode = I2C_MODE_MASTER;                              // Master mode
//   conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
//   conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
//   conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
//   conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
//   conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
//   err = i2c_param_config(i2c_master_port, &conf);           // Configure
//   if (err == ESP_OK) {printf("- parameters: ok\n");}

//   // Install I2C driver
//   err = i2c_driver_install(i2c_master_port, conf.mode,
//                      I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
//                      I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
//   if (err == ESP_OK) {printf("- initialized: yes\n");}

//   // Data in MSB mode
//   i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
// }

// // Utility  Functions //////////////////////////////////////////////////////////

// // Utility function to test for I2C device address -- not used in deploy
// int testConnection(uint8_t devAddr, int32_t timeout) {
//   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//   i2c_master_start(cmd);
//   i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
//   i2c_master_stop(cmd);
//   int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
//   i2c_cmd_link_delete(cmd);
//   return err;
// }

// // Utility function to scan for i2c device
// static void i2c_scanner() {
//   int32_t scanTimeout = 1000;
//   printf("\n>> I2C scanning ..."  "\n");
//   uint8_t count = 0;
//   for (uint8_t i = 1; i < 127; i++) {
//     // printf("0x%X%s",i,"\n");
//     if (testConnection(i, scanTimeout) == ESP_OK) {
//       printf( "- Device found at address: 0x%X%s", i, "\n");
//       count++;
//     }
//   }
//   if (count == 0) {printf("- No I2C devices found!" "\n");}
// }

// ////////////////////////////////////////////////////////////////////////////////

// // ADXL343 Functions ///////////////////////////////////////////////////////////

// // Get Device ID
// int getDeviceID(uint8_t *data) {
//   int ret;
//   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//   i2c_master_start(cmd);
//   i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
//   i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
//   i2c_master_start(cmd);
//   i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
//   i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
//   i2c_master_stop(cmd);
//   ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
//   i2c_cmd_link_delete(cmd);
//   return ret;
// }

// // Write one byte to register; referenced the datasheet
// int writeRegister(uint8_t reg, uint8_t data) {
//   int ret;

//   i2c_cmd_handle_t cmd = i2c_cmd_link_create();

//   i2c_master_start(cmd);  // start
//   i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);   // slave address + write
//   i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);  // register address
  
//   i2c_master_write_byte(cmd, data, ACK_CHECK_DIS); // data
  
//   i2c_master_stop(cmd);   // stop
//   ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);  // output the results
//   i2c_cmd_link_delete(cmd);

//   return ret;
// }

// // Read register; referenced the datasheet
// uint8_t readRegister(uint8_t reg) {
//   int ret;

//   i2c_cmd_handle_t cmd = i2c_cmd_link_create();

//   i2c_master_start(cmd);  // start
//   i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);  // slave address + write
//   i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);  // register address

//   i2c_master_start(cmd);  // start
//   i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN); // slave address + read
//   i2c_master_read_byte(cmd, &ret, ACK_CHECK_DIS);  // nack
//   i2c_master_stop(cmd); // stop
  
//   i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS); // outputs the results
//   i2c_cmd_link_delete(cmd);

//   return ret;
// }

// // read 16 bits (2 bytes)
// int16_t read16(uint8_t reg) {
//   int ret;

//   // can call the read 1-byte register function twice for 2 bytes
//   uint8_t b1 = readRegister(reg);
//   uint8_t b2 = readRegister(reg + 1);

//   ret = ((b2 << 8) | b1);

//   return ret;
// }

// void setRange(range_t range) {
//   /* Red the data format register to preserve bits */
//   uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

//   /* Update the data rate */
//   format &= ~0x0F;
//   format |= range;

//   /* Make sure that the FULL-RES bit is enabled for range scaling */
//   format |= 0x08;

//   /* Write the register back to the IC */
//   writeRegister(ADXL343_REG_DATA_FORMAT, format);

// }

// range_t getRange(void) {
//   /* Red the data format register to preserve bits */
//   return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
// }

// dataRate_t getDataRate(void) {
//   return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
// }

// ////////////////////////////////////////////////////////////////////////////////

// // function to get acceleration
// void getAccel(float * xp, float *yp, float *zp) {
//   *xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
//   *yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
//   *zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
//  printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", *xp, *yp, *zp);
// }

// // function to print roll and pitch
// void calcRP(float x, float y, float z){
//   // YOUR CODE HERE; found this from tilt sensing source
//   float roll = atan2(y , z) * 57.3;
//   float pitch = atan2((-1*x) , sqrt(y * y + z * z)) * 57.3;
//   printf("roll: %.2f \t pitch: %.2f \n", roll, pitch);
// }

// // Task to continuously poll acceleration and calculate roll and pitch
// static void test_adxl343() {
//   printf("\n>> Polling ADAXL343\n");
//   while (1) {
//     float xVal, yVal, zVal;
//     getAccel(&xVal, &yVal, &zVal);
//     calcRP(xVal, yVal, zVal);
//     vTaskDelay(500 / portTICK_RATE_MS);
//   }
// }

////////////////////////////////////////////////////////////////////////////////

// Configure UART and set-up Tx/Rx tasks ///////////////////////////////////////

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART, &uart_config);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void tx_task(void *arg)
{
	char* Txdata = (char*) malloc(30);
    while (1)
    {
    	sprintf (Txdata, "Hello world index = %d\r\n", num++);
    	uart_write_bytes(UART, Txdata, strlen(Txdata));
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    free (Txdata);
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART, data, RX_BUF_SIZE, 500 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = '\0';
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
        }
    }
    free(data);
}

void app_main(void)
{
    uart_init();
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-2, NULL);
}
