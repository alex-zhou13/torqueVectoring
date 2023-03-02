#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MCP4725_DAC_ADDR                    0x62        /*!< Slave address of the MCP4725 DAC (1100 00 [A0])*/

// #define MPU9250_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
// #define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

// #define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
// #define MPU9250_RESET_BIT                   7

// static esp_err_t mcp4725_register_read(uint8_t reg_addr, uint8_t *data, size_t len) {
//     return i2c_master_write_read_device(I2C_MASTER_NUM, MCP4725_DAC_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// }

// static esp_err_t mcp4725_register_write_byte(uint8_t reg_addr, uint8_t data) {
//     int ret;
//     uint8_t write_buf[2] = {reg_addr, data};
//     ret = i2c_master_write_to_device(I2C_MASTER_NUM, MCP4725_DAC_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

//     return ret;
// }

static esp_err_t mcp4725_set_voltage(mcp4725_config_t dac,uint16_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // make a command
    i2c_master_start(cmd); // start command
    i2c_master_write_byte(cmd,(dac.address << 1) | I2C_MASTER_WRITE,1); // write to address
    
    i2c_master_write_byte(cmd,(value >> 8) | MCP4725_WRITE_FAST,1); // write upper 4 bits
    i2c_master_write_byte(cmd,value & MCP4725_MASK,1); // write lower 8 bits
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dac.i2c_bus,cmd,dac.ticks_to_wait); // begin sending command
    i2c_cmd_link_delete(cmd);
    return ret;
}
/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void app_main(void)
{
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
    ESP_ERROR_CHECK(mcp4725_register_read(MPU9250_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Demonstrate writing by reseting the MPU9250 */
    ESP_ERROR_CHECK(mcp4725_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
