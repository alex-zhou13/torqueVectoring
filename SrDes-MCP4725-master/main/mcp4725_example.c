#include <stdio.h> // puts, uint, etc.
#include "freertos/FreeRTOS.h" // freeRTOS core 
#include "freertos/task.h" // freeRTOS tasks
#include "esp_log.h" // logging functions

#include "mcp4725.h" // device driver

/* physical characteristics, change as necessary */
#define SDA_PIN 18
#define SCL_PIN 19
//#define I2C_CLK_SPEED 100000 // 100 KHz
#define I2C_CLK_SPEED 400000 // 400 KHz
#define MCP4725_ADDRESS 0x62 // default address
#define MCP4725_MAX_TICKS (10/portTICK_PERIOD_MS) // max ticks for i2c

// /* Blinking LED characteristics, change as necessary */
// #define LED_PIN 2
// #define LED_DELAY 1000 // milliseconds

static void mcp4725_task(void* pvParameter) {
  static const char* TAG = "mcp4725";
  
  mcp4725_config_t dac = {
    .i2c_bus        = I2C_NUM_0,
    .address        = MCP4725_ADDRESS,
    .ticks_to_wait  = MCP4725_MAX_TICKS
  }; // dac configuration
  
  mcp4725_eeprom_t eeprom_write = {
    //.power_down_mode = MCP4725_POWER_DOWN_0, // not in power down mode, 0
    //.power_down_mode = MCP4725_POWER_DOWN_1, // power down, 1K resistor, 1
    .power_down_mode = MCP4725_POWER_DOWN_100, // power down, 100K resistor, 2 
    //.power_down_mode = MCP4725_POWER_DOWN_500, // power down, 500K resistor, 3
    .input_data = 2047 
  }; // values to write to eeprom, will be set on reboot
  
  mcp4725_eeprom_t eeprom_read; // structure to hold the eeprom after we read it
  
  ESP_ERROR_CHECK(mcp4725_write_eeprom(dac,eeprom_write)); // write the above configuration
  vTaskDelay(50 / portTICK_PERIOD_MS); // wait for device
  ESP_ERROR_CHECK(mcp4725_read_eeprom(dac,&eeprom_read)); // read the eeprom
  
  ESP_LOGI(TAG,"Power_Down_Mode Saved: %i",eeprom_read.power_down_mode); // print the saved configuration
  ESP_LOGI(TAG,"Input_Data Saved: %i",eeprom_read.input_data);
  
  uint16_t val = 0; // value to write
  static const uint16_t increment = 100; // amount to change value by
  static const uint16_t delay = 500; // milliseconds to wait by
  
  for(;;) {
    ESP_ERROR_CHECK(mcp4725_set_voltage(dac,val));
    ESP_LOGI(TAG,"Value=%i",val);
    val += increment;
    if(val>4095) val = 0;
    vTaskDelay(delay / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

// static void blink_task(void* pvParameter) { // task to blink the led
//   gpio_pad_select_gpio(LED_PIN); // set pin to gpio mode
//   gpio_set_direction(LED_PIN,GPIO_MODE_OUTPUT); // set pin to output mode
//   for(;;) {
//     gpio_set_level(LED_PIN,1); // turn led on
//     vTaskDelay(LED_DELAY / portTICK_PERIOD_MS); // let cpu do other stuff
//     gpio_set_level(LED_PIN,0); // turn led off
//     vTaskDelay(LED_DELAY / portTICK_PERIOD_MS); // let cpu do other stuff
//   }
//   vTaskDelete(NULL); // delete task, always needed though never reached
// }

static void i2c_setup() {
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = SDA_PIN;
  conf.scl_io_num = SCL_PIN;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_CLK_SPEED;
  conf.clk_flags = 0;
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
  ESP_LOGI("i2c_bus","SCL=%i SDA=%i CLK_SPEED=%i",SCL_PIN,SDA_PIN,I2C_CLK_SPEED);
}

void app_main() {
  i2c_setup();
  xTaskCreate(&mcp4725_task,"mcp4725_task",2048,NULL,5,NULL);
  // xTaskCreate(&blink_task,"blink_task",configMINIMAL_STACK_SIZE,NULL,1,NULL); // recommended minimal size
}
