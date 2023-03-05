// STD C Library
#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"
#include "esp_types.h"

// RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "freertos/queue.h"

// GPIO，UART，ADC, drivers
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "mcp4725.h" // dac_left device driver

/////////////// GPIO DEF ////////////////
#define RELAY_GPIO 1 // Default GPIO 1

/////////////// VEHIC PARAMS DEF //////////////
#define SIM_WHEEL_DIAM  0.5     // m
#define SIM_RATIO       3.6
#define SIM_AIR_DENSITY 1.23
#define SIM_DRAG_COEF   1
#define SIM_ROLL_COEF   0.1
#define SIM_FRONT_AREA  1.2
#define SIM_FRIC_COEF   0.8
#define SIM_WEIGHT      250     // kg
#define SIM_TIMESTEP    500     // ms
#define SIM_LENGTH      2       // m
#define SIM_WIDTH       1.5

/////////////// I2C DEF ////////////////
#define SDA_PIN 18
#define SCL_PIN 19
#define I2C_CLK_SPEED 400000 // 400 KHz

#define MCP4725_LEFT_ADDRESS 0x62 // default address
#define MCP4725_MAX_TICKS (10/portTICK_PERIOD_MS) // max ticks for i2c

/////////////// UART DEF ////////////////
#define TXD GPIO_NUM_16 // Send
#define RXD GPIO_NUM_17 // Recieve 
#define UART_RTS (UART_PIN_NO_CHANGE)
#define UART_CTS (UART_PIN_NO_CHANGE)

#define UART_PORT_NUM      2
#define UART_BAUD_RATE          115200
#define UART_TASK_STACK_SIZE    2048

#define BUF_SIZE (1024)
char start = 0x1B;
int len_out = 4;

// /////////////// RTOS DEF ////////////////
// static xQueueHandle uart_evt_queue_send = NULL;
// static xQueueHandle dac_left_evt_queue_send = NULL;

/////////////// GLOBAL VARS DEF ////////////////
double throttle_scaled = 0;
double thr_left_scaled = 0;
double thr_rigt_scaled = 0;
double steering_scaled = 0;

/////////////// I2C FUNCS ////////////////
static void mcp4725_task() {
    static const char* TAG = "mcp4725";

    mcp4725_config_t dac_left = {
        .i2c_bus        = I2C_NUM_0,
        .address        = MCP4725_LEFT_ADDRESS,
        .ticks_to_wait  = MCP4725_MAX_TICKS
    }; // dac_left configuration

    // mcp4725_config_t dac_right = {
    //     .i2c_bus        = I2C_NUM_0,
    //     .address        = MCP4725_LEFT_ADDRESS,
    //     .ticks_to_wait  = MCP4725_MAX_TICKS
    // }; // dac_right configuration

    mcp4725_eeprom_t eeprom_write = {
        .power_down_mode = MCP4725_POWER_DOWN_100, // power down, 100K resistor, 2 
        .input_data = 2047 
    }; // values to write to eeprom, will be set on reboot

    mcp4725_eeprom_t eeprom_read; // structure to hold the eeprom after we read it

    ESP_ERROR_CHECK(mcp4725_write_eeprom(dac_left,eeprom_write)); // write the above configuration
    vTaskDelay(50 / portTICK_PERIOD_MS); // wait for device
    ESP_ERROR_CHECK(mcp4725_read_eeprom(dac_left,&eeprom_read)); // read the eeprom

    ESP_LOGI(TAG,"Power_Down_Mode Saved: %i",eeprom_read.power_down_mode); // print the saved configuration
    ESP_LOGI(TAG,"Input_Data Saved: %i",eeprom_read.input_data);
    // Continuously write values as they arrive via UART
    while (1) {
      int messageData[4] = {0};
        // if xQueueReceive(dac_left_evt_queue_send, &messageData, 100 / portTICK_PERIOD_MS)) {
            ESP_ERROR_CHECK(mcp4725_set_voltage(dac_left,thr_left_scaled*(double) 4095)); // scale from 0-1 to 0-4095 = 5v
            vTaskDelay(pdMS_TO_TICKS(10));
        // }
    }
    vTaskDelete(NULL);
}

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

///////////////// UART FUNCS //////////////////
static void uart_init() {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD, RXD, UART_RTS, UART_CTS));

    return;
}

static void uart_rx_task() {
    // Configure a temporary buffer for the incoming data
    // uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    // Buffer for input data
    uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE);
    while (1) {
        int len_in = uart_read_bytes(UART_PORT_NUM, data_in, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        if (len_in > 0) {
            if (data_in[0] == start) {
                int throttle_mv_lower = data_in[1];
                int throttle_mv_upper = data_in[2];
                int steering_mv_lower = data_in[3];
                // int steering_mv_lower = 0;
                int steering_mv_upper = data_in[4];
                // int steering_mv_upper = 0;
                
                data_in[len_in] = '\0';
                int throttle_mv = throttle_mv_upper*256+throttle_mv_lower;
                int steering_mv = steering_mv_upper*256+steering_mv_lower;
                throttle_scaled = (double) throttle_mv/5000;
                steering_scaled = (double) steering_mv/5000 - 0.5;

                // printf("UART data Recieved: th_left=%d\th_right=%d (mV) \n", throttle_mv, steering_mv);
                // printf("Values converted:   throttle=%f\tsteering=%f\n", throttle_scaled, steering_scaled);
                // // Notify PID
                // xQueueSend(uart_evt_queue_send, &mv_thr_send, 10);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    free(data_in);
}

///////////////// GPIO FUNCS //////////////////
static void gpio_init() {
    // Configure GPIO
    gpio_reset_pin(RELAY_GPIO);
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);

    // close relay to send power to next component
    gpio_set_level(RELAY_GPIO, 1);

    return;
}

///////////////// PID FUNCS //////////////////
static void temp_PID_task() {
    while(1) {
      int messageData[4] = {0};
          // if (xQueueReceive(uart_evt_queue_send, &messageData, 100 / portTICK_PERIOD_MS)) {
          thr_left_scaled = throttle_scaled + ((steering_scaled < 0) ? steering_scaled : 0);
          thr_rigt_scaled = throttle_scaled - ((steering_scaled > 0) ? steering_scaled : 0);
          // thr_left_scaled = throttle_scaled;
          // thr_rigt_scaled = throttle_scaled;

          // Notify dac_left
          // xQueueSend(dac_left_evt_queue_send, &mv_thr_send, 10);
          vTaskDelay(pdMS_TO_TICKS(10));
    }
}

double calc_speed(double speed, double t_left, double t_right) {
    double FL = (double) 72*SIM_RATIO/(SIM_WHEEL_DIAM/2)*t_left;
    double FR = (double) 72*SIM_RATIO/(SIM_WHEEL_DIAM/2)*t_right;
    speed = (double) speed + (FR+FL)*((double) SIM_TIMESTEP/1000)/((double)2*SIM_WEIGHT) - ((double) SIM_ROLL_COEF*SIM_WEIGHT + 0.5*SIM_AIR_DENSITY*SIM_DRAG_COEF*SIM_FRONT_AREA*speed*speed)*((double) SIM_TIMESTEP/1000)/(double)SIM_WEIGHT;
    // printf("Force Left\t%.3f\tRight\t%.3f\n", FL, FR);

    return speed;
}

static void speed_emulator_task () {
    // ALG
    double speed = 0;
    while (1) {
        speed = calc_speed(speed, thr_left_scaled, thr_rigt_scaled);
        printf("Throttle Left\t%.3f\tRight\t%.3f\n", thr_left_scaled, thr_rigt_scaled);
        printf("Speed: %f m\\s\n", speed);
        vTaskDelay(pdMS_TO_TICKS(SIM_TIMESTEP));
    }
}

void app_main() {
    i2c_setup();
    gpio_init();
    uart_init();

    xTaskCreate(mcp4725_task,"mcp4725_task",2048,NULL,5,NULL);
    xTaskCreate(uart_rx_task,"uart_rx_task",2048,NULL,5,NULL);
    xTaskCreate(temp_PID_task,"temp_PID_task",2048,NULL,5,NULL);
    xTaskCreate(speed_emulator_task,"speed_emulator_task",2048,NULL,5,NULL);
}
