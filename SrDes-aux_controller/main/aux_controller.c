#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_types.h"
#include "sdkconfig.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"
// #include "led_strip.h"

// GPIO VALS
#define RELAY_GPIO CONFIG_RELAY_GPIO // Default GPIO 12/A11

// ADC VALS
#define DEFAULT_VREF    3300        //Use adc1_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t V_LEFT_CHANNEL = ADC_CHANNEL_8, // adc1_8 GPIO9
                           V_RIGHT_CHANNEL = ADC_CHANNEL_9; // adc1_9 GPIO10
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

// UART VALS
static const int RX_BUF_SIZE = 1024;
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)
#define UART UART_NUM_2

// ADC INIT FUNCTION
static void adc_init() {
    // Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(V_LEFT_CHANNEL, atten);
        adc1_config_channel_atten(V_RIGHT_CHANNEL, atten);
    }
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    return;
}

// GPIO INIT FUNCTION
static void gpio_init() {
    // Configure GPIO
    gpio_reset_pin(RELAY_GPIO);
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);

    // close relay to send power to next component
    gpio_set_level(RELAY_GPIO, 1);

    return;
}

// UART FUNCTIONS
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
	char* Txdata = (char*) malloc(40);
    while (1)
    {
    	// sprintf (Txdata, "Tx received. Hello world index = %d\r\n", num++);
      sprintf (Txdata, "\nTx received. Hello world!");
    	uart_write_bytes(UART, Txdata, strlen(Txdata));
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    free (Txdata);
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "Rx task on";
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
    adc_init();
    gpio_init();

    // Continuously sample adc1
    while (1)
    {
        uint32_t raw_left = 0, raw_right = 0;

        // ADC Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            raw_left += adc1_get_raw((adc1_channel_t)V_LEFT_CHANNEL);
            raw_right += adc1_get_raw((adc1_channel_t)V_RIGHT_CHANNEL);
        }

        raw_left /= NO_OF_SAMPLES;
        raw_right /= NO_OF_SAMPLES;

        // Convert ADC readings to voltages in mV
        double   v_left = (double) (esp_adc_cal_raw_to_voltage(raw_left, adc_chars) - esp_adc_cal_raw_to_voltage(0, adc_chars) + 1)/1000,
                 v_right = (double) (esp_adc_cal_raw_to_voltage(raw_right, adc_chars) - esp_adc_cal_raw_to_voltage(0, adc_chars) + 1)/1000;
        
        // Calculate values
        v_left *= 2, v_right *= 2; // because the effective range is 0-3.1v, we used a 0.5 voltage divider to accomodate 0.5-4.5 (0.25-2.75).

        // Print to console
        printf("Voltage Left\t%.3f\tRight\t%.3f\n", v_left, v_right);
        printf("speed_sum: %f\n", speed_sum);

        vTaskDelay(pdMS_TO_TICKS(TIMESTEP));
    }
}
