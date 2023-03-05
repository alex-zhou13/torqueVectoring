// STD C Library
#include <stdio.h>
#include <stdlib.h>
#include <sys/param.h>
#include "sdkconfig.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"
#include "esp_types.h"

// RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

// GPIO，UART，ADC
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "esp_adc_cal.h"

/////////////// GPIO DEF ////////////////
#define RELAY_GPIO CONFIG_RELAY_GPIO // Default GPIO 12/A11

/////////////// ADC DEF ////////////////
#define TIMESTEP        20
#define DEFAULT_VREF    3300        //Use adc1_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t V_THR_CHANNEL = ADC_CHANNEL_8, // adc1_8 GPIO9
                           V_STR_CHANNEL = ADC_CHANNEL_9; // adc1_9 GPIO10
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

int mv_thr_send = 0;
int mv_str_send = 0;

// RTOS Queue
// static xQueueHandle adc_evt_queue_send = NULL;

/////////////// UART DEF ////////////////
#define TXD GPIO_NUM_17 // Send
#define RXD GPIO_NUM_18 // Recieve 
#define UART_RTS (UART_PIN_NO_CHANGE)
#define UART_CTS (UART_PIN_NO_CHANGE)

#define UART_PORT_NUM           2
#define UART_BAUD_RATE          115200
#define UART_TASK_STACK_SIZE    2048

#define BUF_SIZE (1024)
char start = 0x1B;
int len_out = 4;

///////////////// ADC FUNCS //////////////////
static void adc_init() {
    // Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(V_THR_CHANNEL, atten);
        adc1_config_channel_atten(V_STR_CHANNEL, atten);
    }
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    return;
}

static void adc_read_task() {
    // Continuously sample adc1
    while (1)
    {
        uint32_t raw_thr = 0, raw_str = 0;

        // ADC Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            raw_thr += adc1_get_raw((adc1_channel_t)V_THR_CHANNEL);
            raw_str += adc1_get_raw((adc1_channel_t)V_STR_CHANNEL);
        }

        raw_thr /= NO_OF_SAMPLES;
        raw_str /= NO_OF_SAMPLES;

        // Convert ADC readings to voltages in mV
        int   mv_thr = (esp_adc_cal_raw_to_voltage(raw_thr, adc_chars) - esp_adc_cal_raw_to_voltage(0, adc_chars) + 1),
              mv_str = (esp_adc_cal_raw_to_voltage(raw_str, adc_chars) - esp_adc_cal_raw_to_voltage(0, adc_chars) + 1);
        
        // Calculate values
        mv_thr_send = 2*mv_thr, mv_str_send = 2*mv_str; // because the effective range is 0-3.1v, we used a 0.5 voltage divider to accomodate 0.5-4.5 (0.25-2.75).

        // // Notify UART send_task
        // xQueueSend(adc_evt_queue_send, &mv_thr_send, 10);

        // Print to console
        printf("Voltage Left\t%.3d\tRight\t%.3d\n", mv_thr_send, mv_str_send);

        vTaskDelay(pdMS_TO_TICKS(TIMESTEP));
    }
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

///////////////// UART FUNCS //////////////////
char genCheckSum(char *p, int len) {
  char temp = 0;
  for (int i = 0; i < len; i++){
    temp = temp^p[i];
  }
  // printf("%X\n",temp);

  return temp;
}

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

static void uart_tx_task() {
    // Configure a temporary buffer for the incoming data
    // uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    int len_out = 6;
    // int len_out = 4;
    char start = 0x1B;

    char *data_out = (char *) malloc(len_out);
    while (1) {
        int messageData[6] = {0};
        // if (xQueueReceive(adc_evt_queue_send, &messageData, 100 / portTICK_PERIOD_MS)) {

            data_out[0] = start;
            data_out[1] = (char) ((int) mv_thr_send % 256); // send throttle value in mV
            data_out[2] = (char) ((int) mv_thr_send / 256); // send throttle value in mV
            data_out[3] = (char) ((int) mv_str_send % 256); // send steering value in mV
            data_out[4] = (char) ((int) mv_str_send / 256); // send steering value in mV
            data_out[5] = genCheckSum(data_out,len_out-1);
            // data_out[3] = genCheckSum(data_out,len_out-1);
            
            uart_write_bytes(UART_PORT_NUM, data_out, len_out);
            printf("Data sent! \n");
            vTaskDelay(pdMS_TO_TICKS(50));
        // }
    }
}

void app_main(void)
{
    adc_init();
    gpio_init();
    uart_init();

    xTaskCreate(adc_read_task,"adc_read_task", 4096, NULL, 5, NULL);
    xTaskCreate(uart_tx_task,"uart_tx_task", 4096, NULL, 4, NULL);
}
