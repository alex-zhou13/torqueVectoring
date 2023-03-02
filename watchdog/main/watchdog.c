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

// Define Pins
#define RELAY_GPIO CONFIG_RELAY_GPIO // Default GPIO 12/A11
// #define BLINK_GPIO CONFIG_BLINK_GPIO // Board LED is GPIO 38

// Voltage Reader
#define DEFAULT_VREF    3300        //Use adc1_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t V_LEFT_CHANNEL = ADC_CHANNEL_8, // adc1_8 GPIO19
                           V_RIGHT_CHANNEL = ADC_CHANNEL_9; // adc1_9 GPIO10
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

// Safety shutdown calculator vals
#define TIMESTEP        10          // in ms
#define DURATION        2           // in cumulative seconds sustained
#define THRESHOLD       3           // voltage threshold to start monitoring

// // LED Values and Functions
// static uint8_t s_led_state = 0;
// static led_strip_t *pStrip_a;
// static void set_led(int red, int green, int blue)
// {
//     /* If the addressable LED is enabled */
//     if (s_led_state) {
//         /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
//         pStrip_a->set_pixel(pStrip_a, 0, red, green, blue);
//         /* Refresh the strip to send data */
//         pStrip_a->refresh(pStrip_a, 100);
//     } else {
//         /* Set all LED off to clear all pixels */
//         pStrip_a->clear(pStrip_a, 50);
//     }
// }

// static void configure_led(void)
// {
//     /* LED strip initialization with the GPIO and pixels number*/
//     pStrip_a = led_strip_init(CONFIG_BLINK_LED_RMT_CHANNEL, BLINK_GPIO, 1);
//     /* Set all LED off to clear all pixels */
//     pStrip_a->clear(pStrip_a, 50);
// }

void app_main(void)
{
    // Config GPIO
    gpio_reset_pin(RELAY_GPIO);
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);

    
    // Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(V_LEFT_CHANNEL, atten);
        adc1_config_channel_atten(V_RIGHT_CHANNEL, atten);
    }

    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    // value to monitor excessive speeding
    double speed_sum = 0;

    // close relay to send power to next component
    gpio_set_level(RELAY_GPIO, 1);

    // Config and set board LED to green
    // configure_led();
    // set_led(0,16,0);
    // s_led_state = 1;

    // Continuously sample adc1
    while (1)
    {
        uint32_t raw_left = 0, raw_right = 0;

        // Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            raw_left += adc1_get_raw((adc1_channel_t)V_LEFT_CHANNEL);
            raw_right += adc1_get_raw((adc1_channel_t)V_RIGHT_CHANNEL);
        }

        raw_left /= NO_OF_SAMPLES;
        raw_right /= NO_OF_SAMPLES;

        // Convert ADC readings to voltages in mV
        double   v_left = esp_adc_cal_raw_to_voltage(raw_left, adc_chars) - esp_adc_cal_raw_to_voltage(0, adc_chars) + 1,
                 v_right = esp_adc_cal_raw_to_voltage(raw_right, adc_chars) - esp_adc_cal_raw_to_voltage(0, adc_chars) + 1;
        // Calculate values
        v_left *= 2, v_right *= 2; // because the effective range is 0-3.1v, we used a 0.5 voltage divider to accomodate 0.5-4.5 (0.25-2.75).

        // Evaluate values
        // if (v_left >= THRESHOLD || v_right >= THRESHOLD) {
        //     speed_sum += (((v_left > v_right) ? v_left : v_right) - THRESHOLD) * (double) TIMESTEP/1000
        // } else {
        //     speed_sum = 0;
        // }

        // if (speed_sum > DURATION) {
        //     break; // excessive speed, must break and shut down system
        // }

        // Print to console
        printf("Voltage Left\t%.2f\n", v_left);
        printf("Voltage Right\t%.2f\n", v_right);

        vTaskDelay(pdMS_TO_TICKS(TIMESTEP));
    }

    // Only happens if there is reason to shut down    
    // Blink LED red and open Relay
    while (1) {
        // set_led(255,0,0);
        gpio_set_level(RELAY_GPIO, 0);
        /* Toggle the LED state */
        // s_led_state = !s_led_state;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
