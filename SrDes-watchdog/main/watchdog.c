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

// OUTPUT PINS
#define RELAY_GPIO CONFIG_RELAY_GPIO // Default GPIO 12/A11
// #define BLINK_GPIO CONFIG_BLINK_GPIO // Board LED is GPIO 38

// ADC VALUES
#define DEFAULT_VREF    3300        //Use adc1_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t V_LEFT_CHANNEL = ADC_CHANNEL_8, // adc1_8 GPIO9
                           V_RIGHT_CHANNEL = ADC_CHANNEL_9; // adc1_9 GPIO10
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

// Safety shutdown calculator vals
#define TIMESTEP        10          // in ms
#define DURATION        5           // in cumulative seconds sustained
#define THRESHOLD       3           // voltage threshold to start monitoring


void app_main(void)
{
    // Configure GPIO
    gpio_reset_pin(RELAY_GPIO);
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);

    
    // Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(V_LEFT_CHANNEL, atten);
        adc1_config_channel_atten(V_RIGHT_CHANNEL, atten);
    }
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    // value to monitor excessive speeding
    double speed_sum = 0;

    // close relay to send power to next component
    gpio_set_level(RELAY_GPIO, 1);

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

        // Evaluate values
        if (v_left >= THRESHOLD || v_right >= THRESHOLD) {
            speed_sum += (((v_left > v_right) ? v_left : v_right) - THRESHOLD) * (double) TIMESTEP/1000;
        } else {
            speed_sum = 0;
        }

        if (speed_sum > DURATION) {
            break; // excessive speed, must break and shut down system
        }

        // Print to console
        printf("Voltage Left\t%.3f\tRight\t%.3f\n", v_left, v_right);
        printf("speed_sum: %f\n", speed_sum);

        vTaskDelay(pdMS_TO_TICKS(TIMESTEP));
    }

    // Only happens if there is reason to shut down    
    while (1) {
        printf("Tripped!\n");
        gpio_set_level(RELAY_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
