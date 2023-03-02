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

// Params
#define WHEEL_DIAM  0.5     // m
#define RATIO       3.6
#define AIR_DENSITY 1.23
#define DRAG_COEF   1
#define ROLL_COEF   0.1
#define FRONT_AREA  1.2
#define FRIC_COEF   0.8
#define WEIGHT      250     // kg
#define TIMESTEP    100     // ms
#define LENGTH      2       // m
#define WIDTH       1.5


// TEST INPUT
#define DEFAULT_VREF    3300        //Use adc1_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t V_LEFT_CHANNEL = ADC_CHANNEL_8, // adc1_8 GPIO19
                           V_RIGHT_CHANNEL = ADC_CHANNEL_9; // adc1_9 GPIO10
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

double calc_speed(double speed, double t_left, double t_right) {
    double FL = (double) 72*RATIO/(WHEEL_DIAM/2)*t_left;
    double FR = (double) 72*RATIO/(WHEEL_DIAM/2)*t_right;
    speed = (double) speed + (FR+FL)*((double) TIMESTEP/1000)/((double)2*WEIGHT) - ((double) ROLL_COEF*WEIGHT + 0.5*AIR_DENSITY*DRAG_COEF*FRONT_AREA*speed*speed)*((double) TIMESTEP/1000)/(double)WEIGHT;
    printf("Force Left\t%.3f\tRight\t%.3f\n", FL, FR);
    printf("Power\t%.3f\t", (FR+FL)*(TIMESTEP/1000)/(2*WEIGHT));
    printf("Drag\t%.3f\t", ((double) ROLL_COEF*WEIGHT + 0.5*AIR_DENSITY*DRAG_COEF*FRONT_AREA*speed*speed)*(TIMESTEP/1000)/WEIGHT);

    return speed;
}

void app_main() {
    // TEST SETUP
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(V_LEFT_CHANNEL, atten);
        adc1_config_channel_atten(V_RIGHT_CHANNEL, atten);
    }
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    
    // ALG
    double speed = 0;
    double t_left, t_right;
    while (1) {
        // TEST
        uint32_t raw_left = 0, raw_right = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            raw_left += adc1_get_raw((adc1_channel_t)V_LEFT_CHANNEL);
            raw_right += adc1_get_raw((adc1_channel_t)V_RIGHT_CHANNEL);
        }
        raw_left /= NO_OF_SAMPLES;
        raw_right /= NO_OF_SAMPLES;
        t_left = (double) (esp_adc_cal_raw_to_voltage(raw_left, adc_chars) - esp_adc_cal_raw_to_voltage(0, adc_chars) + 1)/1000;
        t_right = (double) (esp_adc_cal_raw_to_voltage(raw_right, adc_chars) - esp_adc_cal_raw_to_voltage(0, adc_chars) + 1)/1000;
        t_left /= 2.5, t_right /= 2.5; // because the effective range is 0-3.1v, we used a 0.5 voltage divider to accomodate 0.5-4.5 (0.25-2.75).

        // ALG
        // t_left = (count%100 > 0 && count%100 <=10) ? 1 : 0;
        // t_right = (count%100 > 0 && count%100 <=10) ? 1 : 0;
         
        speed = calc_speed(speed, t_left, t_right);
        printf("Throttle Left\t%.3f\tRight\t%.3f\n", t_left, t_right);
        printf("Speed: %f m\\s\n", speed);
        vTaskDelay(pdMS_TO_TICKS(TIMESTEP));
    }
}
