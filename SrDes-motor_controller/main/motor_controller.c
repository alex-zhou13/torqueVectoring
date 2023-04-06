#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "driver/timer.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/////////////// GLOBALS ////////////////
double thr_left;
double thr_right;
#define TIMESTEP        10          // in ms

/////////////// MOTOR/SERVO DEF ////////////////
#define ESC_LEFT_GPIO                (18)   // GPIO connects to the PWM signal line for the left esc/motor
#define ESC_RIGHT_GPIO               (19)   // GPIO connects to the PWM signal line for the right esc/motor

#define MAX_MOTOR_DUTY          (1860) // Maximum motor duty in microseconds
#define TOP_MOTOR_DUTY          (1400) // Maximum motor duty in microseconds
#define MIN_MOTOR_DUTY          (1280) // Minimum motor duty in microseconds

/////////////// ADC DEF ////////////////
#define DEFAULT_VREF    3300        //Use adc1_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t V_LEFT_CHANNEL = ADC_CHANNEL_8, // adc1_8 GPIO9
                           V_RIGHT_CHANNEL = ADC_CHANNEL_9; // adc1_9 GPIO10
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

///////////////// MOTOR/SERVO FUNCS /////////////////
// Basic task that inits motor and sets speeds
static void motor_task() {
    // Init Motor and servo
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, ESC_LEFT_GPIO); // To drive a RC servo, one MCPWM generator is enough
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, ESC_RIGHT_GPIO); // To drive a RC servo, one MCPWM generator is enough

    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);

    int motor_left_duty ;
    int motor_right_duty;

    while (1) {
        motor_left_duty = (TOP_MOTOR_DUTY-MIN_MOTOR_DUTY)*thr_left+MIN_MOTOR_DUTY;
        motor_right_duty = (TOP_MOTOR_DUTY-MIN_MOTOR_DUTY)*thr_right+MIN_MOTOR_DUTY;
        printf("Duty L: %d\tDuty R: %d\n",motor_left_duty,motor_right_duty);
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, motor_left_duty));
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, motor_right_duty));
        vTaskDelay(pdMS_TO_TICKS(TIMESTEP)); //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation under 5V power supply
    }
}
///////////////// ADC FUNCS /////////////////
void adc_task() {
    // Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(V_LEFT_CHANNEL, atten);
        adc1_config_channel_atten(V_RIGHT_CHANNEL, atten);
    }
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

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

        thr_left = v_left/5;
        thr_right = v_right/5;

        // Print to console
        printf("Voltage Left\t%.3f\tRight\t%.3f\n", v_left, v_right);

        vTaskDelay(pdMS_TO_TICKS(TIMESTEP));
    }
}
///////////////// MAIN FUNC //////////////////
void app_main(void)
{
    xTaskCreate(&motor_task, "motor_task", 4096, NULL, 5, NULL);
    xTaskCreate(&adc_task, "adc_task", 4096, NULL, 5, NULL);
    
    vTaskDelay(6500/portTICK_PERIOD_MS); // Don't start PID until ESP is initialized
}


