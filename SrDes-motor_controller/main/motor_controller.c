#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "driver/timer.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/twai.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/////////////// GLOBALS ////////////////
double thr_left_scaled;
double thr_right_scaled;
#define TIMESTEP        15          // in ms
int RPM_L = 0;
int RPM_R = 0;
bool enable = true;
bool testing = false;
bool slip_L = fals
bool slip_R = false;

/////////////// GPIO DEF ////////////////
#define GPIO_LEFT_SLIP     4
#define GPIO_RIGHT_SLIP    5

/////////////// MOTOR/SERVO DEF ////////////////
#define ESC_LEFT_GPIO                (17)   // GPIO connects to the PWM signal line for the left esc/motor
#define ESC_RIGHT_GPIO               (18)   // GPIO connects to the PWM signal line for the right esc/motor

#define MAX_MOTOR_DUTY          (1080) // Maximum motor duty in microseconds
#define MIN_MOTOR_DUTY          (1000) // Minimum motor duty in microseconds

/////////////// ADC DEF ////////////////
#define DEFAULT_VREF    3300        //Use adc1_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t V_LEFT_CHANNEL = ADC_CHANNEL_8, // adc1_8 GPIO9
                           V_RIGHT_CHANNEL = ADC_CHANNEL_9, // adc1_9 GPIO10
                           V_ENABLE_CHANNEL = ADC_CHANNEL_7; // adc1_7 GPIO8
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

/////////////// CAN DEF ////////////////
QueueHandle_t can_tx_queue = NULL;
#define CAN_TX_GPIO_NUM     1//(GPIO_NUM_43)
#define CAN_RX_GPIO_NUM     2//(GPIO_NUM_44)

#define MC_LEFT_ID          0x00
#define MC_RIGHT_ID         0x01

#define GD1_REPORT_FREQ     500  // ms
#define GD5_REPORT_FREQ     1000  // ms

/////////////// VEHIC PARAMS DEF //////////////
#define SIM_MAX_VOLTAGE     237     // V
#define SIM_MAX_DISCHARGE   220     // A, not really needed
#define SIM_RPM_PER_V       17      // RPM
#define SIM_NM_PER_A        0.5     // Nm
#define SIM_MAX_RPM         SIM_MAX_VOLTAGE*SIM_RPM_PER_V
#define SIM_MAX_TORQUE      SIM_MAX_DISCHARGE*SIM_NM_PER_A

#define SIM_WHEEL_DIAM      0.5     // m
#define SIM_RATIO           3.6
#define SIM_AIR_DENSITY     1.23
#define SIM_DRAG_COEF       1
#define SIM_ROLL_COEF       0.1
#define SIM_FRONT_AREA      1.2
#define SIM_STA_FRIC_COEF   1.2
#define SIM_KIN_FRIC_COEF   0.8*SIM_STA_FRIC_COEF // very bad case
#define SIM_WEIGHT          210     // kg
#define SIM_TIMESTEP        15       // ms
#define SIM_LENGTH          2       // m
#define SIM_WIDTH           1.5

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

    // Allow time for the esc to be plugged inESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, motor_left_duty));
    printf("Plug in ESC now!\n");
    // ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MAX_MOTOR_DUTY));
    // ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MAX_MOTOR_DUTY));
    // vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MIN_MOTOR_DUTY-40));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MIN_MOTOR_DUTY-40));
    vTaskDelay(pdMS_TO_TICKS(10000));

    RPM_L = 0;
    RPM_R = 0;
    while (1) {
        motor_left_duty = (int) ((float) (MAX_MOTOR_DUTY-MIN_MOTOR_DUTY)*((float) RPM_L/((float) SIM_MAX_RPM))+MIN_MOTOR_DUTY);
        motor_right_duty = (int) ((float) (MAX_MOTOR_DUTY-MIN_MOTOR_DUTY)*((float) RPM_R/((float) SIM_MAX_RPM))+MIN_MOTOR_DUTY);
        // printf("Duty L: %d\tDuty R: %d\n",motor_left_duty,motor_right_duty);
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, motor_left_duty));
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, motor_right_duty));
        vTaskDelay(pdMS_TO_TICKS(TIMESTEP)); //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation under 5V power supply
    }
}
///////////////// ADC FUNCS /////////////////
void adc_task() {
    // Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(V_LEFT_CHANNEL, atten);
        adc1_config_channel_atten(V_RIGHT_CHANNEL, atten);
        adc1_config_channel_atten(V_ENABLE_CHANNEL, atten);
    }
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    // Continuously sample adc1
    while (1)
    {
        uint32_t raw_left = 0, raw_right = 0, raw_enable = 0;

        // ADC Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            raw_left += adc1_get_raw((adc1_channel_t)V_LEFT_CHANNEL);
            raw_right += adc1_get_raw((adc1_channel_t)V_RIGHT_CHANNEL);
            raw_enable += adc1_get_raw((adc1_channel_t)V_ENABLE_CHANNEL);
        }

        raw_left /= NO_OF_SAMPLES;
        raw_right /= NO_OF_SAMPLES;
        raw_enable /= NO_OF_SAMPLES;

        // printf("Raw Right = %ld\tRaw Left = %ld\n",raw_right, raw_left);

        // Convert ADC readings to voltages in mV
        double   v_left = (double) (esp_adc_cal_raw_to_voltage(raw_left, adc_chars) - esp_adc_cal_raw_to_voltage(0, adc_chars) + 1)/1000,
                 v_right = (double) (esp_adc_cal_raw_to_voltage(raw_right, adc_chars) - esp_adc_cal_raw_to_voltage(0, adc_chars) + 1)/1000,
                 v_enable = (double) (esp_adc_cal_raw_to_voltage(raw_enable, adc_chars) - esp_adc_cal_raw_to_voltage(0, adc_chars) + 1)/1000;
        
        // Calculate values
        v_left *= 2, v_right *= 2, v_enable *= 2; // because the effective range is 0-3.1v, we used a 0.5 voltage divider to accomodate 0.5-4.5 (0.25-2.75).

        enable = (v_enable>2.5); // if "high"

        thr_left_scaled = (v_left/5 > 1) ? 1 : v_left/5;
        thr_right_scaled = (v_right/5 > 1) ? 1 : v_right/5;

        // Print to console
        // printf("Voltage Left\t%.3f\tRight\t%.3f\tthr_L:\t%.3f\tthr_R:\t%.3f\n", v_left, v_right,thr_left_scaled,thr_right_scaled);

        vTaskDelay(pdMS_TO_TICKS(TIMESTEP));
    }
}

///////////////// CAN FUNCS /////////////////
/* DTI HV500 CAN Bus Behavior
Speeds: 125,250,500,1000 Kbit/s]
Default ID is last 2 digits of inverter serial num in DEC
Supports Standard and Extended ID with limitaitions
    Standard: ID 1-30, 31 reserved for boroadcast, 5 bits, unless serial is >= 31, in which case it is truncated to the 5 LSBs
    Extended: ID 1-254, 255 reserved for broadcast, 8 bits
Message format (MSB to the left)
    Big Endian

    Standard:
    Message ID         | Data bytes
    Packet ID| Node ID | 
    10:5     | 4:0     |

    Extended:
    Message ID         | Data bytes
    Packet ID| Node ID | 
    28:8     | 7:0     |

For full list of Packet ID, see https://drive.google.com/file/d/18WqW30AykNudS3VukyKqq-GsveZrWgq1/view

Signals Transmitted by Inverter
    0x20 ERPM, Duty, Input Voltage
    0x21 AC Current, DC Current
    0x22 Controller Temp., Motor Temp., Fault code
    0x23 Id, Iq values
    0x24 Throttle signal, Brake signal, Digital I/Os, Drive enable, Limit status bits, CAN map version

Each message is broadcast at a specific frequency, set before runtime by the DTI CAN Tool. Relevant Params declared above

Commands to the inverter
    0x01 Set Current
    0x02 Set Brake current
    0x03 Set ERPM
    0x04 Set Position
    0x05 Set Relative current
    0x06 Set relative brake current
    0x07 Set digital output (Sets an output to HIGH or LOW)
    0x08 Set maximum AC current
    0x09 Set maximum AC brake current
    0x0A Set maximum DC current
    0x0B Set maximum DC brake current
    0x0C Drive enable
*/

/* This Config
We will not be processing any incoming CAN bus commands, only sending very select commands, namely
    0x20
    0x22, fault codes only
    0x24, with some variables left blank (0xFFFF)
*/
void can_init() {
    //Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500 Kbit/s
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    //Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }

    //Start TWAI driver
    if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return;
    }
}

void can_tx_task() {
    // Incoming format: Packet ID | Node ID (Ex) | Data Bytes
    int txBuffer[10];
    while (can_tx_queue == 0) {
        printf("waiting for queue creation\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while (1) {
        if (xQueueReceive(can_tx_queue, &(txBuffer), (TickType_t)5)) {
            // Configure message to transmit
            twai_message_t message;
            message.identifier = (txBuffer[0]<<8) + txBuffer[1]; // PacketID and NodeID
            message.extd = 1; // Extended format
            message.data_length_code = 8; // Length of message in bytes, always 8 for sending
            for (int i = 0; i < message.data_length_code; i++) {
                message.data[i] = txBuffer[i+2];
            }

            //queue message for transmission
            if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
                // printf("Message queued for transmission\n");
            } else {
                printf("Failed to queue message for transmission\n");
            }
        } else {
            // printf("No message queued for transmission right now\n");
        }
    }
}

void can_rx_task() {
    while (1) {
        // Wait for message to be received
        twai_message_t message;
        int waitticks = 10000;
        if (twai_receive(&message, pdMS_TO_TICKS(waitticks)) == ESP_OK) {
            printf("Message received\n");
        } else {
            printf("Failed to receive message after %d seconds\n", waitticks/1000);
        }

        //Process received message
        if (message.extd) {
            printf("Message is in Extended Format\n");
        } else {
            printf("Message is in Standard Format\n");
        }
        // printf("MessageID:\t%ld\nPacketID:\t%ld\nNodeID: \t%ld", message.identifier, message.identifier>>8, message.identifier%pow(2,8));
        if (!(message.rtr)) {
            for (int i = 0; i < message.data_length_code; i++) {
                printf("Data byte %d = %d\n", i, message.data[i]);
            }
        }
    }
}
    
void can_alert_task() {
    //Reconfigure alerts to detect Error Passive and Bus-Off error states////////////
    uint32_t alerts_to_enable = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        printf("Alerts reconfigured\n");
    } else {
        printf("Failed to reconfigure alerts");
    }

    while(1) {
        //Block indefinitely until an alert occurs
        uint32_t alerts_triggered;
        twai_read_alerts(&alerts_triggered, portMAX_DELAY);
    }
}

///////////////// DEBUGGING/TESTING DUMMY FUNCS //////////////////
// Estimates the wheel and vehicle speed of the vehile given throttle signals
static void speed_emulator_task () {
    double speed = 0;
    int idle_time  = 0;
    while (1) {
        if (enable && !testing) {
            // Calculate motor force for left and right wheels
            double FL = (double) SIM_MAX_TORQUE*SIM_RATIO/(SIM_WHEEL_DIAM/2)*thr_left_scaled; // Nm
            double FR = (double) SIM_MAX_TORQUE*SIM_RATIO/(SIM_WHEEL_DIAM/2)*thr_right_scaled;  // Nm


            float steering_scaled = 0.5; // temp, until we get a steering input
            // Calculate max deliverable force for left and right wheels, taking weight transfer into account (assume max of 75% of weight)
            //                       fric coeff         weight (N)      weight transfer              downforce (speed in m/s)
            double FL_max = (double) SIM_STA_FRIC_COEF*(SIM_WEIGHT*9.8*(0.5-(steering_scaled-0.5)/2)+0.5*SIM_AIR_DENSITY*SIM_DRAG_COEF*SIM_FRONT_AREA/2*pow(speed,2));
            double FR_max = (double) SIM_STA_FRIC_COEF*(SIM_WEIGHT*9.8*(0.5+(steering_scaled-0.5)/2)+0.5*SIM_AIR_DENSITY*SIM_DRAG_COEF*SIM_FRONT_AREA/2*pow(speed,2));
            
            // Calculate if there is slip
            if (FL > FL_max) {
                FL = SIM_KIN_FRIC_COEF/SIM_STA_FRIC_COEF*FL_max;
                slip_L = 1;
                printf("SLIP LEFT\n");
            } else {
                FL = FL;
                slip_L = 0;
            }

            if (FR > FR_max) {
                FR = SIM_KIN_FRIC_COEF/SIM_STA_FRIC_COEF*FR_max;
                slip_R = 1;
                printf("SLIP RIGHT\n");
            } else {
                FL = FL;
                slip_R = 0;
            }

            // Calculate speed
            speed = (double) speed + (FR+FL)*((double) SIM_TIMESTEP/1000.0)/((double)2*SIM_WEIGHT) - ((double) SIM_ROLL_COEF*SIM_WEIGHT + 0.5*SIM_AIR_DENSITY*SIM_DRAG_COEF*SIM_FRONT_AREA*speed*speed)*((double) SIM_TIMESTEP/1000)/(double)SIM_WEIGHT;

            // Apply "brakes" after 2 seconds of no throttle (<1%), acceleration of -5 m/s^2
            if (thr_left_scaled <= 0.01 && thr_right_scaled <= 0.01) {
                idle_time += SIM_TIMESTEP;
                if (idle_time >= 2000) {
                    speed = speed - (double) 5*SIM_TIMESTEP/1000;
                    idle_time = 2000;
                }
            } else {
                idle_time = 0;
            }
            
            // Gate speed by (non-slip) RPM limit and 0 (no reverse)
            speed = (speed >= SIM_MAX_RPM/SIM_RATIO*SIM_WHEEL_DIAM*3.1415/60) ? SIM_MAX_RPM/SIM_RATIO*SIM_WHEEL_DIAM*3.1415/60 : speed;
            speed = (speed >= 0) ? speed : 0;
            
            // Calculate RPM, global var
            // If slip, say gains 1000 RPM per second, otherwise determined by wheel speed
            RPM_L = (slip_L) ? ((RPM_L >= SIM_MAX_RPM) ? SIM_MAX_RPM : RPM_L + SIM_TIMESTEP) : (double) speed / (SIM_WHEEL_DIAM*3.1415) * SIM_RATIO * 60;
            RPM_R = (slip_R) ? ((RPM_R >= SIM_MAX_RPM) ? SIM_MAX_RPM : RPM_R + SIM_TIMESTEP) : (double) speed / (SIM_WHEEL_DIAM*3.1415) * SIM_RATIO * 60;
            
            vTaskDelay(pdMS_TO_TICKS(SIM_TIMESTEP));
        } else if (!testing) {
            speed = 0;
            RPM_L = 0;
            RPM_R = 0;
            vTaskDelay(pdMS_TO_TICKS(SIM_TIMESTEP));
        } else {
            speed = 0;
            RPM_L = (RPM_L<0||RPM_L>=SIM_MAX_RPM) ? 0 : RPM_L + 200;
            RPM_R = RPM_L;
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    printf("TL:\t%.3f\tTR:\t%.3f\tRPML:\t%d\tRPMR:\t%d\t(RPM_max=%d)\tSpeed:\t%f m\\s\n", thr_left_scaled, thr_right_scaled,RPM_L, RPM_R, SIM_MAX_RPM, speed);
    }
}

// Sends the data corresponding to "General Data 1" in the DTI HV500 CAN Bus spec
void gd1_report_task() {
    int txBuffer[10];
    can_tx_queue = xQueueCreate(50, sizeof(txBuffer)); 
    if (can_tx_queue == 0)
    {
        printf("Failed to create can_tx_queue= %p\n", can_tx_queue);
    }

    while (1) {
        txBuffer[0] = 0x20;
        txBuffer[1] = MC_LEFT_ID;
        txBuffer[2] = (uint8_t) RPM_L>>8*3; // ERPM, [31:24]
        txBuffer[3] = (uint8_t) RPM_L>>8*2; // ERPM, [23:16]
        txBuffer[4] = (uint8_t) RPM_L>>8*1; // ERPM, [15:08]
        txBuffer[5] = (uint8_t) RPM_L;      // ERPM, [07:00]
        txBuffer[6] = (uint8_t) 0xFF;       // Duty Cycle [15:08]
        txBuffer[7] = (uint8_t) 0xFF;       // Duty Cycle [07:00]
        txBuffer[8] = (uint8_t) 0xFF;       // Input Voltage [15:08]
        txBuffer[9] = (uint8_t) 0xFF;       // Input Voltage [07:00]
        xQueueSend(can_tx_queue, (void*)txBuffer, (TickType_t)0);
        
        txBuffer[1] = MC_RIGHT_ID;
        txBuffer[2] = (uint8_t) RPM_R>>8*3; // ERPM, [31:24]
        txBuffer[3] = (uint8_t) RPM_R>>8*2; // ERPM, [23:16]
        txBuffer[4] = (uint8_t) RPM_R>>8*1; // ERPM, [15:08]
        txBuffer[5] = (uint8_t) RPM_R;      // ERPM, [07:00]
        xQueueSend(can_tx_queue, (void*)txBuffer, (TickType_t)0);

        vTaskDelay(pdMS_TO_TICKS(GD1_REPORT_FREQ));
    }
}

// Sends the data corresponding to "General Data 5" in the DTI HV500 CAN Bus spec
void gd5_report_task() {
    int txBuffer[10];
    while (can_tx_queue == 0)
        vTaskDelay(pdMS_TO_TICKS(GD1_REPORT_FREQ));

    while (1) {
        int tl = thr_left_scaled*127;
        int tr = thr_right_scaled*127;
        txBuffer[0] = 0x24;
        txBuffer[1] = MC_LEFT_ID;
        txBuffer[2] = (uint8_t) tl;   // throttle L
        txBuffer[3] = (uint8_t) 0x00; // brake
        txBuffer[4] = (uint8_t) 0x00; // digital
        txBuffer[5] = (uint8_t) enable; // Drive Enable
        txBuffer[6] = (uint8_t) 0xFF; // Don't care
        txBuffer[7] = (uint8_t) 0xFF; // Don't care
        txBuffer[8] = (uint8_t) 0xFF; // Don't care
        txBuffer[9] = (uint8_t) 0xFF; // Don't care
        xQueueSend(can_tx_queue, (void*)txBuffer, (TickType_t)0);
        
        txBuffer[1] = MC_RIGHT_ID;
        txBuffer[2] = (uint8_t) tr;   // throttle R
        xQueueSend(can_tx_queue, (void*)txBuffer, (TickType_t)0);

        vTaskDelay(pdMS_TO_TICKS(GD5_REPORT_FREQ));
    }
}

void slip_indicator_task () {
    // Configure GPIO
    gpio_reset_pin(GPIO_LEFT_SLIP);
    gpio_reset_pin(GPIO_RIGHT_SLIP);
    gpio_set_direction(GPIO_LEFT_SLIP, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_RIGHT_SLIP, GPIO_MODE_OUTPUT);

    // close relay to send power to next component
    gpio_set_level(GPIO_LEFT_SLIP, 0);
    gpio_set_level(GPIO_RIGHT_SLIP, 0);

    int flash = 0;
    while (1) {
        if (slip_L || slip_R) {
            flash = !flash;
            gpio_set_level(GPIO_LEFT_SLIP, (slip_L)?flash:0);
            gpio_set_level(GPIO_RIGHT_SLIP, (slip_R)?flash:0);
            
        } else {
            gpio_set_level(GPIO_LEFT_SLIP, 0);
            gpio_set_level(GPIO_RIGHT_SLIP, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

///////////////// MAIN FUNC //////////////////
void app_main(void)
{
    can_init();
    
    xTaskCreate(&motor_task, "motor_task", 4096, NULL, 5, NULL);
    xTaskCreate(&adc_task, "adc_task", 4096, NULL, 5, NULL);

    xTaskCreate(&can_tx_task, "can_tx_task", 4096, NULL, 5, NULL);
    xTaskCreate(&can_rx_task, "can_rx_task", 4096, NULL, 5, NULL);
    xTaskCreate(&can_alert_task, "can_alert_task", 4096, NULL, 5, NULL);
    xTaskCreate(&gd1_report_task, "gd1_report_task", 4096, NULL, 5, NULL);
    xTaskCreate(&gd5_report_task, "gd5_report_task", 4096, NULL, 5, NULL);
    
    xTaskCreate(&speed_emulator_task, "speed_emulator_task", 4096, NULL, 5, NULL);
    xTaskCreate(&slip_indicator_task, "slip_indicator_task", 4096, NULL, 5, NULL);
}


