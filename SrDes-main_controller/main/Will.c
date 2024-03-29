// STD C Library
#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"
#include "esp_types.h"
#include <math.h>
#include <unistd.h>

// RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// GPIO，UART，ADC, drivers
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "mcp4725.h" // dac_left device driver
#include "driver/twai.h"

// PI controller
#include "Linear_Model.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "Linear_Model_private.h"
#include <string.h>

/////////////// GPIO DEF ////////////////
#define RELAY_GPIO 19 // changed to 19

/////////////// VEHIC PARAMS DEF //////////////
#define SIM_MAX_VOLTAGE     237     // V
#define SIM_MAX_DISCHARGE   220     // A, not really needed
#define SIM_RPM_PER_V       17      // RPM
#define SIM_NM_PER_A        0.5     // Nm
#define SIM_MAX_RPM         SIM_MAX_VOLTAGE*SIM_RPM_PER_V
#define SIM_MAX_TORQUE      SIM_MAX_DISCHARGE*SIM_NM_PER_A

#define SIM_WHEEL_DIAM      0.5 // m
#define SIM_RATIO           3.6
#define SIM_AIR_DENSITY     1.23
#define SIM_DRAG_COEF       1
#define SIM_ROLL_COEF       0.1
#define SIM_FRONT_AREA      1.2
#define SIM_FRIC_COEF       1
#define SIM_KIN_FRIC_COEF   0.8*SIM_STA_FRIC_COEF // very bad case
#define SIM_WEIGHT          210   // kg
#define SIM_TIMESTEP        5 // ms
#define SIM_LENGTH          2     // m
#define SIM_WIDTH           1.5

/////////////// I2C DEF ////////////////
#define SDA_PIN 9
#define SCL_PIN 10
#define I2C_CLK_SPEED 400000 // 400 KHz

#define MCP4725_LEFT_ADDRESS 0x63                   // default address
#define MCP4725_RIGHT_ADDRESS 0x62                  // default address
#define MCP4725_MAX_TICKS (10 / portTICK_PERIOD_MS) // max ticks for i2c

/////////////// UART DEF ////////////////
#define TXD GPIO_NUM_17 // Send
#define RXD GPIO_NUM_18 // Recieve
#define UART_RTS (UART_PIN_NO_CHANGE)
#define UART_CTS (UART_PIN_NO_CHANGE)

#define UART_PORT_NUM 2
#define UART_BAUD_RATE 115200
#define UART_TASK_STACK_SIZE 2048

#define BUF_SIZE (1024)
char start = 0x1B;
int len_out = 4;

/////////////// CAN DEF ////////////////
QueueHandle_t can_tx_queue = NULL;
#define CAN_TX_GPIO_NUM 1
#define CAN_RX_GPIO_NUM 2

#define MC_LEFT_ID 0x00
#define MC_RIGHT_ID 0x01

/////////////// GLOBAL VARS DEF ////////////////
double throttle_scaled;
double thr_left_scaled;
double thr_rigt_scaled;
double steering_scaled;
double speed_scaled;
int RPM_L = 0;
int RPM_R = 0;
bool testing_mode = 0; // Boolean value to put the program into testing mode

/////////////// SIMULINK PI DEFS AND FUNCS ///////////////
/* Block signals (default storage) */
B_Linear_Model_T Linear_Model_B;

/* Continuous states */
X_Linear_Model_T Linear_Model_X;

/* External inputs (root inport signals with default storage) */
ExtU_Linear_Model_T Linear_Model_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Linear_Model_T Linear_Model_Y;

/* Real-time model */
static RT_MODEL_Linear_Model_T Linear_Model_M_;
RT_MODEL_Linear_Model_T *const Linear_Model_M = &Linear_Model_M_;
real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex){
    real_T frac;
    real_T yL_0d0;
    uint32_T iLeft;

    /* Column-major Lookup 1-D
    Search method: 'binary'
    Use previous index: 'off'
    Interpolation method: 'Linear point-slope'
    Extrapolation method: 'Linear'
    Use last breakpoint for index at or above upper limit: 'off'
    Remove protection against out-of-range input in generated code: 'off'
    */
    /* Prelookup - Index and Fraction
    Index Search method: 'binary'
    Extrapolation method: 'Linear'
    Use previous index: 'off'
    Use last breakpoint for index at or above upper limit: 'off'
    Remove protection against out-of-range input in generated code: 'off'
    */
    if (u0 <= bp0[0U]) {
        iLeft = 0U;
        frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
    }
    else if (u0 < bp0[maxIndex]) {
        uint32_T bpIdx;
        uint32_T iRght;

        /* Binary Search */
        bpIdx = maxIndex >> 1U;
        iLeft = 0U;
        iRght = maxIndex;
        while (iRght - iLeft > 1U) {
            if (u0 < bp0[bpIdx]) {
                iRght = bpIdx;
            }
            else {
                iLeft = bpIdx;
            }

            bpIdx = (iRght + iLeft) >> 1U;
        }

        frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
    }
    else {
        iLeft = maxIndex - 1U;
        frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
    }

    /* Column-major Interpolation 1-D
    Interpolation method: 'Linear point-slope'
    Use last breakpoint for index at or above upper limit: 'off'
    Overflow mode: 'portable wrapping'
    */
    yL_0d0 = table[iLeft];
    return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si) {
    /* Solver Matrices */
    static const real_T rt_ODE3_A[3] = {
        1.0 / 2.0, 3.0 / 4.0, 1.0};

    static const real_T rt_ODE3_B[3][3] = {
        {1.0 / 2.0, 0.0, 0.0},

        {0.0, 3.0 / 4.0, 0.0},

        {2.0 / 9.0, 1.0 / 3.0, 4.0 / 9.0}};

    time_T t = rtsiGetT(si);
    time_T tnew = rtsiGetSolverStopTime(si);
    time_T h = rtsiGetStepSize(si);
    real_T *x = rtsiGetContStates(si);
    ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
    real_T *y = id->y;
    real_T *f0 = id->f[0];
    real_T *f1 = id->f[1];
    real_T *f2 = id->f[2];
    real_T hB[3];
    int_T i;
    int_T nXc = 4;
    rtsiSetSimTimeStep(si, MINOR_TIME_STEP);

    /* Save the state values at time t in y, we'll use x as ynew. */
    (void)memcpy(y, x,
                 (uint_T)nXc * sizeof(real_T));

    /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
    /* f0 = f(t,y) */
    rtsiSetdX(si, f0);
    Linear_Model_derivatives();

    /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
    hB[0] = h * rt_ODE3_B[0][0];
    for (i = 0; i < nXc; i++) {
        x[i] = y[i] + (f0[i] * hB[0]);
    }

    rtsiSetT(si, t + h * rt_ODE3_A[0]);
    rtsiSetdX(si, f1);
    Linear_Model_step();
    Linear_Model_derivatives();

    /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
    for (i = 0; i <= 1; i++) {
        hB[i] = h * rt_ODE3_B[1][i];
    }

    for (i = 0; i < nXc; i++)
    {
        x[i] = y[i] + (f0[i] * hB[0] + f1[i] * hB[1]);
    }

    rtsiSetT(si, t + h * rt_ODE3_A[1]);
    rtsiSetdX(si, f2);
    Linear_Model_step();
    Linear_Model_derivatives();

    /* tnew = t + hA(3);
    ynew = y + f*hB(:,3); */
    for (i = 0; i <= 2; i++) {
        hB[i] = h * rt_ODE3_B[2][i];
    }

    for (i = 0; i < nXc; i++) {
        x[i] = y[i] + (f0[i] * hB[0] + f1[i] * hB[1] + f2[i] * hB[2]);
    }

    rtsiSetT(si, tnew);
    rtsiSetSimTimeStep(si, MAJOR_TIME_STEP);
}

/* Model step function */
void Linear_Model_step(void) {
    real_T tmp;
    real_T tmp_0;
    real_T tmp_1;
    real_T tmp_2;
    real_T yawRate;
    real_T yawRate_max;
    if (rtmIsMajorTimeStep(Linear_Model_M)) {
        /* set solver stop time */
        if (!(Linear_Model_M->Timing.clockTick0 + 1)) {
            rtsiSetSolverStopTime(&Linear_Model_M->solverInfo,
                                  ((Linear_Model_M->Timing.clockTickH0 + 1) *
                                   Linear_Model_M->Timing.stepSize0 * 4294967296.0));
        }
        else
        {
            rtsiSetSolverStopTime(&Linear_Model_M->solverInfo,
                                  ((Linear_Model_M->Timing.clockTick0 + 1) *
                                       Linear_Model_M->Timing.stepSize0 +
                                   Linear_Model_M->Timing.clockTickH0 *
                                       Linear_Model_M->Timing.stepSize0 * 4294967296.0));
        }
    } /* end MajorTimeStep */

    /* Update absolute time of base rate at minor time step */
    if (rtmIsMinorTimeStep(Linear_Model_M))
    {
        Linear_Model_M->Timing.t[0] = rtsiGetT(&Linear_Model_M->solverInfo);
    }

    /* MATLAB Function: '<Root>/Desired_yawRate' incorporates:
     *  Inport: '<Root>/Steering Angle Encoder'
     *  Inport: '<Root>/V_CG'
     */
    /* :  g = 9.81; */
    /* :  Cyf = 20; */
    /* :  Cyr = 1; */
    /* :  m = 500; */
    /* :  l_f = 0.813; */
    /* :  l_r = 0.783; */
    /* :  Izz = 190.701; */
    /* :  K_u = 0.05; */
    /* :  tire_road_coeff = 1; */
    /* :  sig = 1; */
    /* :  steering_Angle = steering_In*2*90/180*pi; */
    /* :  yawRate_max = sig*tire_road_coeff*g/V_CG; */
    yawRate_max = 9.81 / speed_scaled;

    /* :  yawRate = V_CG/((l_r+l_f)+K_u*V_CG^2) * steering_Angle; */
    yawRate = speed_scaled / (speed_scaled * speed_scaled * 0.05 + 1.596) * (steering_scaled * 2.0 * 90.0 / 180.0 * 3.1415926535897931);

    /* :  if (abs(yawRate) <= abs(yawRate_max)) */
    if (fabs(yawRate) <= fabs(yawRate_max))
    {
        /* :  des_yawRate = yawRate; */
        Linear_Model_B.des_yawRate = yawRate;
    }
    else
    {
        /* :  else */
        /* :  if (sign(yawRate) == 1) */
        if (!rtIsNaN(yawRate))
        {
            if (yawRate < 0.0)
            {
                yawRate = -1.0;
            }
            else
            {
                yawRate = (yawRate > 0.0);
            }
        }

        if (yawRate == 1.0)
        {
            /* :  des_yawRate = yawRate_max; */
            Linear_Model_B.des_yawRate = yawRate_max;
        }
        else
        {
            /* :  else */
            /* :  des_yawRate = -yawRate_max; */
            Linear_Model_B.des_yawRate = -yawRate_max;
        }
    }

    /* End of MATLAB Function: '<Root>/Desired_yawRate' */

    /* Integrator: '<S5>/Integrator' */
    Linear_Model_B.x[0] = Linear_Model_X.VyYaw_rate[0];
    Linear_Model_B.x[1] = Linear_Model_X.VyYaw_rate[1];

    /* Sum: '<Root>/Sum' */
    Linear_Model_B.Sum = Linear_Model_B.des_yawRate - Linear_Model_B.x[1];

    /* Lookup_n-D: '<Root>/1-D Lookup Table' incorporates:
     *  Inport: '<Root>/V_CG'
     */
    Linear_Model_B.uDLookupTable = look1_binlxpw(speed_scaled,
                                                 Linear_Model_P.uDLookupTable_bp01Data,
                                                 Linear_Model_P.uDLookupTable_tableData, 6U);

    /* Product: '<S42>/PProd Out' */
    Linear_Model_B.PProdOut = Linear_Model_B.Sum * Linear_Model_B.uDLookupTable;

    /* Integrator: '<S37>/Integrator' */
    Linear_Model_B.Integrator = Linear_Model_X.Integrator_CSTATE;

    /* Product: '<S31>/DProd Out' incorporates:
     *  Constant: '<Root>/Constant3'
     */
    Linear_Model_B.DProdOut = Linear_Model_B.Sum * Linear_Model_P.Constant3_Value;

    /* Integrator: '<S32>/Filter' */
    Linear_Model_B.Filter = Linear_Model_X.Filter_CSTATE;

    /* Sum: '<S32>/SumD' */
    Linear_Model_B.SumD = Linear_Model_B.DProdOut - Linear_Model_B.Filter;

    /* Product: '<S40>/NProd Out' */
    Linear_Model_B.NProdOut = Linear_Model_B.SumD * 0.0;

    /* Sum: '<S46>/Sum' */
    Linear_Model_B.Sum_h = (Linear_Model_B.PProdOut + Linear_Model_B.Integrator) +
                           Linear_Model_B.NProdOut;

    /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
     *  Constant: '<Root>/Max Toque of Motors (N*m)'
     *  Constant: '<Root>/Max Voltage'
     *  Inport: '<Root>/Steering Angle Encoder'
     *  Inport: '<Root>/throttle_Input'
     */
    /* :  if (sign(steering_In) == 1) */
    yawRate_max = steering_scaled;
    if (!rtIsNaN(yawRate_max))
    {
        if (yawRate_max < 0.0)
        {
            yawRate_max = -1.0;
        }
        else
        {
            yawRate_max = (yawRate_max > 0.0);
        }
    }

    if (yawRate_max == 1.0)
    {
        /* :  right_Motor_Control = throttle_Input*motor_Control_Voltage_Max; */
        yawRate = throttle_scaled * Linear_Model_P.MaxVoltage_Value;

        /* :  left_Motor_Control = right_Motor_Control - torque_Diff/max_Torque_Scale*motor_Control_Voltage_Max; */
        yawRate_max = yawRate - Linear_Model_B.Sum_h /
                                    Linear_Model_P.MaxToqueofMotorsNm_Value * Linear_Model_P.MaxVoltage_Value;

        /* :  if left_Motor_Control < 0 */
        if (yawRate_max < 0.0)
        {
            /* :  left_Motor_Control = 0; */
            yawRate_max = 0.0;
        }
    }
    else
    {
        /* :  else */
        /* :  left_Motor_Control = throttle_Input*motor_Control_Voltage_Max; */
        yawRate_max = throttle_scaled *
                      Linear_Model_P.MaxVoltage_Value;

        /* :  right_Motor_Control = left_Motor_Control - torque_Diff/max_Torque_Scale*motor_Control_Voltage_Max; */
        yawRate = yawRate_max - Linear_Model_B.Sum_h /
                                    Linear_Model_P.MaxToqueofMotorsNm_Value * Linear_Model_P.MaxVoltage_Value;

        /* :  if right_Motor_Control < 0 */
        if (yawRate < 0.0)
        {
            /* :  right_Motor_Control = 0; */
            yawRate = 0.0;
        }
    }

    Linear_Model_B.right_Motor_Control = yawRate;
    printf("Right: %f", Linear_Model_B.right_Motor_Control);
    Linear_Model_B.left_Motor_Control = yawRate_max;
    printf("Left: %f", Linear_Model_B.left_Motor_Control);

    /* End of MATLAB Function: '<Root>/MATLAB Function' */

    /* Outport: '<Root>/right_Motor_Control' */
    Linear_Model_Y.right_Motor_Control = Linear_Model_B.right_Motor_Control;

    /* Outport: '<Root>/left_Motor_Control' */
    Linear_Model_Y.left_Motor_Control = Linear_Model_B.left_Motor_Control;

    /* Lookup_n-D: '<Root>/1-D Lookup Table1' incorporates:
     *  Inport: '<Root>/V_CG'
     */
    Linear_Model_B.uDLookupTable1 = look1_binlxpw(speed_scaled,
                                                  Linear_Model_P.uDLookupTable1_bp01Data,
                                                  Linear_Model_P.uDLookupTable1_tableData, 6U);

    /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
     *  Inport: '<Root>/V_CG'
     */
    /* :  g = 9.81; */
    /* :  Cyf = 20; */
    /* :  Cyr = 1; */
    /* :  m = 500; */
    /* :  l_f = 0.813; */
    /* :  l_r = 0.783; */
    /* :  Izz = 190.701; */
    /* :  K_u = 0.05; */
    /* :  tr = 0.645; */
    /* :  Gr = 3.6; */
    /* :  Rw = 0.25; */
    /* :  A =[-(Cyf+Cyr)/(m*Vx0), (-l_f*Cyf+l_r*Cyr)/(m*Vx0)-Vx0; (-l_f*Cyf+l_r*Cyr)/... */
    /* :      (Izz*Vx0), -(l_f^2*Cyf+l_r^2*Cyr)/(Izz*Vx0)]; */
    Linear_Model_B.A[0] = -21.0 / (500.0 * speed_scaled);
    Linear_Model_B.A[2] = -15.476999999999999 / (500.0 * speed_scaled) -
                          speed_scaled;
    Linear_Model_B.A[1] = -15.476999999999999 / (190.701 * speed_scaled);
    Linear_Model_B.A[3] = -13.832468999999998 / (190.701 * speed_scaled);

    /* :  B = [Cyf/(m*Vx0), 0; (l_f*Cyf)/Izz, 1/(Rw/(2*tr*Gr)*Izz)]; */
    Linear_Model_B.B[0] = 20.0 / (500.0 * speed_scaled);
    Linear_Model_B.B[2] = 0.0;

    /* :  C = [1, 1]; */
    /* :  D = [0, 0]; */
    Linear_Model_B.B[1] = 0.085264366731165539;
    Linear_Model_B.C[0] = 1.0;
    Linear_Model_B.D[0] = 0.0;
    Linear_Model_B.B[3] = 0.097409032988814967;
    Linear_Model_B.C[1] = 1.0;
    Linear_Model_B.D[1] = 0.0;

    /* Product: '<S34>/IProd Out' */
    Linear_Model_B.IProdOut = Linear_Model_B.Sum * Linear_Model_B.uDLookupTable1;

    /* SignalConversion generated from: '<S5>/Product' incorporates:
     *  Inport: '<Root>/Steering Angle Encoder'
     */
    Linear_Model_B.TmpSignalConversionAtProductInp[0] =
        steering_scaled;
    Linear_Model_B.TmpSignalConversionAtProductInp[1] = Linear_Model_B.Sum_h;

    /* Product: '<S5>/Product' */
    yawRate_max = Linear_Model_B.B[0];
    yawRate = Linear_Model_B.TmpSignalConversionAtProductInp[0];
    tmp_0 = Linear_Model_B.TmpSignalConversionAtProductInp[1];
    yawRate_max *= yawRate;
    yawRate_max += 0.0 * tmp_0;

    /* Product: '<S5>/Product' */
    Linear_Model_B.Bu[0] = yawRate_max;

    /* Product: '<S5>/Product' */
    tmp_1 = 0.085264366731165539 * yawRate;
    tmp_1 += 0.097409032988814967 * tmp_0;

    /* Product: '<S5>/Product' */
    Linear_Model_B.Bu[1] = tmp_1;

    /* Product: '<S5>/Product1' */
    yawRate_max = Linear_Model_B.A[0];
    tmp_1 = Linear_Model_B.A[1];
    tmp = Linear_Model_B.A[2];
    tmp_2 = Linear_Model_B.A[3];
    yawRate = Linear_Model_B.x[0];
    tmp_0 = Linear_Model_B.x[1];
    yawRate_max *= yawRate;
    yawRate_max += tmp * tmp_0;

    /* Product: '<S5>/Product1' */
    Linear_Model_B.Ax[0] = yawRate_max;

    /* Sum: '<S5>/Sum' */
    Linear_Model_B.dx[0] = Linear_Model_B.Bu[0] + Linear_Model_B.Ax[0];

    /* Product: '<S5>/Product1' */
    tmp_1 *= yawRate;
    tmp_1 += tmp_2 * tmp_0;

    /* Product: '<S5>/Product1' */
    Linear_Model_B.Ax[1] = tmp_1;

    /* Sum: '<S5>/Sum' */
    Linear_Model_B.dx[1] = Linear_Model_B.Bu[1] + Linear_Model_B.Ax[1];
    if (rtmIsMajorTimeStep(Linear_Model_M))
    {
        /* Matfile logging */
        // rt_UpdateTXYLogVars(Linear_Model_M->rtwLogInfo, (Linear_Model_M->Timing.t));
    } /* end MajorTimeStep */

    if (rtmIsMajorTimeStep(Linear_Model_M))
    {
        /* signal main to stop simulation */
        { /* Sample time: [0.0s, 0.0s] */
            if ((rtmGetTFinal(Linear_Model_M) != -1) &&
                !((rtmGetTFinal(Linear_Model_M) - (((Linear_Model_M->Timing.clockTick1 +
                                                     Linear_Model_M->Timing.clockTickH1 * 4294967296.0)) *
                                                   0.2)) >
                  (((Linear_Model_M->Timing.clockTick1 +
                     Linear_Model_M->Timing.clockTickH1 * 4294967296.0)) *
                   0.2) *
                      (DBL_EPSILON)))
            {
                rtmSetErrorStatus(Linear_Model_M, "Simulation finished");
            }
        }

        rt_ertODEUpdateContinuousStates(&Linear_Model_M->solverInfo);

        /* Update absolute time for base rate */
        /* The "clockTick0" counts the number of times the code of this task has
         * been executed. The absolute time is the multiplication of "clockTick0"
         * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
         * overflow during the application lifespan selected.
         * Timer of this task consists of two 32 bit unsigned integers.
         * The two integers represent the low bits Timing.clockTick0 and the high bits
         * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
         */
        if (!(++Linear_Model_M->Timing.clockTick0))
        {
            ++Linear_Model_M->Timing.clockTickH0;
        }

        Linear_Model_M->Timing.t[0] = rtsiGetSolverStopTime(&Linear_Model_M->solverInfo);

        {
            /* Update absolute timer for sample time: [0.2s, 0.0s] */
            /* The "clockTick1" counts the number of times the code of this task has
             * been executed. The resolution of this integer timer is 0.2, which is the step size
             * of the task. Size of "clockTick1" ensures timer will not overflow during the
             * application lifespan selected.
             * Timer of this task consists of two 32 bit unsigned integers.
             * The two integers represent the low bits Timing.clockTick1 and the high bits
             * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
             */
            Linear_Model_M->Timing.clockTick1++;
            if (!Linear_Model_M->Timing.clockTick1)
            {
                Linear_Model_M->Timing.clockTickH1++;
            }
        }
    } /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void Linear_Model_derivatives(void)
{
    XDot_Linear_Model_T *_rtXdot;
    _rtXdot = ((XDot_Linear_Model_T *)Linear_Model_M->derivs);

    /* Derivatives for Integrator: '<S5>/Integrator' */
    _rtXdot->VyYaw_rate[0] = Linear_Model_B.dx[0];
    _rtXdot->VyYaw_rate[1] = Linear_Model_B.dx[1];

    /* Derivatives for Integrator: '<S37>/Integrator' */
    _rtXdot->Integrator_CSTATE = Linear_Model_B.IProdOut;

    /* Derivatives for Integrator: '<S32>/Filter' */
    _rtXdot->Filter_CSTATE = Linear_Model_B.NProdOut;
}

/* Model initialize function */
void Linear_Model_initialize(void)
{
    /* Registration code */

    /* initialize non-finites */
    rt_InitInfAndNaN(sizeof(real_T));

    /* initialize real-time model */
    (void)memset((void *)Linear_Model_M, 0,
                 sizeof(RT_MODEL_Linear_Model_T));

    {
        /* Setup solver object */
        rtsiSetSimTimeStepPtr(&Linear_Model_M->solverInfo,
                              &Linear_Model_M->Timing.simTimeStep);
        rtsiSetTPtr(&Linear_Model_M->solverInfo, &rtmGetTPtr(Linear_Model_M));
        rtsiSetStepSizePtr(&Linear_Model_M->solverInfo,
                           &Linear_Model_M->Timing.stepSize0);
        rtsiSetdXPtr(&Linear_Model_M->solverInfo, &Linear_Model_M->derivs);
        rtsiSetContStatesPtr(&Linear_Model_M->solverInfo, (real_T **)&Linear_Model_M->contStates);
        rtsiSetNumContStatesPtr(&Linear_Model_M->solverInfo,
                                &Linear_Model_M->Sizes.numContStates);
        rtsiSetNumPeriodicContStatesPtr(&Linear_Model_M->solverInfo,
                                        &Linear_Model_M->Sizes.numPeriodicContStates);
        rtsiSetPeriodicContStateIndicesPtr(&Linear_Model_M->solverInfo,
                                           &Linear_Model_M->periodicContStateIndices);
        rtsiSetPeriodicContStateRangesPtr(&Linear_Model_M->solverInfo,
                                          &Linear_Model_M->periodicContStateRanges);
        rtsiSetErrorStatusPtr(&Linear_Model_M->solverInfo, (&rtmGetErrorStatus(Linear_Model_M)));
        rtsiSetRTModelPtr(&Linear_Model_M->solverInfo, Linear_Model_M);
    }

    rtsiSetSimTimeStep(&Linear_Model_M->solverInfo, MAJOR_TIME_STEP);
    Linear_Model_M->intgData.y = Linear_Model_M->odeY;
    Linear_Model_M->intgData.f[0] = Linear_Model_M->odeF[0];
    Linear_Model_M->intgData.f[1] = Linear_Model_M->odeF[1];
    Linear_Model_M->intgData.f[2] = Linear_Model_M->odeF[2];
    Linear_Model_M->contStates = ((X_Linear_Model_T *)&Linear_Model_X);
    rtsiSetSolverData(&Linear_Model_M->solverInfo, (void *)&Linear_Model_M->intgData);
    rtsiSetIsMinorOutputWithModeChange(&Linear_Model_M->solverInfo, false);
    rtsiSetSolverName(&Linear_Model_M->solverInfo, "ode3");
    rtmSetTPtr(Linear_Model_M, &Linear_Model_M->Timing.tArray[0]);
    rtmSetTFinal(Linear_Model_M, 10.0);
    Linear_Model_M->Timing.stepSize0 = 0.2;

    // /* Setup for data logging */
    // {
    //   static RTWLogInfo rt_DataLoggingInfo;
    //   rt_DataLoggingInfo.loggingInterval = (NULL);
    //   Linear_Model_M->rtwLogInfo = &rt_DataLoggingInfo;
    // }

    // /* Setup for data logging */
    // {
    //   rtliSetLogXSignalInfo(Linear_Model_M->rtwLogInfo, (NULL));
    //   rtliSetLogXSignalPtrs(Linear_Model_M->rtwLogInfo, (NULL));
    //   rtliSetLogT(Linear_Model_M->rtwLogInfo, "tout");
    //   rtliSetLogX(Linear_Model_M->rtwLogInfo, "");
    //   rtliSetLogXFinal(Linear_Model_M->rtwLogInfo, "");
    //   rtliSetLogVarNameModifier(Linear_Model_M->rtwLogInfo, "rt_");
    //   rtliSetLogFormat(Linear_Model_M->rtwLogInfo, 0);
    //   rtliSetLogMaxRows(Linear_Model_M->rtwLogInfo, 0);
    //   rtliSetLogDecimation(Linear_Model_M->rtwLogInfo, 1);

    //   /*
    //    * Set pointers to the data and signal info for each output
    //    */
    //   {
    //     static void * rt_LoggedOutputSignalPtrs[] = {
    //       &Linear_Model_Y.right_Motor_Control,
    //       &Linear_Model_Y.left_Motor_Control
    //     };

    //     rtliSetLogYSignalPtrs(Linear_Model_M->rtwLogInfo, ((LogSignalPtrsType)
    //       rt_LoggedOutputSignalPtrs));
    //   }

    //   {
    //     static int_T rt_LoggedOutputWidths[] = {
    //       1,
    //       1
    //     };

    //     static int_T rt_LoggedOutputNumDimensions[] = {
    //       1,
    //       1
    //     };

    //     static int_T rt_LoggedOutputDimensions[] = {
    //       1,
    //       1
    //     };

    //     static boolean_T rt_LoggedOutputIsVarDims[] = {
    //       0,
    //       0
    //     };

    //     static void* rt_LoggedCurrentSignalDimensions[] = {
    //       (NULL),
    //       (NULL)
    //     };

    //     static int_T rt_LoggedCurrentSignalDimensionsSize[] = {
    //       4,
    //       4
    //     };

    //     static BuiltInDTypeId rt_LoggedOutputDataTypeIds[] = {
    //       SS_DOUBLE,
    //       SS_DOUBLE
    //     };

    //     static int_T rt_LoggedOutputComplexSignals[] = {
    //       0,
    //       0
    //     };

    //     static RTWPreprocessingFcnPtr rt_LoggingPreprocessingFcnPtrs[] = {
    //       (NULL),
    //       (NULL)
    //     };

    //     static const char_T *rt_LoggedOutputLabels[] = {
    //       "",
    //       "" };

    //     static const char_T *rt_LoggedOutputBlockNames[] = {
    //       "Linear_Model/right_Motor_Control",
    //       "Linear_Model/left_Motor_Control" };

    //     static RTWLogDataTypeConvert rt_RTWLogDataTypeConvert[] = {
    //       { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

    //       { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 }
    //     };

    //     static RTWLogSignalInfo rt_LoggedOutputSignalInfo[] = {
    //       {
    //         2,
    //         rt_LoggedOutputWidths,
    //         rt_LoggedOutputNumDimensions,
    //         rt_LoggedOutputDimensions,
    //         rt_LoggedOutputIsVarDims,
    //         rt_LoggedCurrentSignalDimensions,
    //         rt_LoggedCurrentSignalDimensionsSize,
    //         rt_LoggedOutputDataTypeIds,
    //         rt_LoggedOutputComplexSignals,
    //         (NULL),
    //         rt_LoggingPreprocessingFcnPtrs,

    //         { rt_LoggedOutputLabels },
    //         (NULL),
    //         (NULL),
    //         (NULL),

    //         { rt_LoggedOutputBlockNames },

    //         { (NULL) },
    //         (NULL),
    //         rt_RTWLogDataTypeConvert
    //       }
    //     };

    //     rtliSetLogYSignalInfo(Linear_Model_M->rtwLogInfo,
    //                           rt_LoggedOutputSignalInfo);

    //     /* set currSigDims field */
    //     rt_LoggedCurrentSignalDimensions[0] = &rt_LoggedOutputWidths[0];
    //     rt_LoggedCurrentSignalDimensions[1] = &rt_LoggedOutputWidths[1];
    //   }

    //   rtliSetLogY(Linear_Model_M->rtwLogInfo, "yout");
    // }

    /* block I/O */
    (void)memset(((void *)&Linear_Model_B), 0,
                 sizeof(B_Linear_Model_T));

    /* states (continuous) */
    {
        (void)memset((void *)&Linear_Model_X, 0,
                     sizeof(X_Linear_Model_T));
    }

    /* external inputs */
    (void)memset(&Linear_Model_U, 0, sizeof(ExtU_Linear_Model_T));

    /* external outputs */
    (void)memset(&Linear_Model_Y, 0, sizeof(ExtY_Linear_Model_T));

    /* Matfile logging */
    // rt_StartDataLoggingWithStartTime(Linear_Model_M->rtwLogInfo, 0.0, rtmGetTFinal
    //   (Linear_Model_M), Linear_Model_M->Timing.stepSize0, (&rtmGetErrorStatus
    //   (Linear_Model_M)));

    /* InitializeConditions for Integrator: '<S5>/Integrator' */
    Linear_Model_X.VyYaw_rate[0] = Linear_Model_P.Integrator_IC;
    Linear_Model_X.VyYaw_rate[1] = Linear_Model_P.Integrator_IC;

    /* InitializeConditions for Integrator: '<S37>/Integrator' */
    Linear_Model_X.Integrator_CSTATE =
        Linear_Model_P.PIDController_InitialConditio_e;

    /* InitializeConditions for Integrator: '<S32>/Filter' */
    Linear_Model_X.Filter_CSTATE = Linear_Model_P.PIDController_InitialConditionF;
}

/* Model terminate function */
void Linear_Model_terminate(void)
{
    /* (no terminate code required) */
}

/////////////// I2C FUNCS ////////////////
static void mcp4725_task() {
    static const char *TAG = "mcp4725";

    mcp4725_config_t dac_left = {
        .i2c_bus = I2C_NUM_0,
        .address = MCP4725_LEFT_ADDRESS,
        .ticks_to_wait = MCP4725_MAX_TICKS}; // dac_left configuration

    mcp4725_config_t dac_right = {
        .i2c_bus = I2C_NUM_0,
        .address = MCP4725_RIGHT_ADDRESS,
        .ticks_to_wait = MCP4725_MAX_TICKS}; // dac_right configuration

    mcp4725_eeprom_t eeprom_write = {
        .power_down_mode = MCP4725_POWER_DOWN_100, // power down, 100K resistor, 2
        .input_data = 2047};                       // values to write to eeprom, will be set on reboot

    mcp4725_eeprom_t eeprom_read_left;  // structure to hold the eeprom after we read it
    mcp4725_eeprom_t eeprom_read_right; // structure to hold the eeprom after we read it

    // ESP_ERROR_CHECK(mcp4725_write_eeprom(dac_left,eeprom_write)); // write the above configuration
    ESP_ERROR_CHECK(mcp4725_write_eeprom(dac_right, eeprom_write));      // write the above configuration
    vTaskDelay(50 / portTICK_PERIOD_MS);                                 // wait for device
    ESP_ERROR_CHECK(mcp4725_read_eeprom(dac_left, &eeprom_read_left));   // read the eeprom
    ESP_ERROR_CHECK(mcp4725_read_eeprom(dac_right, &eeprom_read_right)); // read the eeprom

    ESP_LOGI(TAG, "Power_Down_Mode Left Saved: %i", eeprom_read_left.power_down_mode);   // print the saved configuration
    ESP_LOGI(TAG, "Power_Down_Mode Right Saved: %i", eeprom_read_right.power_down_mode); // print the saved configuration
    ESP_LOGI(TAG, "Input_Data Left Saved: %i", eeprom_read_left.input_data);
    ESP_LOGI(TAG, "Input_Data Right Saved: %i", eeprom_read_right.input_data);

    // Continuously write values as they arrive via UART
    while (1) {
        int messageData[4] = {0};
        // if xQueueReceive(dac_left_evt_queue_send, &messageData, 100 / portTICK_PERIOD_MS)) {
        ESP_ERROR_CHECK(mcp4725_set_voltage(dac_left, thr_left_scaled * (double)4095));  // scale from 0-1 to 0-4095 = 5v
        ESP_ERROR_CHECK(mcp4725_set_voltage(dac_right, thr_rigt_scaled * (double)4095)); // scale from 0-1 to 0-4095 = 5v
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
    ESP_LOGI("i2c_bus", "SCL=%i SDA=%i CLK_SPEED=%i", SCL_PIN, SDA_PIN, I2C_CLK_SPEED);
}

///////////////// UART FUNCS //////////////////
static void uart_init() {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
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
    static const char *RX_TASK_TAG = "RX_TASK";
    char *data_in = (char *)malloc(BUF_SIZE);

    // To calculate speed using accelerometer
    float t = 0.1;
    float vx_current = 7.0;
    float vy_current = 7.0;
    while (1) {
        int len_in = uart_read_bytes(UART_PORT_NUM, data_in, (BUF_SIZE - 1), 100 / portTICK_PERIOD_MS);
        if (len_in > 0) {
            data_in[len_in] = '\0';
            float x_dir, y_dir, z_dir, steering, throttle;
            sscanf(data_in, "%f,%f,%f,%f,%f", &x_dir, &y_dir, &z_dir, &steering, &throttle);
            printf("Steering: %f\n", steering);
            printf("Throttle: %f\n", throttle);
            float vx_next = vx_current + x_dir * t;
            float vy_next = vy_current + y_dir * t;
            vx_current = vx_next;
            vy_current = vy_next;
            float current_speed_input = sqrt(vx_next * vx_next + vy_next * vy_next);
            throttle_scaled = throttle / 1024;
            steering_scaled = (steering / 1024 - 0.5) * 2 * 90 / 180 * 3.14;
            // speed_scaled = current_speed_input;
            printf("steering_scaled = %f, throttle_scaled = %f, speed_scaled = %f\n", steering_scaled, throttle_scaled, speed_scaled);
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

    Linear_Model_initialize();

    while (1) {
        int messageData[4] = {0};
        Linear_Model_step();
        // if (xQueueReceive(uart_evt_queue_send, &messageData, 100 / portTICK_PERIOD_MS)) {
        // thr_left_scaled = throttle_scaled + ((steering_scaled < 0) ? steering_scaled : 0);
        // thr_rigt_scaled = throttle_scaled - ((steering_scaled > 0) ? steering_scaled : 0);
        // thr_left_scaled = throttle_scaled;
        // thr_rigt_scaled = throttle_scaled;
        // In testing mode, the throttle signal is stepped progressively from 0-1
        if (!testing_mode) {
            thr_left_scaled = Linear_Model_Y.left_Motor_Control;
            thr_rigt_scaled = Linear_Model_Y.right_Motor_Control;
        }

        // Notify dac_left
        // xQueueSend(dac_left_evt_queue_send, &mv_thr_send, 10);
        vTaskDelay(pdMS_TO_TICKS(100));
        printf("Linear Left: %.2f, linear right: %.2f\n", thr_left_scaled, thr_rigt_scaled);
    }

    Linear_Model_terminate();
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
We will not be processing any outgoing CAN bus commands, only recieving very select commands, namely
0x20
0x22, fault codes only
0x24, with some variables left blank (0xFFFF)
*/
void can_init() {
    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500 Kbit/s
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    }
    else {
        printf("Failed to install driver\n");
        return;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    }
    else {
        printf("Failed to start driver\n");
        return;
    }
}

void can_tx_task()
{
    // Incoming format: Packet ID | Node ID (Ex) | Data Bytes
    int txBuffer[10];
    while (1) {
        if (xQueueReceive(can_tx_queue, &(txBuffer), (TickType_t)5)) {
            // Configure message to transmit
            twai_message_t message;
            message.identifier = txBuffer[0] * pow(2, 8) + txBuffer[1]; // PacketID and NodeID
            message.extd = 1;                                           // Extended format
            message.data_length_code = 8;                               // Length of message in bytes, always 8 for sending
            for (int i = 0; i < message.data_length_code; i++) {
                message.data[i] = txBuffer[i + 2];
            }

            // Queue message for transmission
            if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
                printf("Message queued for transmission\n");
            }
            else {
                printf("Failed to queue message for transmission\n");
            }
        }
        else {
            printf("No message queued for transmission right now\n");
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
        }
        else {
            printf("Failed to receive message after %d seconds\n", waitticks / 1000);
        }

        // Process received message
        int packetID = 0;
        int nodeID = 0;

        if (message.extd) {
            printf("Message is in Extended Format\n");
            packetID = message.identifier >> 8;
            nodeID = (int) message.identifier % (int) pow(2, 8);
        }
        else {
            printf("Message is in Standard Format\n");
            packetID = message.identifier >> 5;
            nodeID = (int) message.identifier % (int) pow(2, 5);
        }
        printf("MessageID:\t%ld\tPacketID:\t%d\nNodeID:\t%d\t", message.identifier, packetID, nodeID);
        if (!(message.rtr)) {
            // Process each byte
            if (packetID == 0x20) {
                if (nodeID == MC_LEFT_ID)
                    RPM_L = (message.data[0]<<24) + (message.data[1]<<16) + (message.data[2]<<8) + message.data[3];
                if (nodeID == MC_RIGHT_ID)
                    RPM_R = (message.data[0]<<24) + (message.data[1]<<16) + (message.data[2]<<8) + message.data[3];
                speed_scaled = (double) (RPM_L+RPM_R)/2/SIM_RATIO*SIM_WHEEL_DIAM*3.1415/60;
                printf("RPM_L:\t%d\tRPM_R:\t%d\tSpeed:\t%f\n",RPM_L,RPM_R, speed_scaled);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void can_alert_task() {
    // Reconfigure alerts to detect Error Passive and Bus-Off error states////////////
    uint32_t alerts_to_enable = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        printf("Alerts reconfigured\n");
    }
    else {
        printf("Failed to reconfigure alerts");
    }

    while (1) {
        // Block indefinitely until an alert occurs
        uint32_t alerts_triggered;
        twai_read_alerts(&alerts_triggered, portMAX_DELAY);
    }
}

///////////////// DEBUGGING/TESTING DUMMY FUNCS //////////////////
double calc_speed(double speed, double t_left, double t_right) {
    double FL = (double)72 * SIM_RATIO / (SIM_WHEEL_DIAM / 2) * t_left;
    double FR = (double)72 * SIM_RATIO / (SIM_WHEEL_DIAM / 2) * t_right;
    speed = (double)speed + (FR + FL) * ((double)SIM_TIMESTEP / 1000) / ((double)2 * SIM_WEIGHT) - ((double)SIM_ROLL_COEF * SIM_WEIGHT + 0.5 * SIM_AIR_DENSITY * SIM_DRAG_COEF * SIM_FRONT_AREA * speed * speed) * ((double)SIM_TIMESTEP / 1000) / (double)SIM_WEIGHT;
    // printf("Force Left\t%.3f\tRight\t%.3f\n", FL, FR);

    return speed;
}

static void speed_emulator_task() {
    // ALG
    double speed = 0;
    while (1) {
        if (testing_mode) {
            speed = calc_speed(speed, thr_left_scaled, thr_rigt_scaled);
            printf("Throttle Left\t%.3f\tRight\t%.3f\n", thr_left_scaled, thr_rigt_scaled);
            printf("Speed: %f m\\s\n", speed);
            vTaskDelay(pdMS_TO_TICKS(SIM_TIMESTEP));
        }
    }
}

void throttle_emulator_task() {
    int i = 0;
    while (1) {
        thr_left_scaled = (float)i / 10.0;
        thr_rigt_scaled = (float)1.0 - (i / 10.0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        i = (i + 1) % 11;
    }
}

///////////////// MAIN FUNC //////////////////
void app_main() {
    i2c_setup();
    gpio_init();
    uart_init();
    can_init();

    // xTaskCreate(mcp4725_task,"mcp4725_task",2048,NULL,5,NULL);
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);
    xTaskCreate(temp_PID_task, "temp_PID_task", 4096, NULL, 5, NULL);
    // xTaskCreate(speed_emulator_task,"speed_emulator_task",4096,NULL,5,NULL);
    // xTaskCreate(can_tx_task,"can_tx_task",4096,NULL,5,NULL);
    xTaskCreate(can_rx_task,"can_rx_task",4096,NULL,5,NULL);
    xTaskCreate(can_alert_task,"can_alert_task",4096,NULL,5,NULL);

    if (testing_mode)
        xTaskCreate(throttle_emulator_task, "throttle_emulator_task", 4096, NULL, 5, NULL);
}