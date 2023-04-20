/*
 * Linear_Model.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Linear_Model".
 *
 * Model version              : 1.14
 * Simulink Coder version : 9.7 (R2022a) 13-Nov-2021
 * C source code generated on : Wed Apr 19 00:49:09 2023
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "Linear_Model.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "Linear_Model_private.h"
#include <string.h>

/* Block signals (default storage) */
B_Linear_Model_T Linear_Model_B;

/* Continuous states */
X_Linear_Model_T Linear_Model_X;

/* Block states (default storage) */
DW_Linear_Model_T Linear_Model_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Linear_Model_T Linear_Model_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Linear_Model_T Linear_Model_Y;

/* Real-time model */
static RT_MODEL_Linear_Model_T Linear_Model_M_;
RT_MODEL_Linear_Model_T *const Linear_Model_M = &Linear_Model_M_;
real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
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
  } else if (u0 < bp0[maxIndex]) {
    uint32_T bpIdx;
    uint32_T iRght;

    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
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
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

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
  int_T nXc = 2;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  Linear_Model_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  Linear_Model_step();
  Linear_Model_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  Linear_Model_step();
  Linear_Model_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void Linear_Model_step(void)
{
  real_T yawRate;
  real_T yawRate_max;
  if (rtmIsMajorTimeStep(Linear_Model_M)) {
    /* set solver stop time */
    if (!(Linear_Model_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&Linear_Model_M->solverInfo,
                            ((Linear_Model_M->Timing.clockTickH0 + 1) *
        Linear_Model_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&Linear_Model_M->solverInfo,
                            ((Linear_Model_M->Timing.clockTick0 + 1) *
        Linear_Model_M->Timing.stepSize0 + Linear_Model_M->Timing.clockTickH0 *
        Linear_Model_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(Linear_Model_M)) {
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
  yawRate_max = 9.81 / Linear_Model_U.V_CG;

  /* :  yawRate = V_CG/((l_r+l_f)+K_u*V_CG^2) * steering_Angle; */
  yawRate = Linear_Model_U.V_CG / (Linear_Model_U.V_CG * Linear_Model_U.V_CG *
    0.05 + 1.596) * (Linear_Model_U.SteeringAngleEncoder * 2.0 * 90.0 / 180.0 *
                     3.1415926535897931);

  /* :  if (abs(yawRate) <= abs(yawRate_max)) */
  if (fabs(yawRate) <= fabs(yawRate_max)) {
    /* :  des_yawRate = yawRate; */
    Linear_Model_B.des_yawRate = yawRate;
  } else {
    /* :  else */
    /* :  if (sign(yawRate) == 1) */
    if (!rtIsNaN(yawRate)) {
      if (yawRate < 0.0) {
        yawRate = -1.0;
      } else {
        yawRate = (yawRate > 0.0);
      }
    }

    if (yawRate == 1.0) {
      /* :  des_yawRate = yawRate_max; */
      Linear_Model_B.des_yawRate = yawRate_max;
    } else {
      /* :  else */
      /* :  des_yawRate = -yawRate_max; */
      Linear_Model_B.des_yawRate = -yawRate_max;
    }
  }

  /* End of MATLAB Function: '<Root>/Desired_yawRate' */
  if (rtmIsMajorTimeStep(Linear_Model_M)) {
    /* DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn' */
    yawRate_max = Linear_Model_DW.DiscreteTransferFcn_states[0] *
      Linear_Model_P.DiscreteTransferFcn_NumCoef[1];
    yawRate_max += Linear_Model_DW.DiscreteTransferFcn_states[1] *
      Linear_Model_P.DiscreteTransferFcn_NumCoef[2];

    /* DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn' */
    Linear_Model_B.DiscreteTransferFcn = yawRate_max;
  }

  /* Sum: '<Root>/Sum' */
  Linear_Model_B.Sum = Linear_Model_B.des_yawRate -
    Linear_Model_B.DiscreteTransferFcn;

  /* Lookup_n-D: '<Root>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/V_CG'
   */
  Linear_Model_B.uDLookupTable = look1_binlxpw(Linear_Model_U.V_CG,
    Linear_Model_P.uDLookupTable_bp01Data,
    Linear_Model_P.uDLookupTable_tableData, 6U);

  /* Product: '<S40>/PProd Out' */
  Linear_Model_B.PProdOut = Linear_Model_B.Sum * Linear_Model_B.uDLookupTable;

  /* Integrator: '<S35>/Integrator' */
  Linear_Model_B.Integrator = Linear_Model_X.Integrator_CSTATE;

  /* Product: '<S29>/DProd Out' incorporates:
   *  Constant: '<Root>/Constant3'
   */
  Linear_Model_B.DProdOut = Linear_Model_B.Sum * Linear_Model_P.Constant3_Value;

  /* Integrator: '<S30>/Filter' */
  Linear_Model_B.Filter = Linear_Model_X.Filter_CSTATE;

  /* Sum: '<S30>/SumD' */
  Linear_Model_B.SumD = Linear_Model_B.DProdOut - Linear_Model_B.Filter;

  /* Product: '<S38>/NProd Out' */
  Linear_Model_B.NProdOut = Linear_Model_B.SumD * 0.0;

  /* Sum: '<S44>/Sum' */
  Linear_Model_B.Sum_h = (Linear_Model_B.PProdOut + Linear_Model_B.Integrator) +
    Linear_Model_B.NProdOut;

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Constant: '<Root>/Max Toque of Motors (N*m)'
   *  Constant: '<Root>/Max Voltage'
   *  Inport: '<Root>/Steering Angle Encoder'
   *  Inport: '<Root>/throttle_Input'
   */
  /* :  if (sign(steering_In) == 1) */
  yawRate_max = Linear_Model_U.SteeringAngleEncoder;
  if (!rtIsNaN(yawRate_max)) {
    if (yawRate_max < 0.0) {
      yawRate_max = -1.0;
    } else {
      yawRate_max = (yawRate_max > 0.0);
    }
  }

  if (yawRate_max == 1.0) {
    /* :  right_Motor_Control = throttle_Input*motor_Control_Voltage_Max; */
    yawRate = Linear_Model_U.throttle_Input * Linear_Model_P.MaxVoltage_Value;

    /* :  left_Motor_Control = right_Motor_Control - torque_Diff/max_Torque_Scale*motor_Control_Voltage_Max; */
    yawRate_max = yawRate - Linear_Model_B.Sum_h /
      Linear_Model_P.MaxToqueofMotorsNm_Value * Linear_Model_P.MaxVoltage_Value;

    /* :  if left_Motor_Control < 0 */
    if (yawRate_max < 0.0) {
      /* :  left_Motor_Control = 0; */
      yawRate_max = 0.0;
    }
  } else {
    /* :  else */
    /* :  left_Motor_Control = throttle_Input*motor_Control_Voltage_Max; */
    yawRate_max = Linear_Model_U.throttle_Input *
      Linear_Model_P.MaxVoltage_Value;

    /* :  right_Motor_Control = left_Motor_Control - torque_Diff/max_Torque_Scale*motor_Control_Voltage_Max; */
    yawRate = yawRate_max - Linear_Model_B.Sum_h /
      Linear_Model_P.MaxToqueofMotorsNm_Value * Linear_Model_P.MaxVoltage_Value;

    /* :  if right_Motor_Control < 0 */
    if (yawRate < 0.0) {
      /* :  right_Motor_Control = 0; */
      yawRate = 0.0;
    }
  }

  /* :  left_Motor_Control_out = left_Motor_Control; */
  Linear_Model_B.left_Motor_Control_out = yawRate_max;

  /* :  right_Motor_Control_out = right_Motor_Control; */
  Linear_Model_B.right_Motor_Control_out = yawRate;

  /* End of MATLAB Function: '<Root>/MATLAB Function' */

  /* Outport: '<Root>/right_Motor_Control_out' */
  Linear_Model_Y.right_Motor_Control_out =
    Linear_Model_B.right_Motor_Control_out;

  /* Outport: '<Root>/left_Motor_Control_out' */
  Linear_Model_Y.left_Motor_Control_out = Linear_Model_B.left_Motor_Control_out;

  /* Lookup_n-D: '<Root>/1-D Lookup Table1' incorporates:
   *  Inport: '<Root>/V_CG'
   */
  Linear_Model_B.uDLookupTable1 = look1_binlxpw(Linear_Model_U.V_CG,
    Linear_Model_P.uDLookupTable1_bp01Data,
    Linear_Model_P.uDLookupTable1_tableData, 6U);

  /* Product: '<S32>/IProd Out' */
  Linear_Model_B.IProdOut = Linear_Model_B.Sum * Linear_Model_B.uDLookupTable1;
  if (rtmIsMajorTimeStep(Linear_Model_M)) {
    /* Matfile logging */
    // rt_UpdateTXYLogVars(Linear_Model_M->rtwLogInfo, (Linear_Model_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(Linear_Model_M)) {
    if (rtmIsMajorTimeStep(Linear_Model_M)) {
      real_T denAccum;

      /* Update for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn' */
      denAccum = Linear_Model_B.Sum_h;
      denAccum -= Linear_Model_DW.DiscreteTransferFcn_states[0] *
        Linear_Model_P.DiscreteTransferFcn_DenCoef[1];
      denAccum -= Linear_Model_DW.DiscreteTransferFcn_states[1] *
        Linear_Model_P.DiscreteTransferFcn_DenCoef[2];
      denAccum /= Linear_Model_P.DiscreteTransferFcn_DenCoef[0];
      Linear_Model_DW.DiscreteTransferFcn_states[1] =
        Linear_Model_DW.DiscreteTransferFcn_states[0];
      Linear_Model_DW.DiscreteTransferFcn_states[0] = denAccum;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(Linear_Model_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(Linear_Model_M)!=-1) &&
          !((rtmGetTFinal(Linear_Model_M)-(((Linear_Model_M->Timing.clockTick1+
               Linear_Model_M->Timing.clockTickH1* 4294967296.0)) * 0.2)) >
            (((Linear_Model_M->Timing.clockTick1+
               Linear_Model_M->Timing.clockTickH1* 4294967296.0)) * 0.2) *
            (DBL_EPSILON))) {
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
    if (!(++Linear_Model_M->Timing.clockTick0)) {
      ++Linear_Model_M->Timing.clockTickH0;
    }

    Linear_Model_M->Timing.t[0] = rtsiGetSolverStopTime
      (&Linear_Model_M->solverInfo);

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
      if (!Linear_Model_M->Timing.clockTick1) {
        Linear_Model_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void Linear_Model_derivatives(void)
{
  XDot_Linear_Model_T *_rtXdot;
  _rtXdot = ((XDot_Linear_Model_T *) Linear_Model_M->derivs);

  /* Derivatives for Integrator: '<S35>/Integrator' */
  _rtXdot->Integrator_CSTATE = Linear_Model_B.IProdOut;

  /* Derivatives for Integrator: '<S30>/Filter' */
  _rtXdot->Filter_CSTATE = Linear_Model_B.NProdOut;
}

/* Model initialize function */
void Linear_Model_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)Linear_Model_M, 0,
                sizeof(RT_MODEL_Linear_Model_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&Linear_Model_M->solverInfo,
                          &Linear_Model_M->Timing.simTimeStep);
    rtsiSetTPtr(&Linear_Model_M->solverInfo, &rtmGetTPtr(Linear_Model_M));
    rtsiSetStepSizePtr(&Linear_Model_M->solverInfo,
                       &Linear_Model_M->Timing.stepSize0);
    rtsiSetdXPtr(&Linear_Model_M->solverInfo, &Linear_Model_M->derivs);
    rtsiSetContStatesPtr(&Linear_Model_M->solverInfo, (real_T **)
                         &Linear_Model_M->contStates);
    rtsiSetNumContStatesPtr(&Linear_Model_M->solverInfo,
      &Linear_Model_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&Linear_Model_M->solverInfo,
      &Linear_Model_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&Linear_Model_M->solverInfo,
      &Linear_Model_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&Linear_Model_M->solverInfo,
      &Linear_Model_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&Linear_Model_M->solverInfo, (&rtmGetErrorStatus
      (Linear_Model_M)));
    rtsiSetRTModelPtr(&Linear_Model_M->solverInfo, Linear_Model_M);
  }

  rtsiSetSimTimeStep(&Linear_Model_M->solverInfo, MAJOR_TIME_STEP);
  Linear_Model_M->intgData.y = Linear_Model_M->odeY;
  Linear_Model_M->intgData.f[0] = Linear_Model_M->odeF[0];
  Linear_Model_M->intgData.f[1] = Linear_Model_M->odeF[1];
  Linear_Model_M->intgData.f[2] = Linear_Model_M->odeF[2];
  Linear_Model_M->contStates = ((X_Linear_Model_T *) &Linear_Model_X);
  rtsiSetSolverData(&Linear_Model_M->solverInfo, (void *)
                    &Linear_Model_M->intgData);
  rtsiSetIsMinorOutputWithModeChange(&Linear_Model_M->solverInfo, false);
  rtsiSetSolverName(&Linear_Model_M->solverInfo,"ode3");
  rtmSetTPtr(Linear_Model_M, &Linear_Model_M->Timing.tArray[0]);
  rtmSetTFinal(Linear_Model_M, 10.0);
  Linear_Model_M->Timing.stepSize0 = 0.2;

  // /* Setup for data logging */
  // {
    // static RTWLogInfo rt_DataLoggingInfo;
    // rt_DataLoggingInfo.loggingInterval = (NULL);
    // Linear_Model_M->rtwLogInfo = &rt_DataLoggingInfo;
  // }

  // /* Setup for data logging */
  // {
    // rtliSetLogXSignalInfo(Linear_Model_M->rtwLogInfo, (NULL));
    // rtliSetLogXSignalPtrs(Linear_Model_M->rtwLogInfo, (NULL));
    // rtliSetLogT(Linear_Model_M->rtwLogInfo, "tout");
    // rtliSetLogX(Linear_Model_M->rtwLogInfo, "");
    // rtliSetLogXFinal(Linear_Model_M->rtwLogInfo, "");
    // rtliSetLogVarNameModifier(Linear_Model_M->rtwLogInfo, "rt_");
    // rtliSetLogFormat(Linear_Model_M->rtwLogInfo, 0);
    // rtliSetLogMaxRows(Linear_Model_M->rtwLogInfo, 0);
    // rtliSetLogDecimation(Linear_Model_M->rtwLogInfo, 1);

    // /*
     // * Set pointers to the data and signal info for each output
     // */
    // {
      // static void * rt_LoggedOutputSignalPtrs[] = {
        // &Linear_Model_Y.right_Motor_Control_out,
        // &Linear_Model_Y.left_Motor_Control_out
      // };

      // rtliSetLogYSignalPtrs(Linear_Model_M->rtwLogInfo, ((LogSignalPtrsType)
        // rt_LoggedOutputSignalPtrs));
    // }

    // {
      // static int_T rt_LoggedOutputWidths[] = {
        // 1,
        // 1
      // };

      // static int_T rt_LoggedOutputNumDimensions[] = {
        // 1,
        // 1
      // };

      // static int_T rt_LoggedOutputDimensions[] = {
        // 1,
        // 1
      // };

      // static boolean_T rt_LoggedOutputIsVarDims[] = {
        // 0,
        // 0
      // };

      // static void* rt_LoggedCurrentSignalDimensions[] = {
        // (NULL),
        // (NULL)
      // };

      // static int_T rt_LoggedCurrentSignalDimensionsSize[] = {
        // 4,
        // 4
      // };

      // static BuiltInDTypeId rt_LoggedOutputDataTypeIds[] = {
        // SS_DOUBLE,
        // SS_DOUBLE
      // };

      // static int_T rt_LoggedOutputComplexSignals[] = {
        // 0,
        // 0
      // };

      // static RTWPreprocessingFcnPtr rt_LoggingPreprocessingFcnPtrs[] = {
        // (NULL),
        // (NULL)
      // };

      // static const char_T *rt_LoggedOutputLabels[] = {
        // "",
        // "" };

      // static const char_T *rt_LoggedOutputBlockNames[] = {
        // "Linear_Model/right_Motor_Control_out",
        // "Linear_Model/left_Motor_Control_out" };

      // static RTWLogDataTypeConvert rt_RTWLogDataTypeConvert[] = {
        // { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 },

        // { 0, SS_DOUBLE, SS_DOUBLE, 0, 0, 0, 1.0, 0, 0.0 }
      // };

      // static RTWLogSignalInfo rt_LoggedOutputSignalInfo[] = {
        // {
          // 2,
          // rt_LoggedOutputWidths,
          // rt_LoggedOutputNumDimensions,
          // rt_LoggedOutputDimensions,
          // rt_LoggedOutputIsVarDims,
          // rt_LoggedCurrentSignalDimensions,
          // rt_LoggedCurrentSignalDimensionsSize,
          // rt_LoggedOutputDataTypeIds,
          // rt_LoggedOutputComplexSignals,
          // (NULL),
          // rt_LoggingPreprocessingFcnPtrs,

          // { rt_LoggedOutputLabels },
          // (NULL),
          // (NULL),
          // (NULL),

          // { rt_LoggedOutputBlockNames },

          // { (NULL) },
          // (NULL),
          // rt_RTWLogDataTypeConvert
        // }
      // };

      // rtliSetLogYSignalInfo(Linear_Model_M->rtwLogInfo,
                            // rt_LoggedOutputSignalInfo);

      // /* set currSigDims field */
      // rt_LoggedCurrentSignalDimensions[0] = &rt_LoggedOutputWidths[0];
      // rt_LoggedCurrentSignalDimensions[1] = &rt_LoggedOutputWidths[1];
    // }

    // rtliSetLogY(Linear_Model_M->rtwLogInfo, "yout");
  // }

  /* block I/O */
  (void) memset(((void *) &Linear_Model_B), 0,
                sizeof(B_Linear_Model_T));

  /* states (continuous) */
  {
    (void) memset((void *)&Linear_Model_X, 0,
                  sizeof(X_Linear_Model_T));
  }

  /* states (dwork) */
  (void) memset((void *)&Linear_Model_DW, 0,
                sizeof(DW_Linear_Model_T));

  /* external inputs */
  (void)memset(&Linear_Model_U, 0, sizeof(ExtU_Linear_Model_T));

  /* external outputs */
  (void)memset(&Linear_Model_Y, 0, sizeof(ExtY_Linear_Model_T));

  // /* Matfile logging */
  // rt_StartDataLoggingWithStartTime(Linear_Model_M->rtwLogInfo, 0.0, rtmGetTFinal
    // (Linear_Model_M), Linear_Model_M->Timing.stepSize0, (&rtmGetErrorStatus
    // (Linear_Model_M)));

  /* InitializeConditions for DiscreteTransferFcn: '<Root>/Discrete Transfer Fcn' */
  Linear_Model_DW.DiscreteTransferFcn_states[0] =
    Linear_Model_P.DiscreteTransferFcn_InitialStat;
  Linear_Model_DW.DiscreteTransferFcn_states[1] =
    Linear_Model_P.DiscreteTransferFcn_InitialStat;

  /* InitializeConditions for Integrator: '<S35>/Integrator' */
  Linear_Model_X.Integrator_CSTATE =
    Linear_Model_P.PIDController_InitialConditio_e;

  /* InitializeConditions for Integrator: '<S30>/Filter' */
  Linear_Model_X.Filter_CSTATE = Linear_Model_P.PIDController_InitialConditionF;
}

/* Model terminate function */
void Linear_Model_terminate(void)
{
  /* (no terminate code required) */
}