/*
 * Linear_Model.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Linear_Model".
 *
 * Model version              : 1.7
 * Simulink Coder version : 9.7 (R2022a) 13-Nov-2021
 * C source code generated on : Wed Mar  1 23:22:33 2023
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "Linear_Model.h"
#include "rtwtypes.h"
#include "Linear_Model_private.h"
#include "rt_nonfinite.h"

/* Block signals (default storage) */
B_Linear_Model_T Linear_Model_B;

/* Continuous states */
X_Linear_Model_T Linear_Model_X;

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
  int_T nXc = 4;
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
  real_T tmp;
  real_T tmp_0;
  real_T tmp_1;
  real_T tmp_2;
  real_T tmp_3;
  real_T tmp_4;
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

  if (rtmIsMajorTimeStep(Linear_Model_M)) {
    /* Lookup_n-D: '<Root>/1-D Lookup Table' incorporates:
     *  Constant: '<Root>/V_CG '
     */
    Linear_Model_B.uDLookupTable = look1_binlxpw(Linear_Model_P.V_CG_Value,
      Linear_Model_P.uDLookupTable_bp01Data,
      Linear_Model_P.uDLookupTable_tableData, 10U);

    /* Lookup_n-D: '<Root>/1-D Lookup Table1' incorporates:
     *  Constant: '<Root>/V_CG '
     */
    Linear_Model_B.uDLookupTable1 = look1_binlxpw(Linear_Model_P.V_CG_Value,
      Linear_Model_P.uDLookupTable1_bp01Data,
      Linear_Model_P.uDLookupTable1_tableData, 10U);

    /* Gain: '<Root>/Gain' incorporates:
     *  Constant: '<Root>/Steering angle encoder'
     */
    Linear_Model_B.Gain = Linear_Model_P.Gain_Gain *
      Linear_Model_P.Steeringangleencoder_Value;

    /* MATLAB Function: '<Root>/Desired_yawRate' incorporates:
     *  Constant: '<Root>/V_CG '
     */
    /* :  Cyf = 20; */
    /* :  Cyr = 1; */
    /* :  m = 500; */
    /* :  l_f = 0.813; */
    /* :  l_r = 0.783; */
    /* :  Izz = 190.701; */
    /* :  K_u = 1; */
    /* :  yawRate = V_CG/((l_r+l_f)+K_u+V_CG^2) * steering_Angle; */
    Linear_Model_B.yawRate = Linear_Model_P.V_CG_Value /
      (Linear_Model_P.V_CG_Value * Linear_Model_P.V_CG_Value + 2.596) *
      Linear_Model_B.Gain;

    /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
     *  Constant: '<Root>/V_CG '
     */
    /* :  Cyf = 20; */
    /* :  Cyr = 1; */
    /* :  m = 500; */
    /* :  l_f = 0.813; */
    /* :  l_r = 0.783; */
    /* :  Izz = 190.701; */
    /* :  k_u = 1; */
    /* :  A =[-(Cyf+Cyr)/(m*Vx0), (-l_f*Cyf+l_r*Cyr)/(m*Vx0)-Vx0; (-l_f*Cyf+l_r*Cyr)/... */
    /* :      (Izz*Vx0), -(l_f^2*Cyf+l_r^2*Cyr)/(Izz*Vx0)]; */
    Linear_Model_B.A_p[0] = -21.0 / (500.0 * Linear_Model_P.V_CG_Value);
    Linear_Model_B.A_p[2] = -15.476999999999999 / (500.0 *
      Linear_Model_P.V_CG_Value) - Linear_Model_P.V_CG_Value;
    Linear_Model_B.A_p[1] = -15.476999999999999 / (190.701 *
      Linear_Model_P.V_CG_Value);
    Linear_Model_B.A_p[3] = -13.832468999999998 / (190.701 *
      Linear_Model_P.V_CG_Value);

    /* :  B = [Cyf/(m*Vx0), 0; (l_f*Cyf)/Izz, 1/(0.05*Izz)]; */
    Linear_Model_B.B_g[0] = 20.0 / (500.0 * Linear_Model_P.V_CG_Value);
    Linear_Model_B.B_g[2] = 0.0;

    /* :  C = [1, 1]; */
    /* :  D = [0, 0]; */
    Linear_Model_B.B_g[1] = 0.085264366731165539;
    Linear_Model_B.C_m[0] = 1.0;
    Linear_Model_B.D_e[0] = 0.0;
    Linear_Model_B.B_g[3] = 0.10487621984153203;
    Linear_Model_B.C_m[1] = 1.0;
    Linear_Model_B.D_e[1] = 0.0;

    /* MATLAB Function: '<Root>/MATLAB Function2' */
    /* :  Cyf = 20; */
    /* :  Cyr = 1; */
    /* :  m = 500; */
    /* :  l_f = 0.813; */
    /* :  l_r = 0.783; */
    /* :  Izz = 190.701; */
    /* :  k_u = 1; */
    /* :  A =[-(Cyf+Cyr)/(m*Vx0), (-l_f*Cyf+l_r*Cyr)/(m*Vx0)-Vx0; (-l_f*Cyf+l_r*Cyr)/... */
    /* :      (Izz*Vx0), -(l_f^2*Cyf+l_r^2*Cyr)/(Izz*Vx0)]; */
    Linear_Model_B.A[0] = (rtMinusInf);
    Linear_Model_B.A[2] = (rtMinusInf);
    Linear_Model_B.A[1] = (rtMinusInf);
    Linear_Model_B.A[3] = (rtMinusInf);

    /* :  B = [Cyf/(m*Vx0), 0; (l_f*Cyf)/Izz, 1/(0.05*Izz)]; */
    Linear_Model_B.B[0] = (rtInf);
    Linear_Model_B.B[2] = 0.0;

    /* :  C = [1, 1]; */
    Linear_Model_B.B[1] = 0.085264366731165539;
    Linear_Model_B.C[0] = 1.0;
    Linear_Model_B.B[3] = 0.10487621984153203;
    Linear_Model_B.C[1] = 1.0;

    /* :  D = 0; */
    Linear_Model_B.D = 0.0;
  }

  /* Integrator: '<S5>/Integrator' */
  Linear_Model_B.x[0] = Linear_Model_X.VyYaw_rate[0];
  Linear_Model_B.x[1] = Linear_Model_X.VyYaw_rate[1];

  /* Sum: '<Root>/Sum' */
  Linear_Model_B.Sum = Linear_Model_B.yawRate - Linear_Model_B.x[1];

  /* Product: '<S31>/DProd Out' incorporates:
   *  Constant: '<Root>/Constant3'
   */
  Linear_Model_B.DProdOut = Linear_Model_B.Sum * Linear_Model_P.Constant3_Value;

  /* Integrator: '<S32>/Filter' */
  Linear_Model_B.Filter = Linear_Model_X.Filter_CSTATE;

  /* Sum: '<S32>/SumD' */
  Linear_Model_B.SumD = Linear_Model_B.DProdOut - Linear_Model_B.Filter;

  /* Product: '<S34>/IProd Out' */
  Linear_Model_B.IProdOut = Linear_Model_B.Sum * Linear_Model_B.uDLookupTable1;

  /* Integrator: '<S37>/Integrator' */
  Linear_Model_B.Integrator = Linear_Model_X.Integrator_CSTATE;

  /* Product: '<S40>/NProd Out' */
  Linear_Model_B.NProdOut = Linear_Model_B.SumD * 0.0;

  /* Product: '<S42>/PProd Out' */
  Linear_Model_B.PProdOut = Linear_Model_B.Sum * Linear_Model_B.uDLookupTable;

  /* Sum: '<S46>/Sum' */
  Linear_Model_B.Sum_h = (Linear_Model_B.PProdOut + Linear_Model_B.Integrator) +
    Linear_Model_B.NProdOut;

  /* SignalConversion generated from: '<S5>/Product' */
  Linear_Model_B.TmpSignalConversionAtProductInp[0] = 0.0;
  Linear_Model_B.TmpSignalConversionAtProductInp[1] = Linear_Model_B.Sum_h;

  /* Product: '<S5>/Product' */
  tmp = Linear_Model_B.B_g[0];
  tmp_3 = Linear_Model_B.B_g[1];
  tmp_1 = Linear_Model_B.B_g[2];
  tmp_4 = Linear_Model_B.B_g[3];
  tmp_0 = Linear_Model_B.TmpSignalConversionAtProductInp[0];
  tmp_2 = Linear_Model_B.TmpSignalConversionAtProductInp[1];
  tmp *= tmp_0;
  tmp += tmp_1 * tmp_2;

  /* Product: '<S5>/Product' */
  Linear_Model_B.Bu[0] = tmp;

  /* Product: '<S5>/Product' */
  tmp_3 *= tmp_0;
  tmp_3 += tmp_4 * tmp_2;

  /* Product: '<S5>/Product' */
  Linear_Model_B.Bu[1] = tmp_3;

  /* Product: '<S5>/Product1' */
  tmp = Linear_Model_B.A_p[0];
  tmp_3 = Linear_Model_B.A_p[1];
  tmp_1 = Linear_Model_B.A_p[2];
  tmp_4 = Linear_Model_B.A_p[3];
  tmp_0 = Linear_Model_B.x[0];
  tmp_2 = Linear_Model_B.x[1];
  tmp *= tmp_0;
  tmp += tmp_1 * tmp_2;

  /* Product: '<S5>/Product1' */
  Linear_Model_B.Ax[0] = tmp;

  /* Sum: '<S5>/Sum' */
  Linear_Model_B.dx[0] = Linear_Model_B.Bu[0] + Linear_Model_B.Ax[0];

  /* Product: '<S5>/Product1' */
  tmp_3 *= tmp_0;
  tmp_3 += tmp_4 * tmp_2;

  /* Product: '<S5>/Product1' */
  Linear_Model_B.Ax[1] = tmp_3;

  /* Sum: '<S5>/Sum' */
  Linear_Model_B.dx[1] = Linear_Model_B.Bu[1] + Linear_Model_B.Ax[1];
  if (rtmIsMajorTimeStep(Linear_Model_M)) {
    /* Matfile logging */
    rt_UpdateTXYLogVars(Linear_Model_M->rtwLogInfo, (Linear_Model_M->Timing.t));
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

  /* Derivatives for Integrator: '<S5>/Integrator' */
  _rtXdot->VyYaw_rate[0] = Linear_Model_B.dx[0];
  _rtXdot->VyYaw_rate[1] = Linear_Model_B.dx[1];

  /* Derivatives for Integrator: '<S32>/Filter' */
  _rtXdot->Filter_CSTATE = Linear_Model_B.NProdOut;

  /* Derivatives for Integrator: '<S37>/Integrator' */
  _rtXdot->Integrator_CSTATE = Linear_Model_B.IProdOut;
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
  rtsiSetIsMinorTimeStepWithModeChange(&Linear_Model_M->solverInfo, false);
  rtsiSetSolverName(&Linear_Model_M->solverInfo,"ode3");
  rtmSetTPtr(Linear_Model_M, &Linear_Model_M->Timing.tArray[0]);
  rtmSetTFinal(Linear_Model_M, 10.0);
  Linear_Model_M->Timing.stepSize0 = 0.2;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    Linear_Model_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(Linear_Model_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(Linear_Model_M->rtwLogInfo, (NULL));
    rtliSetLogT(Linear_Model_M->rtwLogInfo, "tout");
    rtliSetLogX(Linear_Model_M->rtwLogInfo, "");
    rtliSetLogXFinal(Linear_Model_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(Linear_Model_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(Linear_Model_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(Linear_Model_M->rtwLogInfo, 0);
    rtliSetLogDecimation(Linear_Model_M->rtwLogInfo, 1);
    rtliSetLogY(Linear_Model_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(Linear_Model_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(Linear_Model_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &Linear_Model_B), 0,
                sizeof(B_Linear_Model_T));

  /* states (continuous) */
  {
    (void) memset((void *)&Linear_Model_X, 0,
                  sizeof(X_Linear_Model_T));
  }

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(Linear_Model_M->rtwLogInfo, 0.0, rtmGetTFinal
    (Linear_Model_M), Linear_Model_M->Timing.stepSize0, (&rtmGetErrorStatus
    (Linear_Model_M)));

  /* InitializeConditions for Integrator: '<S5>/Integrator' */
  Linear_Model_X.VyYaw_rate[0] = Linear_Model_P.Integrator_IC;
  Linear_Model_X.VyYaw_rate[1] = Linear_Model_P.Integrator_IC;

  /* InitializeConditions for Integrator: '<S32>/Filter' */
  Linear_Model_X.Filter_CSTATE = Linear_Model_P.PIDController_InitialConditionF;

  /* InitializeConditions for Integrator: '<S37>/Integrator' */
  Linear_Model_X.Integrator_CSTATE =
    Linear_Model_P.PIDController_InitialConditio_e;
}

/* Model terminate function */
void Linear_Model_terminate(void)
{
  /* (no terminate code required) */
}
