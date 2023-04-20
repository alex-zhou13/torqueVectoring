/*
 * Linear_Model.h
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

#ifndef RTW_HEADER_Linear_Model_h_
#define RTW_HEADER_Linear_Model_h_
#ifndef Linear_Model_COMMON_INCLUDES_
#define Linear_Model_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* Linear_Model_COMMON_INCLUDES_ */

#include "Linear_Model_types.h"
#include <float.h>
#include <string.h>
#include <stddef.h>
#include "rtGetInf.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T DiscreteTransferFcn;          /* '<Root>/Discrete Transfer Fcn' */
  real_T Sum;                          /* '<Root>/Sum' */
  real_T uDLookupTable;                /* '<Root>/1-D Lookup Table' */
  real_T PProdOut;                     /* '<S40>/PProd Out' */
  real_T Integrator;                   /* '<S35>/Integrator' */
  real_T DProdOut;                     /* '<S29>/DProd Out' */
  real_T Filter;                       /* '<S30>/Filter' */
  real_T SumD;                         /* '<S30>/SumD' */
  real_T NProdOut;                     /* '<S38>/NProd Out' */
  real_T Sum_h;                        /* '<S44>/Sum' */
  real_T uDLookupTable1;               /* '<Root>/1-D Lookup Table1' */
  real_T IProdOut;                     /* '<S32>/IProd Out' */
  real_T right_Motor_Control_out;      /* '<Root>/MATLAB Function' */
  real_T left_Motor_Control_out;       /* '<Root>/MATLAB Function' */
  real_T des_yawRate;                  /* '<Root>/Desired_yawRate' */
} B_Linear_Model_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTransferFcn_states[2];/* '<Root>/Discrete Transfer Fcn' */
} DW_Linear_Model_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S35>/Integrator' */
  real_T Filter_CSTATE;                /* '<S30>/Filter' */
} X_Linear_Model_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S35>/Integrator' */
  real_T Filter_CSTATE;                /* '<S30>/Filter' */
} XDot_Linear_Model_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE;         /* '<S35>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S30>/Filter' */
} XDis_Linear_Model_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T SteeringAngleEncoder;         /* '<Root>/Steering Angle Encoder' */
  real_T V_CG;                         /* '<Root>/V_CG' */
  real_T throttle_Input;               /* '<Root>/throttle_Input' */
} ExtU_Linear_Model_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T right_Motor_Control_out;      /* '<Root>/right_Motor_Control_out' */
  real_T left_Motor_Control_out;       /* '<Root>/left_Motor_Control_out' */
} ExtY_Linear_Model_T;

/* Parameters (default storage) */
struct P_Linear_Model_T_ {
  real_T PIDController_InitialConditionF;
                              /* Mask Parameter: PIDController_InitialConditionF
                               * Referenced by: '<S30>/Filter'
                               */
  real_T PIDController_InitialConditio_e;
                              /* Mask Parameter: PIDController_InitialConditio_e
                               * Referenced by: '<S35>/Integrator'
                               */
  real_T DiscreteTransferFcn_NumCoef[3];
                    /* Expression: [0, 0.0974090329888150, 0.000170465807730426]
                     * Referenced by: '<Root>/Discrete Transfer Fcn'
                     */
  real_T DiscreteTransferFcn_DenCoef[3];
                   /* Expression: [1, 0.00477228554123995, 9.18837401797686e-07]
                    * Referenced by: '<Root>/Discrete Transfer Fcn'
                    */
  real_T DiscreteTransferFcn_InitialStat;/* Expression: 0
                                          * Referenced by: '<Root>/Discrete Transfer Fcn'
                                          */
  real_T uDLookupTable_tableData[7];
                           /* Expression: [305, 395, 450, 500, 510, 475, 450]/19
                            * Referenced by: '<Root>/1-D Lookup Table'
                            */
  real_T uDLookupTable_bp01Data[7];    /* Expression: [6, 9, 12, 15, 18, 21, 24]
                                        * Referenced by: '<Root>/1-D Lookup Table'
                                        */
  real_T Constant3_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant3'
                                        */
  real_T MaxToqueofMotorsNm_Value;     /* Expression: 75
                                        * Referenced by: '<Root>/Max Toque of Motors (N*m)'
                                        */
  real_T MaxVoltage_Value;             /* Expression: 5
                                        * Referenced by: '<Root>/Max Voltage'
                                        */
  real_T uDLookupTable1_tableData[7];
                              /* Expression: [1.2, 1, 0.8, 0.6, 0.4, 0.2, 1]*0.7
                               * Referenced by: '<Root>/1-D Lookup Table1'
                               */
  real_T uDLookupTable1_bp01Data[7];   /* Expression: [6, 9, 12, 15, 18, 21, 24]
                                        * Referenced by: '<Root>/1-D Lookup Table1'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_Linear_Model_T {
  const char_T *errorStatus;
  //RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;
  X_Linear_Model_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[2];
  real_T odeF[3][2];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
extern P_Linear_Model_T Linear_Model_P;

/* Block signals (default storage) */
extern B_Linear_Model_T Linear_Model_B;

/* Continuous states (default storage) */
extern X_Linear_Model_T Linear_Model_X;

/* Block states (default storage) */
extern DW_Linear_Model_T Linear_Model_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Linear_Model_T Linear_Model_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Linear_Model_T Linear_Model_Y;

/* Model entry point functions */
extern void Linear_Model_initialize(void);
extern void Linear_Model_step(void);
extern void Linear_Model_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Linear_Model_T *const Linear_Model_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<Root>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<Root>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<Root>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<Root>/Cast To Double4' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Linear_Model'
 * '<S1>'   : 'Linear_Model/Desired_yawRate'
 * '<S2>'   : 'Linear_Model/MATLAB Function'
 * '<S3>'   : 'Linear_Model/PID Controller'
 * '<S4>'   : 'Linear_Model/PID Controller/Anti-windup'
 * '<S5>'   : 'Linear_Model/PID Controller/D Gain'
 * '<S6>'   : 'Linear_Model/PID Controller/Filter'
 * '<S7>'   : 'Linear_Model/PID Controller/Filter ICs'
 * '<S8>'   : 'Linear_Model/PID Controller/I Gain'
 * '<S9>'   : 'Linear_Model/PID Controller/Ideal P Gain'
 * '<S10>'  : 'Linear_Model/PID Controller/Ideal P Gain Fdbk'
 * '<S11>'  : 'Linear_Model/PID Controller/Integrator'
 * '<S12>'  : 'Linear_Model/PID Controller/Integrator ICs'
 * '<S13>'  : 'Linear_Model/PID Controller/N Copy'
 * '<S14>'  : 'Linear_Model/PID Controller/N Gain'
 * '<S15>'  : 'Linear_Model/PID Controller/P Copy'
 * '<S16>'  : 'Linear_Model/PID Controller/Parallel P Gain'
 * '<S17>'  : 'Linear_Model/PID Controller/Reset Signal'
 * '<S18>'  : 'Linear_Model/PID Controller/Saturation'
 * '<S19>'  : 'Linear_Model/PID Controller/Saturation Fdbk'
 * '<S20>'  : 'Linear_Model/PID Controller/Sum'
 * '<S21>'  : 'Linear_Model/PID Controller/Sum Fdbk'
 * '<S22>'  : 'Linear_Model/PID Controller/Tracking Mode'
 * '<S23>'  : 'Linear_Model/PID Controller/Tracking Mode Sum'
 * '<S24>'  : 'Linear_Model/PID Controller/Tsamp - Integral'
 * '<S25>'  : 'Linear_Model/PID Controller/Tsamp - Ngain'
 * '<S26>'  : 'Linear_Model/PID Controller/postSat Signal'
 * '<S27>'  : 'Linear_Model/PID Controller/preSat Signal'
 * '<S28>'  : 'Linear_Model/PID Controller/Anti-windup/Passthrough'
 * '<S29>'  : 'Linear_Model/PID Controller/D Gain/External Parameters'
 * '<S30>'  : 'Linear_Model/PID Controller/Filter/Cont. Filter'
 * '<S31>'  : 'Linear_Model/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S32>'  : 'Linear_Model/PID Controller/I Gain/External Parameters'
 * '<S33>'  : 'Linear_Model/PID Controller/Ideal P Gain/Passthrough'
 * '<S34>'  : 'Linear_Model/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S35>'  : 'Linear_Model/PID Controller/Integrator/Continuous'
 * '<S36>'  : 'Linear_Model/PID Controller/Integrator ICs/Internal IC'
 * '<S37>'  : 'Linear_Model/PID Controller/N Copy/Disabled'
 * '<S38>'  : 'Linear_Model/PID Controller/N Gain/External Parameters'
 * '<S39>'  : 'Linear_Model/PID Controller/P Copy/Disabled'
 * '<S40>'  : 'Linear_Model/PID Controller/Parallel P Gain/External Parameters'
 * '<S41>'  : 'Linear_Model/PID Controller/Reset Signal/Disabled'
 * '<S42>'  : 'Linear_Model/PID Controller/Saturation/Passthrough'
 * '<S43>'  : 'Linear_Model/PID Controller/Saturation Fdbk/Disabled'
 * '<S44>'  : 'Linear_Model/PID Controller/Sum/Sum_PID'
 * '<S45>'  : 'Linear_Model/PID Controller/Sum Fdbk/Disabled'
 * '<S46>'  : 'Linear_Model/PID Controller/Tracking Mode/Disabled'
 * '<S47>'  : 'Linear_Model/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S48>'  : 'Linear_Model/PID Controller/Tsamp - Integral/Passthrough'
 * '<S49>'  : 'Linear_Model/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S50>'  : 'Linear_Model/PID Controller/postSat Signal/Forward_Path'
 * '<S51>'  : 'Linear_Model/PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_Linear_Model_h_ */
