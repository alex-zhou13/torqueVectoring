/*
 * Linear_Model_data.c
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

/* Block parameters (default storage) */
P_Linear_Model_T Linear_Model_P = {
  /* Mask Parameter: PIDController_InitialConditionF
   * Referenced by: '<S30>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditio_e
   * Referenced by: '<S35>/Integrator'
   */
  0.0,

  /* Expression: [0, 0.0974090329888150, 0.000170465807730426]
   * Referenced by: '<Root>/Discrete Transfer Fcn'
   */
  { 0.0, 0.097409032988815, 0.000170465807730426 },

  /* Expression: [1, 0.00477228554123995, 9.18837401797686e-07]
   * Referenced by: '<Root>/Discrete Transfer Fcn'
   */
  { 1.0, 0.00477228554123995, 9.18837401797686E-7 },

  /* Expression: 0
   * Referenced by: '<Root>/Discrete Transfer Fcn'
   */
  0.0,

  /* Expression: [305, 395, 450, 500, 510, 475, 450]/19
   * Referenced by: '<Root>/1-D Lookup Table'
   */
  { 16.05263157894737, 20.789473684210527, 23.684210526315791,
    26.315789473684209, 26.842105263157894, 25.0, 23.684210526315791 },

  /* Expression: [6, 9, 12, 15, 18, 21, 24]
   * Referenced by: '<Root>/1-D Lookup Table'
   */
  { 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0 },

  /* Expression: 0
   * Referenced by: '<Root>/Constant3'
   */
  0.0,

  /* Expression: 75
   * Referenced by: '<Root>/Max Toque of Motors (N*m)'
   */
  75.0,

  /* Expression: 5
   * Referenced by: '<Root>/Max Voltage'
   */
  5.0,

  /* Expression: [1.2, 1, 0.8, 0.6, 0.4, 0.2, 1]*0.7
   * Referenced by: '<Root>/1-D Lookup Table1'
   */
  { 0.84, 0.7, 0.55999999999999994, 0.42, 0.27999999999999997,
    0.13999999999999999, 0.7 },

  /* Expression: [6, 9, 12, 15, 18, 21, 24]
   * Referenced by: '<Root>/1-D Lookup Table1'
   */
  { 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0 }
};
