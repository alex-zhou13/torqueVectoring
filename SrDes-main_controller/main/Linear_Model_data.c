/*
 * Linear_Model_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Linear_Model".
 *
 * Model version              : 1.11
 * Simulink Coder version : 9.7 (R2022a) 13-Nov-2021
 * C source code generated on : Thu Mar 23 16:30:31 2023
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
   * Referenced by: '<S32>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditio_e
   * Referenced by: '<S37>/Integrator'
   */
  0.0,

  /* Expression: x0
   * Referenced by: '<S5>/Integrator'
   */
  0.0,

  /* Expression: [250, 300, 275, 294, 300, 275, 350]
   * Referenced by: '<Root>/1-D Lookup Table'
   */
  { 250.0, 300.0, 275.0, 294.0, 300.0, 275.0, 350.0 },

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

  /* Expression: [12000, 11000, 12500, 20000, 10000, 20000, 14000]
   * Referenced by: '<Root>/1-D Lookup Table1'
   */
  { 12000.0, 11000.0, 12500.0, 20000.0, 10000.0, 20000.0, 14000.0 },

  /* Expression: [6, 9, 12, 15, 18, 21, 24]
   * Referenced by: '<Root>/1-D Lookup Table1'
   */
  { 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0 }
};
