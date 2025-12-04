//  Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
//  This file is subject to the terms and conditions defined in the file
//  'LICENSE' , which is part of this package

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME get_duration_period

// Need to include simstruc.h for the definition of the SimStruct and
// its associated macro definitions.
#include "simstruc.h"

#include "local_utils.h"

// Abstract:
//    The sizes information is used by Simulink to determine the S-function
//    block's characteristics (number of inputs, outputs, states, etc.).
static void mdlInitializeSizes(SimStruct* S) {
  /* Number of continuous and discrete states */
  ssSetNumContStates(S, 0);
  ssSetNumDiscStates(S, 0);

  ssSetNumSFcnParams(S, 1);
  
  ssSetSFcnParamTunable(S, 0, SS_PRM_NOT_TUNABLE);

  int numInputs = 0;

  /* Direct input signal access and direct input feedthrough */
  for (int ii = 0; ii < numInputs; ii++) {
    ssSetInputPortRequiredContiguous(S, ii, true); /*direct input signal access*/
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     */
    ssSetInputPortDirectFeedThrough(S, ii, 1);
  }

  /* Output ports, their sizes and datatypes */
  if (!ssSetNumOutputPorts(S, 1))
    return;
  for (int i = 0; i < 1; i++) {
    ssSetOutputPortWidth(S, i, 1);
    ssSetOutputPortDataType(S, i, SS_DOUBLE);
  }

  /* Data-related */
  ssSetNumRWork(S, 0); /* number of real work vector elements   */
  ssSetNumIWork(S, 0); /* number of integer work vector elements*/
  ssSetNumPWork(S, 0); /* number of pointer work vector elements*/
  ssSetNumDWork(S, 1);
  ssSetDWorkWidth(S, 0, 1);
  ssSetDWorkDataType(S, 0, SS_DOUBLE);
  
  /* Misc. */
  ssSetNumSampleTimes(S, 1);   /* number of sample times */
  ssSetNumModes(S, 0);         /* size of mode vector */
  ssSetNumNonsampledZCs(S, 0); /* number of states for which a block detects zero crossings */
  ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE); /* sim state compliance */
  ssSetOptions(S, 0);                                /* s-function options */
}

// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes(SimStruct* S) {
  ssSetSampleTime(S, 0, 0.001);
  ssSetOffsetTime(S, 0, 0.0);
  ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

// Function: mdlStart =======================================================
// Abstract:
//   This function is called once at start of model execution. If you
//   have states that should be initialized once, this is the place
//   to do it.
#define MDL_START
static void mdlStart(SimStruct* S) {}

// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block.
static void mdlOutputs(SimStruct* S, int_T tid) {}

#define MDL_UPDATE /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
/* Function: mdlUpdate ======================================================
 * Abstract:
 *    This function is called once for every major integration time step.
 *    Discrete states are typically updated here, but this function is useful
 *    for performing any tasks that should only take place once per
 *    integration step.
 */
static void mdlUpdate(SimStruct* S, int_T tid) {}
#endif /* MDL_UPDATE */

#define MDL_DERIVATIVES /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
/* Function: mdlDerivatives =================================================
 * Abstract:
 *    In this function, you compute the S-function block's derivatives.
 *    The derivatives are placed in the derivative vector, ssGetdX(S).
 */
static void mdlDerivatives(SimStruct* S) {}
#endif /* MDL_DERIVATIVES */

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct* S) {}

#if defined(MATLAB_MEX_FILE)
#define MDL_RTW
/* Function: mdlRTW ===========================================================
 * Abstract:
 */
static void mdlRTW(SimStruct *S)
{   
    size_t robot_ip_length = mxGetNumberOfElements(ssGetSFcnParam(S, 0)) + 1;
    static char robot_ip_[20];
    mxGetString(ssGetSFcnParam(S, 0), robot_ip_, robot_ip_length);
    
    // define robot's unique id
    std::string robot_id_string(robot_ip_);
    replaceStringInPlace(robot_id_string,".","");
    
    static char robot_id_cstr[20];
    robot_id_string.copy(robot_id_cstr, robot_id_string.size() + 1);
    robot_id_cstr[robot_id_string.size()] = '\0';
    
    if ( !ssWriteRTWParamSettings(S, 1,
            SSWRITE_VALUE_QSTR, "robot_id", &robot_id_cstr)) {
        ssSetErrorStatus(S,"Error writing parameter settings data to .rtw file");           
        return; /* An error occured, which will be reported by Simulink. */
    }
}
#endif /* MDL_RTW */

// Required S-function trailer
#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c"  /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif
