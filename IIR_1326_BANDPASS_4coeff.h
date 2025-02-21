/*
 * Filter Coefficients (C Source) generated by the Filter Design and Analysis Tool
 * Generated by MATLAB(R) 24.2 and Signal Processing Toolbox 24.2.
 * Generated on: 30-Jan-2025 02:40:57
 */

/*
 * Discrete-Time IIR Filter (real)
 * -------------------------------
 * Filter Structure    : Direct-Form II, Second-Order Sections
 * Number of Sections  : 2
 * Stable              : Yes
 * Linear Phase        : No
 */

/* General type conversion for MATLAB generated C-code  */
//#include "tmwtypes.h"
/* 
 * Expected path to tmwtypes.h 
 * C:\Program Files\MATLAB\R2024b\extern\include\tmwtypes.h 
 */
/*
 * Warning - Filter coefficients were truncated to fit specified data type.  
 *   The resulting response may not match generated theoretical response.
 *   Use the Filter Design & Analysis Tool to design accurate
 *   single-precision filter coefficients.
 */
#define MWSPT_NSEC 5
const int NL_1326[MWSPT_NSEC] = { 1,3,1,3,1 };
const float NUM_1326[MWSPT_NSEC][3] = {
  {
     0.0117404582,              0,              0 
  },
  {
                1, -0.09138354659,              1 
  },
  {
     0.0117404582,              0,              0 
  },
  {
                1,   -1.833462834,              1 
  },
  {
                1,              0,              0 
  }
};
const int DL_1326[MWSPT_NSEC] = { 1,3,1,3,1 };
const float DEN_1326[MWSPT_NSEC][3] = {
  {
                1,              0,              0 
  },
  {
                1,   -1.322796226,   0.9930923581 
  },
  {
                1,              0,              0 
  },
  {
                1,   -1.339529634,   0.9931614995 
  },
  {
                1,              0,              0 
  }
};
