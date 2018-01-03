/*******************************************************************************
* JM_balance_config
*
* Contains the settings for configuration of mip_balance.c
*******************************************************************************/

#ifndef JM_balance_config
#define JM_balance_config

// Inner Loop controller for MIP angle theta
#define INNER_LOOP_HZ    100.0              // Frequency of inner loop set by the DMP
#define D1_ORDER         2                  // degree of denominator, n
#define D1_REL_DEGREE    0                  // n-m=r
#define DT_1 					   0.01               // Inner loop sample time
#define D1_K            -10
#define D1_NUM          {1, -1.857, 0.8641}
#define D1_DEN          {1, -1.368, 0.3679}
#define D1_SAT           1                  // maximum duty cycle
#define PREFACTOR        1

// Outer Loop controller for MIP wheel position (relative to body) phi
#define OUTER_LOOP_HZ    20.0               // Frequency of the outer loop
#define D2_ORDER         2                  // degree of denominator, n
#define D2_REL_DEGREE    0                  // n-m=r
#define DT_2             0.05               // outer loop sample time
#define D2_K             2
#define D2_NUM          {0.18856,  -0.37209,  0.18354}
#define D2_DEN          {1.00000,  -1.86046,   0.86046}
#define D2_SAT           0.35               // theta_ref_max
#define PHI_REF          0

// Print thread
#define PRINT_THREAD_HZ  10.0          // Frequency of print_theta function

// IMU
#define ACCEL_RAW_TO_MS2 0.00119750977 // (4*9.81)/(2^15) at 4G FSR, 16 bit ADC
#define GYRO_RAW_TO_RADS 0.00053263222 // (1000)*(2*PI/360)/(2^15) at 1000 DPS FSR, 16 bit ADC

// Complimentary filter (theta estimate)
#define TIME_CONSTANT    0.5		       // time constant of Complimentary filter
#define STEP_SIZE	       0.01		       // Operating at 100 Hz in DMP
#define OMEGA_C          2             // 1/TIME_CONSTANT

// Misc. constant
#define TWOPI            2*3.14

// Encoders
#define ENC_STEPS        60 // number of steps encoder reads
#define ENC_CH_R         2  // right encoder channel
#define ENC_CH_L         3  // left encoder channel
#define ENC_POLARITY_R  -1
#define ENC_POLARITY_L   1

// Motors
#define MOTOR_CH_R         2  // right motor channel (from rear of cape)
#define MOTOR_CH_L         3  // left motor channel (from rear of cape)
#define MOTOR_POLARITY_R  -1  // (default: -1)
#define MOTOR_POLARITY_L   1  // (default: 1)
#define GEAR_RATIO         35.555

// MIP physical constants
#define WHEEL_RADIUS       0.034 // wheel radius in m
#define CAPE_OFFSET_ANGLE  0.25  // offset angle in radians
#define THETA_REF_MAX      0.35  // radians (original 0.35
#define TIP_ANGLE          0.9   // radian

// JM_filter_t
// generic filter up to order 4; may change to order "order"
typedef struct JM_filter_t {
  int initialization; // 1=intitialized, -1=not intitialized
  int order;        // order of denom
  int rel_deg;      // relative degree of num & den (assumed causal)
  float sat;        // max/min value of output, set to 0 if do not want saturation
  int step;

  float gain;
  float num[3];     // k=0;k-1=1;...k-n=n
  float den[3];     // k=0;k-1=1;...k-n=n
  float inputs[3];  // k=0;k-1=1;...k-n=n
  float outputs[3]; // k=0;k-1=1;...k-n=n
} JM_filter_t;

#endif	//JM_balance_config
