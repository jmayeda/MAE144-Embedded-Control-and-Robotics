/*******************************************************************************
* JM_balance_config
*
* Contains the settings for configuration of mip_balance.c
*******************************************************************************/

#ifndef JM_balance_config
#define JM_balance_config

// Inner Loop controller for MIP angle theta
#define INNER_LOOP_HZ    100.0         // Frequency of inner loop set by the DMP
#define D1_ORDER         2
#define D1_REL_DEGREE    0
#define DT_1 					   0.01
#define D1_K            -26//1.05//30
#define D1_NUM          {1, -1.923, 0.9247}//{-4.945, 8.862, -3.967} //{1, -1.895, 0.8985}
#define D1_DEN          {1, -1.368, 0.3679} //{ 1.000, -1.481, 0.4812}//{1, -1.333, 0.3329}

// Outer Loop controller for MIP wheel position (relative to body) phi
#define OUTER_LOOP_HZ    20.0          // Frequency of the outer loop
#define D2_ORDER         2
#define D2_REL_DEGREE    0
#define DT_2             0.05
#define D2_K             1
#define D2_NUM          {1, -1.917, 0.9177}
#define D2_DEN          {1,  1.375, 0.4724}
#define D2_SAT           0.25  // theta_ref_max
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
#define ENC_POLARITY_R   1  // clockwise is negative (default: -1)
#define ENC_POLARITY_L  -1  // clockwise is positive (default: 1)

// Motors
#define MOTOR_CH_R         2  // right motor channel (from rear of cape)
#define MOTOR_CH_L         3  // left motor channel (from rear of cape)
#define MOTOR_POLARITY_R  -1  // (default: -1)
#define MOTOR_POLARITY_L   1  // (default: 1)
#define MAX_DUTY_CYCLE     1  // maximum setting for duty cycle
#define MIN_DUTY_CYCLE    -1  // minimum setting for duty cycle
#define GEAR_RATIO         35.555

// MIP physical constants
#define WHEEL_RADIUS       0.034 // wheel radius in m
#define CAPE_OFFSET_ANGLE  0.3  // offset angle in radians
#define THETA_REF_MAX      0.25 // radians (original 0.35
#define TIP_ANGLE          0.9 // radian

// JM_filter_t
// generic filter up to order 4; may change to order "order"
typedef struct JM_filter_t {
  int initialization; // 1=intitialized, -1=not intitialized
  int order;        // order of denom
  int rel_deg;      // relative degree of num & den (assumed causal)
  float sat;        // max/min value of output, set to 0 if do not want saturation

  float gain;
  float num[4];     // k=0;k-1=1;...k-n=n
  float den[4];     // k=0;k-1=1;...k-n=n
  float inputs[4];  // k=0;k-1=1;...k-n=n
  float outputs[4]; // k=0;k-1=1;...k-n=n
} JM_filter_t;

#endif	//JM_balance_config
