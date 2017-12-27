/*******************************************************************************
* JM_balance.c
*
* Jason Mayeda, Fall 2017
* This program is meant to balance the MIP for the MAE 144 final project.
*******************************************************************************/

#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "JM_balance_config.h"

// Decide when to "turn on" the controller
typedef enum con_state_t{
	ARMED,
	DISARMED
}con_state_t;

// Keep track of important system state variables
typedef struct sys_state_t{
  float theta;             // MIP body angle
  float phi;   		   // wheel turn angle
  float u1;      	   // motor duty cycle; control output (inner loop)
  float u2;                // theta_ref; control output (outer loop)
  float wheel_L_RAD;       // Left wheel position in radians
  float wheel_R_RAD;       // right wheel position in radians
  float time_count;
} sys_state_t;

// Struct to hold important setpoint parameters for outer and inner loop
typedef struct setpoint_t{
	float theta_ref;  	 // MIP body angle
	float theta_err;
	float phi_ref;           // wheel turn angle
	float phi_err;
	con_state_t con_state;
} setpoint_t;

// Global variables
sys_state_t state;
setpoint_t setpoint;
rc_imu_data_t imu_data;
// filters
JM_filter_t D1; // inner loop controller
JM_filter_t D2; // outer loop controller
//JM_filter_t HP; // high pass filter for gyro
//JM_filter_t LP; // low pass filter for accel

// function declarations
void printf_header();
void arm_controller();
void disarm_controller();

void inner_loop();    // IMU interrupt function @ 100Hz
void* printf_data();  // p_thread @ 10Hz
void* outer_loop();   // p_thread @ 20Hz

float advance_JM_filter(JM_filter_t* filter, float new_input);
int zero_JM_filter(JM_filter_t* filter);
JM_filter_t create_JM_filter(float gain, int n, int r, \
  float* D_den, float* D_num, float sat);

/*******************************************************************************
* int main()
*
*******************************************************************************/
int main(){
	// initialize the robotics cape library
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}
  printf("\n");
  printf("*****************************\n");
	printf("    MAE 144 Final Project    \n");
  printf("******** Jason Mayeda *******\n");

	// We are not ready yet..
	rc_set_led(RED,1);
	rc_set_led(GREEN,0);
	rc_set_state(UNINITIALIZED);
	setpoint.con_state = DISARMED;

  // inner loop controller D1
  float D1_num[] = D1_NUM;
  float D1_den[] = D1_DEN;
  D1 = create_JM_filter(D1_K, D1_ORDER, D1_REL_DEGREE, D1_den, D1_num, D1_SAT);

  // outer loop controller D2
  float D2_num[] = D2_NUM;
  float D2_den[] = D2_DEN;
  D2 = create_JM_filter(D2_K, D2_ORDER, D2_REL_DEGREE, D2_den, D2_num, D2_SAT);

	// print_theta thread, run @ 10Hz and low Priority
	pthread_t printf_thread;
	pthread_create(&printf_thread, NULL, printf_data, (void*) NULL);

	// outer loop controller thread
	pthread_t outer_loop_thread;
	pthread_create(&outer_loop_thread, NULL, outer_loop, (void*) NULL);

	// Set default configuration for IMU (4G and 1000DPS)
	rc_imu_config_t conf = rc_default_imu_config();
	conf.dmp_sample_rate = INNER_LOOP_HZ;

	//set up the imu for dmp interrupt operation
	if(rc_initialize_imu_dmp(&imu_data, conf)){
		printf("rc_initialize_imu_failed\n");
		return -1;
	}

	// Run inner loop as DMP interrupt function
	rc_set_imu_interrupt_func(&inner_loop);

  // Pause to let some things finish
  rc_usleep(100000);

  // done initializing so set state to RUNNING
  rc_set_state(RUNNING);

	printf_header();

	// Important functions handled in separate threads and DMP interrupts
	while(rc_get_state()!=EXITING){
		rc_usleep(10000);

    // Check for exiting and recovery conditions
    if(setpoint.con_state == ARMED && fabs(state.theta) > TIP_ANGLE){
      disarm_controller();
      setpoint.con_state = DISARMED; // MIP has tipped over
      rc_set_led(RED, 1);
      rc_set_led(GREEN, 0);
    }
    if(setpoint.con_state == DISARMED && fabs(state.theta) < TIP_ANGLE){
      arm_controller();
      setpoint.con_state = ARMED; // recovered from tip
      rc_set_led(RED, 0);
      rc_set_led(GREEN, 1);
    }
	}

	// join threads
	pthread_join(printf_thread, NULL);
	pthread_join(outer_loop_thread, NULL);

	// exit cleanly
	rc_power_off_imu();
	rc_cleanup();
	return 0;
}

/*******************************************************************************
* void outer_loop()
*
* Inner loop is a pthread operatins at 20 Hz.
* 1. Estimates wheel angular position phi by averaging encoder values of wheels.
* 2. Implements D2 difference equation.
*******************************************************************************/
void* outer_loop(){
	while(rc_get_state()!=EXITING){
		if(rc_get_state() == RUNNING){

			// Outer loop setpoint
			setpoint.phi_ref = PHI_REF; // 0 radians
			// Sample the encoders to get an estimate for the wheel position phi
			state.wheel_R_RAD = (rc_get_encoder_pos(ENC_CH_R) * TWOPI) \
				/ (ENC_POLARITY_R * ENC_STEPS * GEAR_RATIO);
			state.wheel_L_RAD = (rc_get_encoder_pos(ENC_CH_L) * TWOPI) \
				/ (ENC_POLARITY_L * ENC_STEPS * GEAR_RATIO);

      // wheel angular position phi is measured relative to the body, so
      // subtract theta from measurement (if phi has same angle sign convention)
			state.phi = 0.5 * (state.wheel_L_RAD + state.wheel_R_RAD) - state.theta;
			setpoint.phi_err = setpoint.phi_ref - state.phi;

      // Advance D2 difference equation
      setpoint.theta_ref = advance_JM_filter(&D2, setpoint.phi_err);
		}
		// run at OUTER_LOOP_HZ (10Hz)
		rc_usleep(1e6/OUTER_LOOP_HZ);
	}
	return NULL;
} // void outer_loop()

/*******************************************************************************
* void inner_loop()
*
* Inner loop controller operating at 100 Hz off of DMP interrupt.
* 1. Estimates the body angle theta using IMU + complimentary filter.
* 2. Checks for exiting conditions.
* 3. Implement D1 difference equation.
* 4. Drive motors using duty cycle output.
*******************************************************************************/
void inner_loop(){
	// =========================  ESTIMATE THETA =============================== //
	// Declare local variables
	float dtheta;                      // gyro readings
	float y_accel, z_accel;            // accel readings
	float theta_g_raw[2], theta_g[2];  // theta_g estimates
	float theta_a_raw[2], theta_a[2];  // theta_a estimates

	// Read gyro data and convert raw ADC values to rad/s
	dtheta = imu_data.raw_gyro[0] * GYRO_RAW_TO_RADS;

	// Integrate dtheta using Euler's integration
	theta_g_raw[0] = theta_g_raw[1] + dtheta * DT_1;

	// Read raw values from IMU and convert to m/s^2
	y_accel = imu_data.raw_accel[1] * ACCEL_RAW_TO_MS2;
	z_accel = imu_data.raw_accel[2] * ACCEL_RAW_TO_MS2;

	// Calculate theta_a_raw from acceleration projections
	// -1 added to get the correct sign convention
	theta_a_raw[0] = atan2((-1.0) * z_accel,y_accel);

	// Apply a HP filter on theta_g_raw to get theta_g
	theta_g[0] = theta_g[1] * (1.0 - OMEGA_C * STEP_SIZE) \
		+ theta_g_raw[0] - theta_g_raw[1];

	// Apply a LP filter on theta_a_raw to get theta_a
	theta_a[0] = (1.0 - OMEGA_C * STEP_SIZE) * theta_a[1] \
		+ (OMEGA_C * STEP_SIZE) * theta_a_raw[1];

	// Apply complimentary filter and account for offset angle
	state.theta = theta_a[0] + theta_g[0] + CAPE_OFFSET_ANGLE;

	// update old values
	theta_a_raw[1] = theta_a_raw[0];
	theta_a[1] = theta_a[0];
	theta_g_raw[1] = theta_g_raw[0];
	theta_g[1] = theta_g[0];
	// ========================================================================= //

	// Check for exiting conditions
	if(rc_get_state()==EXITING){
		rc_disable_motors();
		return;
	}
  // Implement controller D1
	setpoint.theta_err = setpoint.theta_ref - state.theta;
  state.u1 = advance_JM_filter(&D1, setpoint.theta_err);

	// Drive the motors at duty cycle given
	rc_set_motor(MOTOR_CH_R, MOTOR_POLARITY_R * state.u1);
	rc_set_motor(MOTOR_CH_L, MOTOR_POLARITY_L * state.u1);

  // Keep track of current time using time step 'dt'
  state.time_count += DT_1;

	return;
} // void inner_loop()

/*******************************************************************************
* void arm_controller()
*
* "Arm" controller if certain starting conditions are met.
*******************************************************************************/
void arm_controller(){
  setpoint.con_state = ARMED;
  state.theta = 0;
  state.phi = 0;
  state.wheel_L_RAD = 0;
  state.wheel_R_RAD = 0;

  zero_JM_filter(&D1);
  zero_JM_filter(&D2);
  rc_enable_motors();
  return;
}

/*******************************************************************************
* void disarm_controller()
*
* "Disarm" controller if certain conditions are met.
*******************************************************************************/
void disarm_controller(){
  setpoint.con_state = DISARMED;
  rc_disable_motors();
  return;
}

/*******************************************************************************
* void printf_header()
*
* Print header to the screen.
*******************************************************************************/
void printf_header(){
  printf(" time (s) |");
  printf("  theta   |");
  printf("theta_ref |");
  printf("   phi    |");
  printf(" phi_ref  |");
  printf("   u1     |");
  printf("   u2     |");
  printf("controller|");
  printf("\n");

  return;
} // void printf_header

/*******************************************************************************
* void printf_data()
*
* Print important data to the screen.
*******************************************************************************/
void* printf_data(){
	while(rc_get_state()!=EXITING){
		// Check if the controllers are armed
		if(rc_get_state() == RUNNING){

			printf("\r");
			printf(" %7.3f |", state.time_count);
			printf(" %7.3f |", state.theta);
			printf(" %7.3f |", setpoint.theta_ref);
			printf(" %7.3f |", state.phi);
			printf(" %7.3f |", setpoint.phi_ref);
			printf(" %7.3f |", state.u1);
			printf(" %7.3f |", state.u2);
      if(setpoint.con_state == ARMED) printf(" ARMED |");
      else printf(" DISARMED |");
			fflush(stdout);
		}
		// run at PRINT_THREAD_HZ (10Hz)
		rc_usleep(1e6/PRINT_THREAD_HZ);
	}
	return NULL;
} // void printf_data()

/*******************************************************************************
* float advance_JM_filter(JM_filter_t* filter, float new_input)
*
* Advance a generic filter of order n and relative degree n-m=r
* Inputs: pointer to a struct JM_filter_t that contains basic filter parameters
* JM_filter_t* filter: {gain, order, rel. degree, den, num, sat, dt}
* Outputs: latest value of output new_output;
*******************************************************************************/
float advance_JM_filter(JM_filter_t* filter, float new_input) {
  // Declare local variables
	int i;
  int n = filter->order;
	int r = filter->rel_deg;
	float Dnum[] = filter->num;
	float Dden[] = filter->den;
	float sat = filter->saturation;

  // advance filter outputs
  for(i=n; i>0; i--){
    filter->outputs[i] = filter->outputs[i-1];
  }

	// advance filter inputs
  filter->inputs[0] = new_input;
	for(i=n-r; i>0; i--){
		filter->inputs[i] = filter->inputs[i-1];
	}

  // calculate the difference equation
	for(i=n; i>0; i--){
		filter->outputs[0] -= Dden[i] * filter->outputs[i];
	}
	for(i=n-r; i>=0; i--){
		filter->outputs[0] += filter->gain * Dnum[i] * filter->inputs[i];
	}
  filter->outputs[0] /= Dden[0];

	// enable saturation
	if(sat != 0){
		if(filter->outputs[0] >= sat) filter->outputs[0] = sat;
		if(filter->outputs[0] <= -sat) filter->outputs[0] = -sat;
	}

	float new_output = filter->outputs[0];
	return new_output;
} // advance_filter()


/*******************************************************************************
* JM_filter_t create_JM_filter(float gain, int order, int rel_deg, float* D_den,
* float* D_num, float sat, float dt)
*
* Create a filter of type JM_filter_t. Supports generic filter of any order
* and relative degree as specified in the config file.
*******************************************************************************/
JM_filter_t create_JM_filter(float gain, int n, int r, float* D_den, \
	float* D_num, float sat) {

	// filter struct
	JM_filter_t filter;
	int i;

  // fill den & num of filter and zero outputs and inputs
	for(i=n, i>=0, i--){
		filter.den[i] = D_den[i];
		filter.outputs[i] = 0.0;
	}
	for(i=n-r, i>=0, i--){
		filter.num[i] = D_num[i];
		filter.inputs[i] = 0.0;
	}

  filter.order = n;
	filter.rel_deg = r;
	filter.gain = gain;
	filter.sat = sat;

	// We're all set
	filter.initialization = 1;
	return filter;
} // create_JM_filter()

/*******************************************************************************
* int zero_JM_filter(JM_filter_t* filter)
*
* This function will "zero" the filter of type JM_filter_t.
*******************************************************************************/
int zero_JM_filter(JM_filter_t* filter){
	int n = filter->order;
	int r = filter->rel_deg;
  int i;

	for(i=n, i>=0, i--){
    filter.outputs[i] = 0.0;
	}
	for(i=n-r, i>=0, i--){
		filter.inputs[i] = 0.0;
	}
	return 0;
} // zero_JM_filter
