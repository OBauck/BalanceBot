
#include <math.h>

#include "nrf.h"
#include "app_mpu.h"
#include "app_error.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "imu.h"
#include "SEGGER_RTT.h"

//tuning, rule: Ki = 0.05...0.1Kp^2
#define imu_mahonytwoKpDef (2.0f * 0.5f) // 2 * proportional gain
#define imu_mahonytwoKiDef (2.0f * 0.025f) // 2 * integral gain

#define GYRO_GAIN 0.00106526f	//2000*M_PI/32768/180
#define ACC_GAIN 0x4000			//2^16

#define MPU_INT_PIN 31

#define M_PI 3.14156

const nrf_drv_timer_t timer = NRF_DRV_TIMER_INSTANCE(1);

accel_values_t accel_values;
gyro_values_t gyro_values;

float q0, q1, q2, q3;
float roll, pitch, yaw;
float rollGy, pitchGy, yawGy;
float integralFBx, integralFBy, integralFBz;
float phi;

bool m_imu_data_ready = false;

/**
 * @brief Simple interrupt handler setting a flag indicating that data is ready
 *
 */
static void int_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    //imu_updateQuaternion();
	m_imu_data_ready = true;
}

/**
 * @brief Function for initiating the GPIOTE module and enable the 
 * nRF5 to trigger an interrupt on a Low-To-High event on pin MPU_MPU_INT_PIN
 *
 */
static void gpiote_setup(void)
{
    uint32_t err_code;

	if(!nrf_drv_gpiote_is_init())
	{
		err_code = nrf_drv_gpiote_init();
		APP_ERROR_CHECK(err_code);
	}
    
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);

    err_code = nrf_drv_gpiote_in_init(MPU_INT_PIN, &in_config, int_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(MPU_INT_PIN, true);
}

static void timer_dummy_handler(nrf_timer_event_t event_type, void* p_context)
{
}

static void timer_setup(void)
{
	uint32_t err_code;
	
	err_code = nrf_drv_timer_init(&timer, NULL, timer_dummy_handler);
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_timer_enable(&timer);
}

//returns the current value of the timer
uint32_t current_microseconds(void)
{
	return nrf_drv_timer_capture(&timer, NRF_TIMER_CC_CHANNEL0);
}

float current_milliseconds(void)
{
	return (float)current_microseconds()/1000.0f;
}

void imu_init()
{
	uint32_t err_code;
    // Initiate MPU driver
    err_code = mpu_init();
    APP_ERROR_CHECK(err_code); // Check for errors in return value
    
    // Setup and configure the MPU with intial values
    mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); // Load default values
    // The following line decieds how fast the nRF5 will sample the MPU for data. If you sample to fast
    // the Softdevice might not be able to transmit all the packets and you will receive a BLE_ERROR_NO_TX_PACKETS event.
    p_mpu_config.smplrt_div = 9;   // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_2G; // Set accelerometer full scale range to 2G
	p_mpu_config.gyro_config.fs_sel = GFS_2000DPS; //Set gyrescope full scale range to 2000 deg/s
    err_code = mpu_config(&p_mpu_config); // Configure the MPU with above values
    APP_ERROR_CHECK(err_code); // Check for errors in return value
	
	// This is a way to configure the interrupt pin behaviour
    mpu_int_pin_cfg_t p_int_pin_cfg = MPU_DEFAULT_INT_PIN_CONFIG(); // Default configurations
    p_int_pin_cfg.int_rd_clear = 1; // When this bit is equal to 1, interrupt status bits are cleared on any read operation
    err_code = mpu_int_cfg_pin(&p_int_pin_cfg); // Configure pin behaviour
    APP_ERROR_CHECK(err_code); // Check for errors in return value
    
    // Enable the MPU interrupts
    mpu_int_enable_t p_int_enable = MPU_DEFAULT_INT_ENABLE_CONFIG();
    p_int_enable.data_rdy_en = 1; // Trigger interrupt everytime new sensor values are available
    err_code = mpu_int_enable(&p_int_enable); // Configure interrupts
    APP_ERROR_CHECK(err_code); // Check for errors in return value
	
	gpiote_setup();
	timer_setup();
	
	q0 = 1.0;
	q1 = q2 = q3 = 0;
	integralFBx = integralFBy = integralFBz = 0;
	pitch = roll = yaw = 0;
}

void imu_mahonyUpdate(gyro_values_t *p_gyro_values, accel_values_t *p_accel_values, uint32_t delta_t_us) 
{
	float gx = (float)p_gyro_values->x * GYRO_GAIN;
	float gy = (float)p_gyro_values->y * GYRO_GAIN;
	float gz = (float)p_gyro_values->z * GYRO_GAIN;
		
	float ax = (float)p_accel_values->x / ACC_GAIN;
	float ay = (float)p_accel_values->y / ACC_GAIN;
	float az = (float)p_accel_values->z / ACC_GAIN;
		
	float delta_t_s = (float)delta_t_us / 1000000.0f;
	
    float dummy = ay;
    ay = -az;
    az = dummy;
    float norm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
     
    //look at http://www.olliw.eu/2013/imu-data-fusing/ almost at the bottom for decription
 
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
 
        // Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az);
        ax /= norm;
        ay /= norm;
        az /= norm;
 
        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;
 
        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);
 
        // Compute and apply integral feedback if enabled
        if(imu_mahonytwoKiDef > 0.0f) {
            integralFBx += imu_mahonytwoKiDef * halfex * (1.0f * delta_t_s); // integral error scaled by Ki
            integralFBy += imu_mahonytwoKiDef * halfey * (1.0f * delta_t_s);
            integralFBz += imu_mahonytwoKiDef * halfez * (1.0f * delta_t_s);
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }
 
        // Apply proportional feedback
        gx += imu_mahonytwoKpDef * halfex;
        gy += imu_mahonytwoKpDef * halfey;
        gz += imu_mahonytwoKpDef * halfez;
    }
 
    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f * delta_t_s));       // pre-multiply common factors
    gy *= (0.5f * (1.0f * delta_t_s));
    gz *= (0.5f * (1.0f * delta_t_s));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
 
    // Normalise quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

void imu_updateQuaternion()
{
	uint32_t err_code;
	
	err_code = mpu_read_accel(&accel_values);
	APP_ERROR_CHECK(err_code);
	
	//SEGGER_RTT_printf(0, "acc values: %d\t%d\t%d\t", accel_values.x, accel_values.y, accel_values.z);
	
	err_code = mpu_read_gyro(&gyro_values);
	APP_ERROR_CHECK(err_code);
	
	//SEGGER_RTT_printf(0, "gyro values: %d\t%d\t%d\n", gyro_values.x, gyro_values.y, gyro_values.z);
	
	m_imu_data_ready = false;
	
	//TODO: try different filter to get the angle (complementary filter and/or kalman filter)
	imu_mahonyUpdate(&gyro_values, &accel_values, 10000);	//10000us = 100Hz
}

void imu_get_roll_pitch_yaw(float *p_roll, float *p_pitch, float *p_yaw)
{
	
	*p_yaw = asin(2*q1*q2 - 2*q0*q3);		
	*p_roll = -asin(2*q1*q3 + 2*q0*q2);	
	*p_pitch = asin(2*q2*q3 - 2*q0*q1);
	
	/**yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1);
	*roll = -atan2(2*q0*q2 + 2*q1*q3, 2*q0*q0 + 2*q3*q3 - 1);
	*pitch = atan2(2*q2*q3 - 2*q0*q1, 2*q0*q0 + 2*q3*q3 - 1);*/
}

void imu_update_phi_from_quarternion(void)
{
	phi = atan2(-(2*q2*q3 - 2*q0*q1), -(2*q0*q0 + 2*q3*q3 - 1)) / 3.14156 * 180;
	//phi = asin(2*q2*q3 - 2*q0*q1) / 3.14156 * 180;
}

void imu_get_phi(float *p_phi)
{
	*p_phi = phi;
}

void imu_get_phi_rate(float *p_phi_rate)
{
	*p_phi_rate = gyro_values.x;
}

bool is_imu_data_ready(void)
{
	/*
	if(m_imu_data_ready)
	{
		m_imu_data_ready = false;
		return true;
	}
	
	return false;
	*/
	return m_imu_data_ready;
}

float getkalmanangle(float newkalmanangle, float newRate, float dt) 
{
	
	/* Kalman filter variables */
    static float Q_kalmanangle = 0.001; // Process noise variance for the accelerometer
    static float Q_bias = 0.003; // Process noise variance for the gyro bias
    static float R_measure = 0.03; // Measurement noise variance - this is actually the variance of the measurement noise

    static float kalmanangle = 0; // The kalmanangle calculated by the Kalman filter - part of the 2x1 state vector
    static float bias = 0; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    static float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getkalmanangle to update the rate

    static float P[2][2] = {{0,0},{0,0}}; // Error covariance matrix - This is a 2x2 matrix
    static float K[2]; // Kalman gain - This is a 2x1 vector
    static float y; // kalmanangle difference
    static float S; // Estimate error
	
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	rate = newRate - bias;
	kalmanangle += dt * rate;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_kalmanangle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	S = P[0][0] + R_measure;
	/* Step 5 */
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;

	// Calculate kalmanangle and bias - Update estimate with measurement zk (newkalmanangle)
	/* Step 3 */
	y = newkalmanangle - kalmanangle;
	/* Step 6 */
	kalmanangle += K[0] * y;
	bias += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	P[0][0] -= K[0] * P[0][0];
	P[0][1] -= K[0] * P[0][1];
	P[1][0] -= K[1] * P[0][0];
	P[1][1] -= K[1] * P[0][1];

	return kalmanangle;
};

void imu_update_kalman(void)
{	
	static float phi_filtered = 0;
	static const float dt = 0.01;	//100Hz
	
	uint32_t err_code;
	
	err_code = mpu_read_accel(&accel_values);
	APP_ERROR_CHECK(err_code);
	
	//SEGGER_RTT_printf(0, "acc values: %d\t%d\t%d\t", accel_values.x, accel_values.y, accel_values.z);
	
	err_code = mpu_read_gyro(&gyro_values);
	APP_ERROR_CHECK(err_code);
	
	m_imu_data_ready = false;
	
	//SEGGER_RTT_printf(0, "gyro values: %d\t%d\t%d\n", gyro_values.x, gyro_values.y, gyro_values.z);
	
	float gx = gyro_values.x * GYRO_GAIN;
	float acc_phi = atan2(-(double)accel_values.z, -(double)accel_values.y) * 180 / M_PI;
	phi_filtered = 0.9 * (phi_filtered - gx * dt) + 0.1 * acc_phi;	
	
	phi = -phi_filtered;//-getkalmanangle(phi_filtered, gx, dt);
	
	char str[100];
	sprintf(str, "gx: %.1f, acc_phi: %.1f, phi_filtered: %.1f, phi: %.1f\n", gx, acc_phi, phi_filtered, phi);
	//SEGGER_RTT_printf(0,"%s", str);
}
