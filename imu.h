#ifndef IMU_H
#define IMU_H

void imu_init(void);
void imu_updateQuaternion(void);
void imu_get_roll_pitch_yaw(float *roll, float *pitch, float *yaw);
void imu_update_phi_from_quarternion(void);
void imu_get_phi(float *p_phi);
void imu_get_phi_rate(float *p_phi_rate);
uint32_t current_microseconds(void);
float current_milliseconds(void);
bool is_imu_data_ready(void);
void imu_update_kalman(void);

#endif //IMU_H
