/*
 * queternion.c
 *
 *  Created on: Jul 11, 2024
 *      Author: yahya
 */
#include "queternion.h"
#include "math.h"
#include "reset_detect.h"

extern backup_sram_datas_s *saved_datas;

float euler[3];		//pitch roll yaw

void updateQuaternion(float gx, float gy, float gz, float dt) {
  // Convert angular velocities to quaternion rates of change
  float qDot1 = 0.5f * (-saved_datas->q[1] * gx - saved_datas->q[2] * gy - saved_datas->q[3] * gz);
  float qDot2 = 0.5f * (saved_datas->q[0] * gx + saved_datas->q[2] * gz - saved_datas->q[3] * gy);
  float qDot3 = 0.5f * (saved_datas->q[0] * gy - saved_datas->q[1] * gz + saved_datas->q[3] * gx);
  float qDot4 = 0.5f * (saved_datas->q[0] * gz + saved_datas->q[1] * gy - saved_datas->q[2] * gx);

  // Integrate to get new quaternion values
  saved_datas->q[0] += qDot1 * dt;
  saved_datas->q[1] += qDot2 * dt;
  saved_datas->q[2] += qDot3 * dt;
  saved_datas->q[3] += qDot4 * dt;

  // Normalize quaternion to prevent drift
  float norm = sqrt(saved_datas->q[0] * saved_datas->q[0] + saved_datas->q[1] * saved_datas->q[1] + saved_datas->q[2] * saved_datas->q[2] + saved_datas->q[3] * saved_datas->q[3]);
  saved_datas->q[0] /= norm;
  saved_datas->q[1] /= norm;
  saved_datas->q[2] /= norm;
  saved_datas->q[3] /= norm;
}

void quaternionToEuler(void) {
  euler[1] = atan2(2.0f * (saved_datas->q[0] * saved_datas->q[1] + saved_datas->q[2] * saved_datas->q[3]), 1.0f - 2.0f * (saved_datas->q[1] * saved_datas->q[1] + saved_datas->q[2] * saved_datas->q[2])) * (180.0 / M_PI);
  euler[0] = asin(2.0f * (saved_datas->q[0] * saved_datas->q[2] - saved_datas->q[3] * saved_datas->q[1])) * (180.0 / M_PI);
  euler[2] = atan2(2.0f * (saved_datas->q[0] * saved_datas->q[3] + saved_datas->q[1] * saved_datas->q[2]), 1.0f - 2.0f * (saved_datas->q[2] * saved_datas->q[2] + saved_datas->q[3] * saved_datas->q[3])) * (180.0 / M_PI);
}


float quaternionToTheta(){

	float theta = 0.0;

	float r13 = 2 * saved_datas->q[1] * saved_datas->q[3] + 2 * saved_datas->q[2] * saved_datas->q[0];
	float r23 = 2 * saved_datas->q[2] * saved_datas->q[3] - 2 * saved_datas->q[1] * saved_datas->q[0];
	float r33 = 1 - 2 * saved_datas->q[1] * saved_datas->q[1] - 2 * saved_datas->q[2] * saved_datas->q[2];

	float z_x = r13;
	float z_y = r23;
	float z_z = r33;

	float dotProduct = z_z;
	float magnitude = sqrt(z_x * z_x + z_y * z_y + z_z * z_z);

	theta = acos(dotProduct / magnitude) * 180.0 / 3.14;
	return theta;
}

// İvmeölçerden başlangıç quaternioni hesaplama
void getInitialQuaternion() {

    double norm = sqrt(BMI_sensor.acc_z * BMI_sensor.acc_z + BMI_sensor.acc_x * BMI_sensor.acc_x + BMI_sensor.acc_y * BMI_sensor.acc_y);
    double accel_temp[3];

    accel_temp[0] = (double)BMI_sensor.acc_x;
    accel_temp[1] = (double)BMI_sensor.acc_y;
    accel_temp[2] = (double)BMI_sensor.acc_z;

    accel_temp[0] /= norm;
    accel_temp[1] /= norm;
    accel_temp[2] /= norm;

    double q_temp[4];

    q_temp[0] = sqrt(1.0 -accel_temp[1]) * 0.5;
    double k = 0.5 / q_temp[0];
    q_temp[1] = accel_temp[0] * k * 0.5;
    q_temp[2] = accel_temp[2] * k * 0.5;
    q_temp[3] = 0.0;

    norm = sqrt(q_temp[0] * q_temp[0] + q_temp[1] * q_temp[1] + q_temp[2] * q_temp[2] + q_temp[3] * q_temp[3]);

    saved_datas->q[0] = q_temp[0] / norm;
    saved_datas->q[1] = q_temp[1] / norm;
    saved_datas->q[2] = q_temp[2] / norm;
    saved_datas->q[3] = 0.0f;
}

void quaternionSet_zero(void)
{
	saved_datas->q[0] = 1.0;
	saved_datas->q[1] = 0.0;
	saved_datas->q[2] = 0.0;
	saved_datas->q[3] = 0.0;
}

