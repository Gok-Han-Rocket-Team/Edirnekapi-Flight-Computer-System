/*
 * queternion.c
 *
 *  Created on: Jul 11, 2024
 *      Author: yahya
 */
#include "queternion.h"
#include "math.h"

static float q[4];
float euler[3];		//pitch roll yaw

void updateQuaternion(float gx, float gy, float gz, float dt) {
  // Convert angular velocities to quaternion rates of change
  float qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
  float qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
  float qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
  float qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

  // Integrate to get new quaternion values
  q[0] += qDot1 * dt;
  q[1] += qDot2 * dt;
  q[2] += qDot3 * dt;
  q[3] += qDot4 * dt;

  // Normalize quaternion to prevent drift
  float norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] /= norm;
  q[1] /= norm;
  q[2] /= norm;
  q[3] /= norm;
}

void quaternionToEuler(void) {
  euler[1] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * (180.0 / M_PI);
  euler[0] = asin(2.0f * (q[0] * q[2] - q[3] * q[1])) * (180.0 / M_PI);
  euler[2] = atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * (180.0 / M_PI);
}

void quaternionSet_zero(void)
{
	q[0] = 1.0f;
	q[1] = 0.0f;
	q[2] = 0.0f;
	q[3] = 0.0f;
}
