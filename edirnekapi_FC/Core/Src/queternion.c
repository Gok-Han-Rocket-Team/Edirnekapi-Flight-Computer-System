/*
 * queternion.c
 *
 *  Created on: Jul 11, 2024
 *      Author: yahya
 */
#include "queternion.h"
#include "math.h"
double v[3] = {0.0, 0.0, 1.0};
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

void updateQuaternion(float gx, float gy, float gz, float dt) {
  // Convert angular velocities to quaternion rates of change
  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Integrate to get new quaternion values
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalize quaternion to prevent drift
  float norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}

void quaternionToEuler(float* roll, float* pitch, float* yaw) {
  // Convert quaternion to Euler angles
  *roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0 / M_PI);
  *pitch = asin(2.0f * (q0 * q2 - q3 * q1)) * (180.0 / M_PI);
  *yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (180.0 / M_PI);
}
