/*
 * queternion.h
 *
 *  Created on: Jul 11, 2024
 *      Author: yahya
 */

#ifndef INC_QUETERNION_H_
#define INC_QUETERNION_H_


void updateQuaternion(float gx, float gy, float gz, float dt);
void quaternionToEuler(float* roll, float* pitch, float* yaw);
void quaternionSet_zero(void);

#endif
