/*
 * queternion.h
 *
 *  Created on: Jul 11, 2024
 *      Author: yahya
 */

#ifndef INC_QUETERNION_H_
#define INC_QUETERNION_H_

/*
void normalize_quaternion(double* q);
void update_quaternion(double* q, double gx, double gy, double gz, double dt);
void quaternion_multiply(double* q1, double* q2, double* result);
void quaternion_inverse(double* q, double* result);
void rotate_vector_by_quaternion(double* q, double* v, double* result);
*/

void updateQuaternion(float gx, float gy, float gz, float dt);
void quaternionToEuler(float* roll, float* pitch, float* yaw);

#endif /* INC_QUETERNION_H_ */
