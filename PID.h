/* 
 * File:   PID.h
 * Author: kamal
 *
 * Created on November 5, 2015, 5:42 PM
 */

#ifndef PID_H
#define	PID_H

#ifdef	__cplusplus
extern "C" {
#endif

void InitializePID(float Kp_val, float Kd_val, float Target_temp);
float UpdatePID(float current_temperature);
inline void SetTargetTemperature(float target_temp);
inline float GetLastPID();
inline float GetCurrentPID();

float IteratePID(float current_temperature);
void SetNewTargetTemperature(float target_temp);



#ifdef	__cplusplus
}
#endif

#endif	/* PID_H */

