

#include <time.h>


float Kp;
float Ki;
float Kd;

float Derivative = 0;
float Integral = 0;

float PID_min = -80;//-100;
float PID_max = 80;//100;


float TargetTemperature = 25; // C
float Error = 0;
float LastError = 0;

float CurrentPIDValue;
float LastPIDValue;


float PreviousError = 0;

time_t  LastTime;

int     HC_Offset = 0;


void InitializePID(float Kp_val, float Ki_val, float Kd_val, float Derivative_val, float Target_temp)
{
    Kp = Kp_val;
    Ki = Ki_val;
    Kd = Kd_val;
    
    Derivative = Derivative_val;
    Integral = 0;
    TargetTemperature = Target_temp;
    
    Error = 0;
    
    CurrentPIDValue = 0;
    LastPIDValue = 0;
//    pid = 0;
    
    
//    if()
    
}

void SetNewTargetTemperature(float target_temp)
{
    TargetTemperature = target_temp;
    Integral = 0;
    PreviousError = 0;
}



float UpdatePID(float current_temperature)
{
    float pid;
    float cool_factor = 1;
    
    Error = TargetTemperature - current_temperature;

    pid = LastPIDValue + (float)0.2 * Error + 10.0 * (Error - Derivative);
//
//    if( ABS_VALUE(Error) < 1)
//      pid = LastPIDValue + 0.1 * Error + 1.0 * (Error - Derivative);
//    else
//    {
//        if(Error < 0)
//        {
//            pid = 3 * Error;
//        }
//        else
//        {
//            pid = 1 * Error;
//        }
//    }

//    float pid = LastPIDValue + Kp/500.0 * Error; // + 0.1 * (Error - Derivative);
    
    Derivative = Error;
    
    if(pid > PID_max)
    {
        pid =  PID_max;
    }
    else if(pid < PID_min)
    {
        pid = PID_min;//PID_min;
    }
    
    CurrentPIDValue = pid;
    LastPIDValue = pid;
    
//    printf("PID = %d\n", (int)pid);
//    WaitMS(1);
    
    return pid;
}

inline void SetTargetTemperature(float target_temp)
{
    TargetTemperature = target_temp;
    Derivative = 0;
}

inline float GetLastPID()
{
    return LastPIDValue;
}

inline float GetCurrentPID()
{
    return CurrentPIDValue;
}

float IteratePID(float current_temperature)
{ 
    float Dt  = (float) (0 - LastTime);
    
    Error = TargetTemperature - current_temperature; 
       // track error over time, scaled to the timer interval
    Integral = Integral + (Error * Dt);
       // determine the amount of change from the last time checked
    Derivative = (Error - PreviousError) / Dt; 
       // calculate how much to drive the output in order to get to the 
       // desired setpoint. 
       // remember the error for the next time around.
    PreviousError = Error; 
    LastTime = 0;// clock();
    return (float) ((Kp * Error) + (Ki * Integral) + (Kd * Derivative));
 
}


int UpdateDutyCycleValue(float current_temperature)
{
    float pid = IteratePID(current_temperature);
    float pos_pid = pid < 0? -pid:pid;
    
    if(pos_pid > 99)
    {
        return 99; 
    }
    else
    {
        return (int) pos_pid;
    }
    
    return (int) pos_pid;
}
