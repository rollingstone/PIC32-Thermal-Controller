//
//
//class PID:
//    """
//    Classe qui gère la boucle de rétroaction du rapport cyclique du peltier sur la température IMS µLSD avec un ago de PID modifié: 
//    pwm_peltier = former_pwm_peltier + P*(set_temperature_lsd - measured_temperature_lsd) + D*(former_measured_temperature_lsd - measured_temperature_lsd)
//       
//    """
//
//    def __init__(self, P=2.0, D=0.0, Derivator=0, PID_min=0, PID_max=99):
//
//        self.Kp=P
//        #self.Ki=I
//        self.Kd=D
//        self.Derivator=Derivator
//        self.PID_min=PID_min
//        self.PID_max=PID_max
//
//        self.set_point=0.0
//        self.error=0.0
//        #self.list_Error = np.zeros(Integrator_nb_values)
//        
//    def update(self,current_value,former_PID):
//        """
//        Calculate PID output value for given reference input and feedback
//        """
//
//        self.error = self.set_point - current_value
//
//        self.P_value = self.Kp * self.error
//        self.D_value = self.Kd * ( self.error - self.Derivator)
//        self.Derivator = self.error
//
//
//        PID = former_PID + self.P_value + self.D_value
//        #print 'P_value ', self.P_value, ' D_value ', self.D_value
//        if PID > self.PID_max:
//            PID = self.PID_max
//        elif PID < self.PID_min:
//            PID = self.PID_min
//    
//        return PID
// 
//    
//    def setPoint(self,set_point):
//        """
//        Initilize the setpoint of PID
//        """
//        self.set_point = set_point
//        #self.Integrator=0
//        self.Derivator=0


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

//    Error *= Error;
//    if(Error < 0)
//    {
//        Error = -Error;
//        cool_factor = 2;//3.7;
//    }
//    else
//    {
//        cool_factor = 1.08;
//    }
//    if(Error < 0.2)
//        return LastPIDValue;
    
//    float pid = LastPIDValue + cool_factor * Kp/500.0 * Error + 1.0 * (Error - Derivative);

    pid = LastPIDValue + (float)0.01 * Error + 1.0 * (Error - Derivative);
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
