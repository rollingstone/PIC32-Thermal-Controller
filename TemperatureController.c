


#include "TemperatureController.h"


#define VAR_MAX      0.001
#define MD_MAX      0.1


int PWMValue1;// = 3600; // 7200 max
int PWMValue2;// = 3600;
int DoAdjustTemperature = 0;

uint16_t    timer_count = 0;


//int     ThermoSPIChannel = SPI_CHANNEL1;

#define     THERMO_SPI_CHANNEL  SPI_CHANNEL1


int TempBufferMAXLength = 50;
float TemperatureList[50];
int TemperatureListLength = 40;
int CurrentTemperatureIDX = 0;
BOOL IsCircular = FALSE;


void TemperatureSystemInit()
{
    CS_INIT();
    DPDT_INIT();
}

void MapAllPins()
{
    
}


float PmodTC1_Temperature()
{
    char sbuf[100];
//    float value = 0;
    int data[4];
           
    SpiChnPutC(THERMO_SPI_CHANNEL, 0);
    data[0] = SpiChnGetC(THERMO_SPI_CHANNEL);
    SpiChnPutC(THERMO_SPI_CHANNEL, 0);
    data[1] = SpiChnGetC(THERMO_SPI_CHANNEL);
    SpiChnPutC(THERMO_SPI_CHANNEL, 0);
    data[2] = SpiChnGetC(THERMO_SPI_CHANNEL);
    SpiChnPutC(THERMO_SPI_CHANNEL, 0);
    data[3] = SpiChnGetC(THERMO_SPI_CHANNEL);
    
    int32_t  value = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    int32_t  vi = (int32_t)value / (int32_t)(1 << 18);
       
    float temp = ((float) vi) * 0.25;
    return temp;
}


float LM95071_Temperature()
{
//    char sbuf[100];
//    float value = 0;
    int data[4];
           
    SpiChnPutC(THERMO_SPI_CHANNEL, 0xAA);
    data[0] = SpiChnGetC(THERMO_SPI_CHANNEL);
    SpiChnPutC(THERMO_SPI_CHANNEL, 0xAA);
    data[1] = SpiChnGetC(THERMO_SPI_CHANNEL);
    
    int32_t  value = (((int32_t) data[0] << 8) | (int32_t) data[1]) & 0xFFFF;
    int32_t  vi = (value >> 2) ;
       
    float temp = (float) vi * 0.03125;
    
//    temp = -1;

//    sprintf(sbuf, "**************************** vi   %d   value %d  temp  %d\n", vi, value, (int)temp);
//    printf( "**************************** vi   %d   value %d  temp  %d\n", vi, value, (int)temp);
//    printf("**************************** vi   %d   value %d  temp  %f\n", vi, value, 3.12);
    
    return temp;
//    return (float)  value * 0.3125;
}


float LSD_Temperature(){
    float temp;
    
    LSD_CS_EX(CS_LOW);
    temp =  LM95071_Temperature();
    LSD_CS_EX(CS_HIGH);
    
    return temp;
}


float Peltier_Temperature(BOOL isTop){
    float temp;

    if(isTop)
    {
        PELTIER_TOP_CS_EX(CS_LOW);
        temp =  PmodTC1_Temperature();
        PELTIER_TOP_CS_EX(CS_HIGH);
    }
    else
    {
        PELTIER_BOTTOM_CS_EX(CS_LOW);
        temp =  PmodTC1_Temperature();
        PELTIER_BOTTOM_CS_EX(CS_HIGH);
    }
    
    return temp;
}

float ADisco_Temperature(){
    float temp;

    ADISCO_CS_EX(CS_LOW);
    temp =  PmodTC1_Temperature();
    ADISCO_CS_EX(CS_HIGH);
    
    return temp;
}

float FacePlate_Temperature(){
    float temp;

    FACE_PLATE_CS_EX(CS_LOW);
    temp =  PmodTC1_Temperature();
    FACE_PLATE_CS_EX(CS_HIGH);
    
    return temp;
}

float CS_N_Temperature(int n){
    float temp;

    switch(n)
    {
        case 1:
            CS_1(CS_LOW);
            temp =  PmodTC1_Temperature();
            CS_1(CS_HIGH);
            break;
        case 2:
            CS_2(CS_LOW);
            temp =  PmodTC1_Temperature();
            CS_2(CS_HIGH);
            break;
        case 3:
            CS_3(CS_LOW);
            temp =  PmodTC1_Temperature();
            CS_3(CS_HIGH);
            break;
        case 4:
            CS_4(CS_LOW);
            temp =  PmodTC1_Temperature();
            CS_4(CS_HIGH);
            break;
        case 5:
            CS_5(CS_LOW);
            temp =  PmodTC1_Temperature();
            CS_5(CS_HIGH);
            break;
    }
    
    return temp;
}

void DPDT_Direction(int id, BOOL isForward )
{
    if(id == 1)
    {
        DPDT_1(isForward);
    }
    else if(id == 2)
    {
        DPDT_2(isForward);
    }
}

float PID()
{
//    static float    
    return 0;
}

void ChangeTemperature(int pid, float error)
{
//    if(error > 0) // target > current
//    {
//        DPDT_2(0); // heat up
//    }
//    else if(error < 0)
//    {
//        DPDT_2(1); // cool down
//    }

    if(pid > 0) // target > current
    {
        DPDT_2(0); // heat up
    }
    else if(pid < 0)
    {
        DPDT_2(1); // cool down
//        printf("Cooling DOWN....\n");
    }
    
    SetPWMDutyCycle(1, (int)ABS_VALUE(pid));
}


void ChangeTemperatureWithPID(int pid, float error)
{
    if(error > 0) // target > current
    {
        DPDT_2(0); // heat up
    }
    else if(error < 0)
    {
        DPDT_2(1); // cool down
    }
    
    SetPWMDutyCycle(1, (int)abs(pid));
}

void AdjustTemperature(float target_temp)
{    
    float LSD_temp = LSD_Temperature();
    float peltier_top_temp = PmodTC1_Temperature();
    float peltier_bottom_temp = PmodTC1_Temperature();
    
    float error_value =  target_temp - LSD_temp;

    float sn = error_value < 0 ? -1:1;

    UInt16Value     *data_buffer;
    uint32_t        data_buffer_length = 20*1000; // bytes
    
    uint32_t    current_idx = 0;

    BOOL        is_allocated = FALSE;
    
    data_buffer = (UInt16Value *) AllocateMaxPossibleMemory(&data_buffer_length);
            
    WaitMS(100);
    
    if(data_buffer == NULL)
    {
        printf("ERROR: Cannot allocate memory for data_short (%d)\n", (int) data_buffer_length);
        return;
    }
    

    InitializePID(40.0, 0, 0.1, 0, target_temp);
    SetTargetTemperature(target_temp);
 
    float last_temp = LSD_Temperature();
    float current_temp = LSD_Temperature();
    
    int count = 0;
    int iw = 0;
    
    int max_iter = 200;
    int i = 0;
    
    float sum_temp = 0;
    int   n = 0;
    
    float temp_var = 999999;
    float var_limit = 1;
    
    float error_val = 0;
    int new_pid = 0;
    
    int k = 0;
    
    InitPWM(1500,20);
   

//    InitSPI1(4);
    
    DPDT_INIT();
    CS_INIT();
    
//    DPDT_2(1);
    
    int command[10];
    int command_length = 3;
    
    BOOL do_send_LSD_data = FALSE;
    int max_data_len = 0;
    
    int temperature_buffer[1000];
    int temperature_buffer_length = 1000;
    int temperature_idx = 0;
    int max_thermometer = 5;
    
    ClearTemperatureList();
    
    
    float sdev = 9999999;
    float mdev = 9999999;

    
    while(1)
    {
        
        if(ReadCommandFromUART1(command, command_length) )
        {
            int k;

            k =  0;
//            while(k < command_length)
//            {
//                printf(" commands %d\n", command[k]);
//                k++;
//            }
            
            if(command[0] == 99)
            {
                printf("Request for sending data_buffer received....\n");

                UInt16Value dv0;
                
                dv0._value = current_idx*2;

                WRITE_TO_UART1(dv0.v.upper);
                WRITE_TO_UART1(dv0.v.lower);
                
//                SPI1CONbits.ON = 0;
                int rx_len = SendDataBySPI2Slave(data_buffer, current_idx*2); // convert word idx to total bytes
                current_idx = 0;
//                SPI1CONbits.ON = 1;
                
                printf("Data sent.... (%d) by SPI2 Slave\n", rx_len);

            }
            else if(command[0] == 1) // LSD
            {
                if(command[1] == 1) // Set 1 , Get 0
                {
                    target_temp = (float) command[2];   // Value as Integer
                    SetTargetTemperature(target_temp);
                    ClearTemperatureList();
//                    InitializePID(40.0, 0, 0.1, 0, target_temp);

                    printf("New target temperature set to %d\n", (int) target_temp);
                }
                if(command[1] == 0) // send back some data from LSD
                {
                    max_data_len = command[2]; //
                    do_send_LSD_data = TRUE;
                    printf("Data send command received.... %d\n", max_data_len);

                }
            }
            else if(command[0] == 0xFF && command[1] == 0 && command[2] == 0xFF)
            {
                int uidx = 0;
                
                printf("**************** Read command received.....Sending %d bytes via UART1\n", current_idx*10);
                
//                WRITE_TO_UART2(0xAA);
                
                UInt16Value cval;
                
                cval._value = current_idx;
                
//                int kk = 0; while(kk++ < 1000);
                
//                mPORTASetBits(BIT_0);
                
                WRITE_TO_UART1((uint32_t) cval.v.upper);
                WRITE_TO_UART1((uint32_t) cval.v.lower);
                
                for(uidx = 0; uidx < current_idx; uidx+=5)
                {
                    WRITE_TO_UART1(data_buffer[uidx].v.upper);
                    WRITE_TO_UART1(data_buffer[uidx].v.lower);
                  
                    WRITE_TO_UART1(data_buffer[uidx+1].v.upper);
                    WRITE_TO_UART1(data_buffer[uidx+1].v.lower);

                    WRITE_TO_UART1(data_buffer[uidx+2].v.upper);
                    WRITE_TO_UART1(data_buffer[uidx+2].v.lower);

                    WRITE_TO_UART1(data_buffer[uidx+3].v.upper);
                    WRITE_TO_UART1(data_buffer[uidx+3].v.lower);

                    WRITE_TO_UART1(data_buffer[uidx+4].v.upper);
                    WRITE_TO_UART1(data_buffer[uidx+4].v.lower);
                }
                
                current_idx = 0;
            }
            else if(command[0] == 111 && command[1] == 0 && command[2] == 111) // is target temp reached
            {
                if(sdev < VAR_MAX && ABS_VALUE(mdev) < MD_MAX)
                {
                    WRITE_TO_UART1(111);
                    WRITE_TO_UART1(111);
                }
                else
                {
                    UInt16Value v16;
                    v16._value = (uint16_t) (current_temp*100.0);
                    
                    WRITE_TO_UART1(v16.v.upper);
                    WRITE_TO_UART1(v16.v.lower);
                    
                }
            }
            
            command[0] = 0;
            command[1] = 0;
            command[2] = 0;
        }


        if(DoAdjustTemperature == 1)
        {
            
//            mPORTGToggleBits(BIT_9);
//            mPORTGToggleBits(BIT_8);
         
//            mPORTGSetBits(BIT_8);
//            mPORTGSetBits(BIT_9);
//            

            current_temp = LSD_Temperature();
            AddTemperatureToList(current_temp);
            
            Calculate_SD_And_MD(target_temp, &sdev, &mdev);

//                k = 0;while(k++< 100);
            error_val = (target_temp - current_temp);

//                float efrac = error_val / 0.5;
//                
//                if(efrac < 0)
//                    efrac = -efrac;
//                
//                if(efrac < 0.5)
//                {
//                    
//                }

//                temp_var += error_val * error_val;
//                n++;

            int new_pid = (int) UpdatePID(current_temp); 

            if(ABS_VALUE(current_temp) > 70.0)
            {
                SetPWMDutyCycle(1, 0);
            }
            else
            {
                ChangeTemperature(new_pid, error_val);
            }

            
//              k=0; while(k++ < 100000);
//              asm("di");
//              mT1IntEnable(0);
//              mT2IntEnable(0);
            
            int int_val, frac_val;
            SplitFloat2Ints(current_temp, &int_val, &frac_val);
            int e = current_temp > target_temp? -( 1000-frac_val ): (1000-frac_val);  

//              printf("PID value == %-3d,\terror*100 == %-5d,\tTemperature == %-3d\n", (int)new_pid, (int)(error_val * 100.0), (int)current_temp);
            printf("Target %d\t,PID value == %3d,\tTemperature == %-03d + (%d/1000)\t",
                    (int) target_temp,
                    (int) new_pid, 
                    int_val, 
                    frac_val 
                    );

            float adisco_temp = ADisco_Temperature();
            float peltier_top = Peltier_Temperature(1);
            float peltier_bottom = Peltier_Temperature(0);
            float face_plate = FacePlate_Temperature();

            float mvalue = 100.0;

            if(current_idx < (data_buffer_length - 5) )
            {
                data_buffer[current_idx]._value   = (uint16_t) (current_temp * mvalue);
                data_buffer[current_idx+1]._value = (uint16_t) (adisco_temp * mvalue);
                data_buffer[current_idx+2]._value = (uint16_t) (peltier_top * mvalue);
                data_buffer[current_idx+3]._value = (uint16_t) (peltier_bottom * mvalue);
                data_buffer[current_idx+4]._value = (uint16_t) (face_plate * mvalue);

            }
            else
            {
                current_idx = 0;
            }



//                int t_ival = 

            printf("\tAdisco %-3d\t Peltier Top %-3d\t Peltier Bottom %-3d\tFace Plate %-03d\n", 
                        (int)adisco_temp, (int)peltier_top, (int)peltier_bottom, (int) face_plate);

//            printf("\tLSD %-3d\tAdisco %-3d\t Peltier Top %-3d\t Peltier Bottom %-3d\tFace Plate %-03d\n", 
//                        (int)data_buffer[current_idx]._value,
//                        (int)data_buffer[current_idx+1]._value, 
//                        (int)data_buffer[current_idx+2]._value, 
//                        (int)data_buffer[current_idx+3]._value, 
//                        (int)data_buffer[current_idx+4]._value);



            current_idx += 5; // 5 word = 10 bytes jump
            
//                printf("\n\ error*100 == %d,\n Temperature == %d\n\n", (int)new_pid, (int)(error_val * 100.0), (int)current_temp);

//                WRITE_TO_UART2(1);
//                WRITE_TO_UART2(1);
//                WRITE_TO_UART2((int) current_temp);

//            if(FALSE && temperature_idx < temperature_buffer_length)
//            {
//                temperature_buffer[temperature_idx] = (int) current_temp;
//
//                if(do_send_LSD_data == TRUE && temperature_idx >= max_data_len)
//                {
//                    printf("Sending back data.....\n");
//
//                    int ik = 0;
//
//                    WRITE_TO_UART2(1);
//                    WRITE_TO_UART2(max_data_len);
//
//                    for(ik  = 0; ik < max_data_len; ik++)
//                    {
//                        WRITE_TO_UART2(temperature_buffer[ik]);
//                        int ff; 
//                         ff = 0; while(ff++ < 10000);
//                    }
//                    do_send_LSD_data = FALSE;
//                    max_data_len = 0;
//                    temperature_idx = 0;
//                }
//                else
//                {
//                    temperature_idx++;
//                }

//            }
//                WaitMS(1);
//                k=0; while(k++ < 100000);
//            mT1IntEnable(1);
//            mT2IntEnable(1);

//            asm("ei");


//            mPORTGClearBits(BIT_8);
//            mPORTGClearBits(BIT_9);

            DoAdjustTemperature = 0;
        }
 //        SetPWMDutyCycle(1, GetLastPID());
//        while(iw++ < 100);
    }
    
}

void __ISR( _TIMER_2_VECTOR, IPL7SRC) __T2Interrupt(void)
 {
    
    mT2ClearIntFlag();         // clear this interrupt .
 } 


void TestUART2DataSendToRaspberryPi()
{
    char buffer[11];
    int count  = 0;
    int idx = 0;
    
    for(idx = 0; idx < 10; idx++)
    {
        buffer[idx] = idx;
    }
    
    buffer[10] = 0;
    
    while(1)
    {
        if(DoAdjustTemperature == 1)
        {
//            mPORTGSetBits(BIT_8 | BIT_9);
//            mPORTGToggleBits(BIT_8 | BIT_9);
            
            printf("Sending to UART2 %d\n", count);
            
//            putsUART2(buffer);
            WRITE_TO_UART2(count);
            
//            UART_DATA   data;
//            data.__data = count;
//            UARTSendData(1, data);
            count++;
            
            count %= 128;
            
//            mPORTGClearBits(BIT_8 | BIT_9);
            DoAdjustTemperature = 0;
        }
    }
}

void __ISR( _TIMER_1_VECTOR, IPL3AUTO) __T1Interrupt(void)
{
    static uint32_t     t1_counter = 0;
    
    if(++t1_counter >= TIMER_1_SCALE_DOWN_FACTOR)
    {
        t1_counter = 0;
        DoAdjustTemperature = 1;
        timer_count++;
//        mPORTAToggleBits(BIT_0 | BIT_1);
    }
   
//    mPORTBToggleBits(BIT_14);
    mT1ClearIntFlag();         // clear this interrupt .
} 


void __ISR( _TIMER_45_VECTOR, IPL3AUTO) __T4Interrupt(void)
{
    if(DoAdjustTemperature == 0)
        DoAdjustTemperature = 1;

    timer_count++;
    
//    mPORTBToggleBits(BIT_14);
 
    mPORTAToggleBits(BIT_0 | BIT_1);
//    printf("******** Interrupt T1 is called************\n");
    mT4ClearIntFlag();         // clear this interrupt .
} 



//void __ISR( _TIMER_2_VECTOR, IPL7SRC) T2Interrupt(void)
// {
//    if(DoAdjust == 0)
//        DoAdjust = 1;
//    
////    SetDCOC1PWM(PWM1_Value);    // these functions send a new value (0 to 7200) to PWM modulation with some PWM value.
////    SetDCOC2PWM(PWM2_Value);    // interrupt will fire again 7200 counts later than now  = 200us. (at 5 khz)
//
////    OC1RS = ((PR2+1) *  PWM1_DC) /(int)100;
////    OC3RS = ((PR2+1) *  PWM2_DC) /(int)100;
//
//    
////    printf("Hello from interrupt\n");
//    
//    
////    if(clock_scale_down_count++ > clock_scale_down)
////    {
//////        mPORTDToggleBits(BIT_0);
////        clock_scale_down_count = 0;
////    }
////    
////    printf("INT2  *******************\n");
//    
//    mT2ClearIntFlag();         // clear this interrupt .
// } 

void SPI1_TempMeasurement_Test()
{
    float temp = 0;
        // SS1 ==> RB2
    
//    CS_INIT();

    mPORTBSetPinsDigitalOut(BIT_2);
    mPORTBSetBits(BIT_2); 
    int kcount = 0;
    
    printf("Started reading temperature....\n");
    while(1)
    {
//       ADISCO_CS(CS_LOW);
       
//       PELTIER_TOP_CS(CS_LOW);
       PELTIER_BOTTOM_CS(CS_LOW);
//       FACE_PLATE_CS(CS_LOW);

       mPORTBClearBits(BIT_2); 
       temp = PmodTC1_Temperature();
       mPORTBSetBits(BIT_2); 
       

       PELTIER_BOTTOM_CS(CS_HIGH);
//       FACE_PLATE_CS(CS_HIGH);
       
//       PELTIER_TOP_CS(CS_HIGH);
//       ADISCO_CS(CS_HIGH);
       
       int frac_temp =(int) ( ((float) temp - ((float) (int) temp) ) * 100.0 );
       
       printf("%d:\tTemperature == %d + (%d/100)\n", kcount++, (int) (temp), frac_temp);
       WaitMS(100);
    }
}



void SPI1_TempMeasurement_LM_Thermo_Test()
{
    float temp = 0;
        // SS1 ==> RB2
//    mPORTBSetPinsDigitalOut(BIT_2);
//    mPORTBSetBits(BIT_2); 
    int kcount = 0;
    LSD_CS(CS_HIGH);
    
    while(1)
    {
//       mPORTBClearBits(BIT_2); 
       LSD_CS_EX(CS_LOW);

       temp = LM95071_Temperature();
       
       int frac_temp =(int) ( ((float) temp - ((float) (int) temp) ) * 1000.0 );
       
       LSD_CS(CS_HIGH);
//       mPORTBSetBits(BIT_2); 
       
       
       printf("%d:\tTemperature == %d + (%d/1000)\n", kcount++, (int) temp, frac_temp);
       WaitMS(100);
//       break;
    }
}


void Calculate_SD_And_MD(float target_temp, float *sd, float *md)
{
    int   i = 0, n = 0;    
    float mu1 = 0;
    float mu2 = 0;

    float sigma2 = 0;
    float mean_dev  = 0;
    
    float mu = target_temp;

    for(i = 0; i < (TempBufferMAXLength); i++)
    {
        float temp = TemperatureList[i];
        if((temp) > -999.0)
        {
            float mdev = (temp - mu);
            
            mean_dev += mdev;
            sigma2 += mdev * mdev;
            n++;
        }
    }
    
    mean_dev /= (float) n;
    sigma2 /= (float) n;
    
    *sd = sigma2;
    *md = mean_dev;
    
    printf("VAR = %d/10000, MD = %d/10000\n", (int) (sigma2*10000.0), (int) (*md*10000.0));
    
}


void AddTemperatureToList(float temp_value)
{

    TemperatureList[CurrentTemperatureIDX] = temp_value;
    CurrentTemperatureIDX++;
    CurrentTemperatureIDX %= TemperatureListLength;
}

void ClearTemperatureList()
{
    int i = 0;
    while(i < TempBufferMAXLength)
    {
        TemperatureList[i] = -999.0;
        i++;
    }
    
    TemperatureList[0] = 10.0;
    
    CurrentTemperatureIDX = 0;
    IsCircular = FALSE;
}




