
#include "PeripheralSettingsAndMacros.h"

#include <plib.h>
#include <xc.h>
#include <time.h>

volatile int PWM1_Value = 10;
volatile int PWM2_Value = 100;

volatile int PWM1_DC = 50;
volatile int PWM2_DC = 50;

volatile int PWM1_DC_Value;
volatile int PWM2_DC_Value;


volatile int clock_scale_down = 10; // 10 fold slower execution 
volatile int clock_scale_down_count = 0; // 10 fold slower execution 

int DoAdjustPID = 0;
int DutyCyclePWM1 = 50;


BOOL    DoReadUART2 = FALSE;


void InitSystem()
{
    SYSTEMConfigPerformance(SYS_FREQ);  // This function sets the PB-Div to 1. Also optimises cache for 72Mhz etc..
    mOSCSetPBDIV(OSC_PB_DIV_2);           // Therefore, configure the PB bus to run at 1/2 CPU Frequency
                                                              // you may run at PBclk of 72Mhz if you like too (omit this step)
                                                              // This will double the PWM frequency.
    

//    SYSTEMConfigPB()
    INTEnableSystemMultiVectoredInt();
    
    InitUART1();
    __XC_UART = 1;
  
    // disable for testing purposes
//    InitUART2();

//    InitSPI2Slave();    
    
    InitSPI1(16);
    
//    InitSPI1Slave();
 
}

void InitSystem_Test()
{
    SYSTEMConfigPerformance(SYS_FREQ);  // This function sets the PB-Div to 1. Also optimises cache for 72Mhz etc..
    mOSCSetPBDIV(OSC_PB_DIV_2);           // Therefore, configure the PB bus to run at 1/2 CPU Frequency
                                                              // you may run at PBclk of 72Mhz if you like too (omit this step)
                                                              // This will double the PWM frequency.
    
//    SYSTEMConfigPB()
    INTEnableSystemMultiVectoredInt();
    
    InitUART1();
    __XC_UART = 1;
  
    // disable for testing purposes
//    InitUART2();
//
    InitSPI2Slave();    
    
//    InitSPI1(32);
    
//    InitSPI1Slave();
 
}


void WaitMS(unsigned int ms)
{
    unsigned int count = 0;
    unsigned int i = 0;
    
    while(count++ < ms)
    {
        i = 0;
        while(i++ < 10000)
        {
            NOP();
        }
    }
}


int InitUART1()
{
    int uart_id = 0;

//    #if defined (__32MX220F032D__) || defined (__32MX250F128D__)
// //   PPSInput(2,U2RX,RPB5); // Assign RPB5 as input pin for U2RX
// //   PPSOutput(4,RPB0,U2TX); // Set RPB0 pin as output for U2TX
//    #elif defined (__32MX430F064L__) || (__32MX450F256L__) || (__32MX470F512L__)
////    PPSInput(2,U1RX,RPF4); // Assign RPF4 as input pin for U1RX
////    PPSOutput(2,RPF5,U1TX); // Set RPF5 pin as output for U1TX
    
    PPS_Unlock();

    U1RXRbits.U1RXR = 2; // 0b0010; // RF4 input J11.46
    RPF5Rbits.RPF5R = 3; //0b0011; TX output J11.48
    
    PPS_Lock();
    
//    #endif
    uint32_t brate;
    
//    brate = 115200L;
    brate = 500000L;
//    brate = 460800;//921600L/2L;

    UARTConfigure((UART_MODULE)uart_id, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode((UART_MODULE)uart_id,(UART_FIFO_MODE) (UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY));
    UARTSetLineControl((UART_MODULE)uart_id,(UART_LINE_CONTROL_MODE) (UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1));
    UARTSetDataRate((UART_MODULE)uart_id, GetPeripheralClock(), brate);
    UARTEnable((UART_MODULE)uart_id, (UART_ENABLE_MODE) UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    return uart_id;
}

int InitUART2()
{
    int uart_id = 1;

//    mPORTCSetPinsDigitalIn(BIT_1 | BIT_3);
//    mPORTCSetPinsDigitalOut(BIT_2 | BIT_4);

//    mPORTCSetPinsDigitalIn(BIT_1 );
//    mPORTCSetPinsDigitalOut(BIT_4);
    
    mPORTDSetPinsDigitalOut(BIT_11);    // U2TX  J10.15
    mPORTDSetPinsDigitalIn(BIT_10 );    // U2RX J10.16
    
    PPS_Unlock();

//    U2RXRbits.U2RXR = 0b1010; // 0b0010; // RC1 input J10.18
//    RPC4Rbits.RPC4R = 0b0001; // U2TX  UTX  J10.21

    U2RXRbits.U2RXR = 0b0011; // 0b0010; // RD10 input J10.16
    RPD11Rbits.RPD11R = 0b0001; // U2TX  UTX  J10.15

   
    //    U2CTSRbits.U2CTSR = 0b1100; // RC3 .. Input  J10.20
//    RPC2Rbits.RPC2R = 0b0001; // U2RTS 0b0011; RTS output J10.19
    
    PPS_Lock();
    
    unsigned int brate;
    brate = 115200L;//115200L * 1L;
//    brate = 256000L;//115200L * 1L;
//    brate = 921600L;
    
//    U2MODEbits.BRGH = 4;
//    U2MODEbits.
 
    UARTConfigure((UART_MODULE)uart_id, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode((UART_MODULE)uart_id,(UART_FIFO_MODE) (UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY));
    UARTSetLineControl((UART_MODULE)uart_id,(UART_LINE_CONTROL_MODE) (UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1));
    UARTSetDataRate((UART_MODULE)uart_id, GetPeripheralClock(), brate);

    SetPriorityIntU2(UART_INT_PR3);
    mU2ClearAllIntFlags();
    mU2RXIntEnable(0);
//    
//    EnableIntU2RX;
//    mU2
//    mU2RXIntEnable(1);
    
    UARTEnable((UART_MODULE)uart_id, (UART_ENABLE_MODE) UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

//    UART_PERIPHERAL
    return uart_id;
}

void MapSPI1MasterPins()
{
    printf("Mapping SPI 1 Master Pins\n");

    UnlockPPS();
 
    SDI1Rbits.SDI1R = 0b1010; //10;  // 0b1010 // RPC4  // SDI1 // J10.44
    RPD0Rbits.RPD0R = 0b1000; // // SDO1 // J10.43
    RPB2Rbits.RPB2R = 0b0111; //7; //0b0111; // SS2 // J10.47
  
    LockPPS();

    mPORTDSetPinsDigitalOut(BIT_10);// SCK1 out
    mPORTCSetPinsDigitalIn(BIT_4);// SDI1 in
    CNPDCbits.CNPDC4 = 1; // pull down

    mPORTDSetPinsDigitalOut(BIT_0);// SDO1 out 

//    mPORTDSetPinsDigitalOut(BIT_9);// SS1 out
//    mPORTDSetBits(BIT_9); // set high by default}
//    CNPDDbits.CNPDD9 = 1; // pull down

    mPORTBSetPinsDigitalOut(BIT_2);// SS1 out
    mPORTBClearBits(BIT_2); // set high by default}
    CNPUBbits.CNPUB2 = 1; // pull up
    
}

void MapSPI1SlavePins()
{
    printf("Mapping SPI 1 Slave Pins (SPI 1 Slave mode may not function properly!!)\n");

    UnlockPPS();
    
    // output pin SDO1
    RPD0Rbits.RPD0R = 0b1000; // // SDO1 // RD0 // J10.43
 
    // input pins SDI1 and SS1

    SDI1Rbits.SDI1R = 0b1010;  // 0b1010 // RPC4  // SDI1
//    SDI1Rbits.SDI1R = 0b0010;  // 0b0010 // RPF5  // SDI1 // J10.52
    SS1Rbits.SS1R = 0b1111; // RB2 // SS1 // J10.46

//    RPB2Rbits.RPB2R = 0b0111; //0b0111; // SS1
//    RPD9Rbits.RPD9R = 7; //0b0111; // SS1
//    SS1Rbits.SS1R = 0; // RPD9 input

    LockPPS();

//    TRISD
    mPORTDSetPinsDigitalIn(BIT_10);// SCK1 out J10.41
    
    // SPI1 at J10.44
    mPORTCSetPinsDigitalIn(BIT_4);// SDI1 in J10.44
    mPORTCClearBits(BIT_4);
    CNPDCbits.CNPDC4 = 1; // pull up
    
    // SDI1 at J10.52
//   
//    mPORTFSetPinsDigitalIn(BIT_5);// SDI1 in J10.52
//    mPORTFClearBits(BIT_5);
//    CNPDFbits.CNPDF5 = 1;
            
    
    mPORTDSetPinsDigitalOut(BIT_0);// SDO1 out // J10.43
//    mPORTDSetPinsDigitalIn(BIT_9);// SS1 out
//    mPORTDClearBits(BIT_9); // set high by default}
//    CNPDDbits.CNPDD9 = 1; // pull down
    
    mPORTBSetPinsDigitalIn(BIT_2);// SS1 in J10.46
    mPORTBClearBits(BIT_2); // set high by default}
//    CNPUBbits.CNPUB2 = 1;
    CNPDBbits.CNPDB2 = 1;
}


void MapSPI2MasterPins()
{
    printf("Mapping SPI 2 Master Pins\n");

    PPS_Unlock();
 
    SDI2Rbits.SDI2R = 1; // RPG7 = 0b0001 // SDI2
    RPG8Rbits.RPG8R = 6; //0b0110; // SDO2
    RPG9Rbits.RPG9R = 6; //0b0110; // SS2
    
    PPS_Lock();

    mPORTGSetPinsDigitalOut(BIT_6);// SCK2 out
    mPORTGSetPinsDigitalIn(BIT_7);// SDI2 in
    
    CNPDGbits.CNPDG7 = 1; // pull down

    mPORTGSetPinsDigitalOut(BIT_8);// SDO2 out
    mPORTGSetPinsDigitalOut(BIT_9);// SS2 out
    CNPDGbits.CNPDG9 = 1; // pull down

}


void MapSPI2SlavePins()
{
    printf("Mapping SPI 2 Slave Pins\n");

    PPS_Unlock();
 
    SDI2Rbits.SDI2R = 1; // RPG7 = 0b0001 // SDI2
    RPG8Rbits.RPG8R = 6; //0b0110; // SDO2
    SS2Rbits.SS2R = 1;// RPG9 input
    
    PPS_Lock();
    
    mPORTGSetPinsDigitalIn(BIT_6);// SCK2 in
    mPORTGSetPinsDigitalIn(BIT_7);// SDI2 in
//    CNPDGbits.CNPDG7 = 1; // pull down
    mPORTGClearBits(BIT_7);
    mPORTGSetPinsDigitalOut(BIT_8);// SDO2 out
    CNPDGbits.CNPDG7 = 1;
  
    mPORTGSetPinsDigitalIn(BIT_9);// SS2 in // RG9
    CNPDGbits.CNPDG9 = 1; // pull down
    mPORTGClearBits(BIT_9); // set low by default
}

//
//int SetPWMDutyCycle(float duty_cycle)
//{
//    PWM1_DC_Value = (PR2 + 1) * ((float)duty_cycle / 100.0);
//    return PWM1_DC_Value;
//}


int SetPWMDutyCycle(int pwm_number, int duty_cycle)
{
    int dval;
    if(pwm_number == 1)
    {
        OC1RS = (PR2 + 1) * ((float)duty_cycle / 100.0);
        return OC1RS;
    }
    
    if(pwm_number == 2)
    {
        OC2RS = (PR2 + 1) * ((float)duty_cycle / 100.0);
        return OC2RS;
    }
    
    if(pwm_number == 3)
    {
        OC3RS = (PR2 + 1) * ((float)duty_cycle / 100.0);
        return OC3RS;
    }

    if(pwm_number == 4)
    {
        OC4RS = (PR2 + 1) * ((float)duty_cycle / 100.0);
        return OC4RS;
    }

    if(pwm_number == 5)
    {
        OC5RS = (PR2 + 1) * ((float)duty_cycle / 100.0);
        return OC5RS;
    }
    
}

void TurnOffPWM(int pwm_number)
{
    switch(pwm_number)
    {
        case 1:
            OC1CONbits.ON = 0;
            break;
        case 2:
            OC2CONbits.ON = 0;
            break;
        case 3:
            OC3CONbits.ON = 0;
            break;
        case 4:
            OC4CONbits.ON = 0;
            break;
        case 5:
            OC5CONbits.ON = 0;
            break;
    }
}

void TurnOnPWM(int pwm_number)
{
    switch(pwm_number)
    {
        case 1:
            OC1CONbits.ON = 1;
            break;
        case 2:
            OC2CONbits.ON = 1;
            break;
        case 3:
            OC3CONbits.ON = 1;
            break;
        case 4:
            OC4CONbits.ON = 1;
            break;
        case 5:
            OC5CONbits.ON = 1;
            break;
    }
}

void InitPWM(int pwm_frequency, int duty_cycle)
{
    int  samplerate = pwm_frequency;

    PPS_Unlock();
    RPD1Rbits.RPD1R = 0b1100; // OC1 // J11.20
//    RPD2Rbits.RPD2R = 0b1011; // OC3
    PPS_Lock();

    OC1CON = 0x0006;
    
    PR2 = ((int)SYS_FREQ/ (pwm_frequency))-1;
    
    OC1RS = (PR2+1) * ((float)duty_cycle /100.0);
    
//    OC2RS = ((PR2+1) *  PWM2_DC) /(int)100;
//    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // init OC1 module, T2 =source 
//    OpenOC2(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // init OC2 module, T2 =source(you could do more OCx)
//    OpenTimer2(T2_ON | T2_PS_1_256 | T2_SOURCE_INT, 0xFFFF);         // init Timer2 mode and period reg (PR2)

    T2CONbits.TCKPS = 0b101;
    
    mT2SetIntPriority(7);  // you don't have to use ipl7, but make sure INT definition is the same as your choice here
    mT2ClearIntFlag();     // make sure no int will be pending until 7200 counts from this point.  
    mT2IntEnable(1);       // allow T2 int

    T2CONbits.ON = 1;
    OC1CONbits.ON = 1;
    
//    printf("PR2 == %d", PR2, OC1RS, OC2RS);

//    SetDCOC1PWM(PWM1_Value);
//    SetDCOC2PWM(PWM2_Value);

//T2_32BIT_MODE_ON

//    mT2SetIntPriority(7);  // you don't have to use ipl7, but make sure INT definition is the same as your choice here
//    mT2ClearIntFlag();     // make sure no int will be pending until 7200 counts from this point.  
//    mT2IntEnable(1);       // allow T2 int
}



void InitPWM_v3(int pwm_frequency, int duty_cycle)
{
    int  samplerate = pwm_frequency;

    PPS_Unlock();
    RPD1Rbits.RPD1R = 0b1100; // OC1
//    RPD2Rbits.RPD2R = 0b1011; // OC3
    PPS_Lock();

    OC1CON = 0x0006;
    
    PR2 = ((int)SYS_FREQ/ (pwm_frequency*256))-1;
    
    int val = ((int)SYS_FREQ/ (pwm_frequency*1))-1;
    int dc = (PR2+1) * ((float)duty_cycle /100.0);

//    OC1RS = (PR2+1) * ((float)duty_cycle /100.0);
    
//    OC2RS = ((PR2+1) *  PWM2_DC) /(int)100;
    
    SetDCOC1PWM(dc);

    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // init OC1 module, T2 =source 
    
//    OpenOC2(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // init OC2 module, T2 =source(you could do more OCx)
    OpenTimer2(T2_ON | T2_PS_1_1 | T2_SOURCE_INT, val);         // init Timer2 mode and period reg (PR2)

    
//    PR2 = ((int)SYS_FREQ/ (pwm_frequency*256))-1;
    
//    T2CONSET = 0x8000;
//    OC1CONSET = 0x8000;
    
//    printf("PR2 == %d", PR2, OC1RS, OC2RS);

//    SetDCOC1PWM(PWM1_Value);
//    SetDCOC2PWM(PWM2_Value);

//T2_32BIT_MODE_ON

    mT2SetIntPriority(7);  // you don't have to use ipl7, but make sure INT definition is the same as your choice here
    mT2ClearIntFlag();     // make sure no int will be pending until 7200 counts from this point.  
    mT2IntEnable(1);       // allow T2 int
}


void InitTimer1(uint32_t freq)
{
    int freq_val = ((int)SYS_FREQ/ (freq))-1;
    
//    int correct_freq = freq * 
    
    PR1 = (uint32_t)SYS_FREQ/ ((uint32_t)(freq*( (uint32_t)128)) )-1;
    
    T1CONbits.TCKPS = 0b10; // 128
    
    mT1SetIntPriority(3);  // you don't have to use ipl3, but make sure INT definition is the same as your choice here
    mT1ClearIntFlag();     // make sure no int will be pending until 7200 counts from this point.  
    mT1IntEnable(1);       // allow T2 int

    T1CONbits.ON = 1;
}



//void InitPWM()
//{
//    SetDCOC1PWM(3600);
//    
//    INTEnableSystemMultiVectoredInt();         // make separate interrupts possible
// 
//    int samplerate = 100;
// 
//    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE,0,0); // init OC1 module, T2 =source 
//    OpenOC2(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE,0,0); // init OC2 module, T2 =source(you could do more OCx)
//    OpenTimer2(T2_ON | T2_PS_1_1 | T2_SOURCE_INT,FPB/samplerate);         // init Timer2 mode and period reg (PR2)
// 
//    PR2 = FPB/samplerate-1;
//
//    mT2SetIntPriority(7);  // you don't have to use ipl7, but make sure INT definition is the same as your choice here
//    mT2ClearIntFlag();     // make sure no int will be pending until 7200 counts from this point.  
//    mT2IntEnable(1);       // allow T2 int
//    
//    
//}





void InitPWM_v2(int sample_rate)
{
    
    PPS_Unlock();
    RPD1Rbits.RPD1R = 0b1100; // OC1
    RPD2Rbits.RPD2R = 0b1011; // OC3
 
    PPS_Lock();
    
//    mPORT
    
    int  samplerate = sample_rate;
    int PR2 = PBUS_CLOCK/samplerate-1;

    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // init OC1 module, T2 =source 
    OpenOC2(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // init OC2 module, T2 =source(you could do more OCx)
    OpenTimer2(T2_ON | T2_PS_1_1 | T2_SOURCE_INT, PBUS_CLOCK/samplerate);         // init Timer2 mode and period reg (PR2)

    SetDCOC1PWM(PWM1_Value);
    SetDCOC2PWM(PWM2_Value);
    
//T2_32BIT_MODE_ON
    

    mT2SetIntPriority(7);  // you don't have to use ipl7, but make sure INT definition is the same as your choice here
    mT2ClearIntFlag();     // make sure no int will be pending until 7200 counts from this point.  
    mT2IntEnable(1);       // allow T2 int
}

void InitSPI1Slave()
{
//    printf("Mapping SPI 1 Slave Pins\n");
    
    
    MapSPI1SlavePins();
    
    SpiChnConfigure(SPI_CHANNEL1, (SpiConfigFlags)(SPI_CONFIG_MODE8 | SPI_CONFIG_ON |
                                                   0));
//    SpiChnSetBrg(SPI_CHANNEL2, baud_rate);
    SpiChnEnable(SPI_CHANNEL1, 1);
}


void InitSPI2Slave()
{
//    printf("Mapping SPI 2 Slave Pins\n");
    
    MapSPI2SlavePins();
    
    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
//    SpiChnSetBrg(SPI_CHANNEL2, baud_rate);
    SpiChnEnable(SPI_CHANNEL2, 1);
}


void InitSPI1(int baud_rate)
{
    MapSPI1MasterPins();
    
    SpiChnConfigure(SPI_CHANNEL1, (SpiConfigFlags)(SPI_CONFIG_MSTEN |  SPI_CONFIG_MSSEN | 
                                                   SPI_CONFIG_MODE8 | SPI_CONFIG_ON |
                                                   0));
    SpiChnSetBrg(SPI_CHANNEL1, baud_rate);
    SpiChnEnable(SPI_CHANNEL1, 1);
}


void InitSPI2(int baud_rate)
{
//    printf("Mapping SPI 2 Master Pins\n");
    MapSPI2MasterPins();
    
    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_MSTEN |  SPI_CONFIG_MSSEN | 
                                                   SPI_CONFIG_MODE8 | SPI_CONFIG_ON |
                                                   0));
    SpiChnSetBrg(SPI_CHANNEL2, baud_rate);
    SpiChnEnable(SPI_CHANNEL2, 1);
}


void SplitFloat2Ints(float fval, int *int_val, int *frac_val)
{
    *int_val = (int) fval;

    *frac_val = (int) ( (float)(fval - (float) *int_val)*1000);
    
//    if(fval >= 0)
//    {
//        *frac_val = (int) ( (float)(fval - (float) *int_val)*1000);
//    }
//    else
//    {
//        *frac_val = (int) ( (float)(-fval + (float) *int_val)*1000);
//    }    
}



uint32_t GetDataBuffer(UART_MODULE id, char *buffer, uint32_t max_size)
{
    uint32_t num_char;

    num_char = 0;

    while(num_char < max_size)
    {
        uint8_t character;

        while(!UARTReceivedDataIsAvailable(id))
            ;
        character = UARTGetDataByte(id);
        if(character == '\r')
            break;
        *buffer = character;
        buffer++;
        num_char++;
    }

    return num_char;
}

void ReadWriteUART2()
{
    int count = 0;
    int data_rx;
    
    int max_buf0_len = 100;
    char buffer0[100];
    int idx = 0;
    
    
//    U2STAbits.URXISEL = 1;
    
//    
//    while(1)
//    {
//        
//        
//        
////        if(DoReadUART2)
//        {
//            int val = ReadUART2();
//            
//            printf("UART2 data %c\n", (char) val);
//        }
//    }
    
//    while(1)
//    {
////        while(DataRdyUART2());
////        while(DataRdyUART2()==0);
//        
//        data_rx = ReadUART2();
////        buffer0[14] = 0;
//            
////        putsUART2(buffer)
////        printf("%-3d:\t received data\t %c", count++, buffer0);
//        printf("%-3d:\t received data\t %c\n", count++, (char)data_rx);
//        
//    }
//    
    
//    UARTStartAutoDataRateDetect(1);
    
    int k = 0;
    char buffer[20];
    while(1)
    {
        
//        while(BusyUART2());
        
        k = 0;
        for(k = 0; k < 10; k++)
        {
    //        while (U2STAbits.UTXBF);
    //        U2TXREG = c;

//            if(UARTDataRateDetected(1) != 0)
            {
    //            int val =  ReadUART2(1);
//                UART_DATA val =  UARTGetData(1);
//                while(BusyUART2());
//                
//                while(DataRdyUART2());

//                while(U2STAbits. == 0);

//                while(U2STAbits.URXDA!=0);
                
                while(U2STAbits.URXDA)
                {
                    data_rx = UARTGetDataByte(1);
                    
//                    getsUART2(9, buffer0, 100);
//                    buffer0[9] = 0;
//                    printf("buffer\t %s\n",buffer0);
                    if(idx < max_buf0_len)
                    {
                        buffer0[idx] = data_rx;
                        idx++;

                    }
                    else{
                        
                        idx = 0;
                        int ix = 0;
                        for(ix = 0; ix < max_buf0_len; ix++)
                        {
                            printf("%-d:\tRead UART2 data %d\n", count++, (char)(buffer0[ix]));
                            
//                            U2TXREG = 'M';
//                            while(U2STAbits.UTXBF);
                        }
                        
                    }
                }
            }
        }
        
//        buffer[k]  = 0;

//        printf("%-d:\tRead UART2 data %c\n", count++, data_rx);
        
//        int ix;
//        for(ix = 0; ix < 1; ix++)
//        {
//            while(BusyUART2());
//
////            WriteUART2('V');
//            U2TXREG = 'M';
//            while(U2STAbits.UTXBF);
//
//        }
        
//        printf("%-d:\tRead UART2 data %c\n", count++, (char) val.data8bit);
//                printf("%-d:\tRead UART2 data %c\n", count++, (char) val);

//        WaitMS(1);

    }
    
    
}

BOOL IsDataWaitingUART2()
{
    return U2STAbits.URXDA != 0;
}

BOOL ReadCommandFromUART2(int *command, int length)
{
    /*
     Command is always 3 bytes of information
     * byte 1 : target ID
     * byte 2: 1-> write 0 Read
     * byte 3: byte to be written
     */
    
    int idx = 0;
    
    int k;
    while(U2STAbits.URXDA && (idx < length))
    {
        command[idx] = UARTGetDataByte(1);
        idx++;
        k = 0; while(k++ < 1000);
    }
    
//    U2STAbits.URXDA = 0;
    
    
    int count  = 0;
    if(command[0] > 0 && (command[1] == 0 || command[1] == 1) && idx >= 3)
    {
        printf("******************************* Sending Acknowledgment ***************************\n");
        
        while(UARTTransmitterIsReady(1) == FALSE);
//        UARTSendDataByte(1 , 0xAA);
//        UARTSendDataByte(1 , 0xAA);
//            WRITE_TO_UART2(0xAA);
//        WRITE_TO_UART2(command[1]);
//        WRITE_TO_UART2(command[2]);
//        
        
//        while(UARTTransmitterIsReady(1) == FALSE);
//        UARTSendDataByte(1, command[0]);
//
//        while(UARTTransmitterIsReady(1) == FALSE);
//        UARTSendDataByte(1, command[1]);
//
//        while(UARTTransmitterIsReady(1) == FALSE);
//        UARTSendDataByte(1, command[2]);
     
        
        return TRUE;
    }
    
    return FALSE;
}



// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}
 
 // Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int istr[100];
    int i = 0;
    
    int xval = x;
    
    if(x < 0)
        x = -x;
    
//    if(x < 0)
//    {
//        str[i++] = '-';
//        x = -x;
//    }
//    
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }
 
    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';
 
    
    if(xval < 0)
    {
        str[i++] = '-';
    }
    
    reverse(str, i);
    str[i] = '\0';
    return i;
}
 
// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;
 
    // Extract floating part
    float fpart;
    
    if(ipart < 0)
        fpart = (-n) - (-ipart);
    else
        fpart = n - ipart;
    
//    if(fpart < 0)
//        fpart = -fpart;
    
 
    // convert integer part to string
    int i = intToStr(ipart, res, 0);
 
    
    int order10 = 1;
    int ix = 0;
    
    while(ix++ < afterpoint)
        order10 *= 10;
    
    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot
 
        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * order10;
 
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}



int SaveFloatDataToFlashMemory(float *data, int length, unsigned int offset /* values */)
{
    unsigned int * base_address = (unsigned int *)BASE_DATA_ADDRESS_NVM + offset;
   
    int idx = 0;
    
    if(length * 2 > (0xFFFF>>1) )
    {
        printf("ERROR: Buffer size must be smaller than 32K\n");
        return 0;
    }
    
    while(idx < length)
    {
        float val = data[idx];
        
        unsigned int ipart = (unsigned int) val;
        unsigned int fpart = 0;
        
        if(val >= 0)
        {
            fpart = (unsigned int) ((val - (int) val) * 1000.0);
        }
        else
        {
            float v2 = -val;
            fpart = (unsigned int) ((v2 - (int) v2) * 1000.0);
        }
        
        NVMWriteWord(base_address + 2*idx,  ipart);
        NVMWriteWord(base_address + (2*idx + 1),  fpart);

        idx++;
    }

    return 1;
}

int SaveInttDataToFlashMemory(unsigned int *data, int length, unsigned int offset /* values */)
{
    unsigned int * base_address = (unsigned int *)BASE_DATA_ADDRESS_NVM + offset;
   
    int idx = 0;
    
    if(length > (0xFFFF>>1) )
    {
        printf("ERROR: Buffer size must be smaller than 32K\n");
        return 0;
    }
    
    while(idx < length)
    {
        unsigned int val = data[idx];

        NVMWriteWord(base_address + idx,  val);

        idx++;
    }
    
    return 1;
}


int SaveIntFloatDataToFlashMemory(IntFloatValue *data, int length, unsigned int offset /* values */)
{
    unsigned int * base_address = (unsigned int *) BASE_DATA_ADDRESS_NVM + offset;
   
    int idx = 0;
    
    if(length > (0xFFFF >> 1) )
    {
        printf("ERROR: Buffer size must be smaller than 32K\n");
        return 0;
    }
    
    while(idx < length)
    {
        unsigned int val = data[idx].ivalue;

        NVMWriteWord(base_address + idx,  val);

        idx++;
    }
    
    return 1;
}


//int SaveIntFloatDataToFlashMemory(IntFloatValue *data, int length, unsigned int offset /* values */)
//{
//    unsigned int * base_address = (unsigned int *) BASE_DATA_ADDRESS_NVM + offset;
//   
//    int idx = 0;
//    
//    if(length > (0xFFFF >> 1) )
//    {
//        printf("ERROR: Buffer size must be smaller than 32K\n");
//        return 0;
//    }
//    
//    while(idx < length)
//    {
//        unsigned int val = data[idx].ivalue;
//
//        NVMWriteWord(base_address + idx,  val);
//
//        idx++;
//    }
//    
//    return 1;
//}

int LoadIntFloatDataFromFlashMemory(IntFloatValue *data, int length, unsigned int offset /* values */)
{
    unsigned int * base_address = (unsigned int *)BASE_DATA_ADDRESS_NVM + offset;
   
    int idx = 0;
    
    if(length > (0xFFFF>>1) )
    {
        printf("ERROR: Buffer size must be smaller than 32K\n");
        return 0;
    }
    
    while(idx < length)
    {
        data[idx].ivalue = *(base_address + idx);
        idx++;
    }
    
    return 1;
}



int LoadFloatDataFromFlashMemory(float *data, int length, unsigned int offset /* values */)
{
    unsigned int * base_address = (unsigned int *)BASE_DATA_ADDRESS_NVM + offset;
   
    int idx = 0;
    
    if(length * 2 > (0xFFFF>>1) )
    {
        printf("ERROR: Buffer size must be smaller than 32K\n");
        return 0;
    }
    
    while(idx < length)
    {
        int ipart = (int)*(base_address + 2*idx);
        int fpart = *(base_address + 2*idx + 1);
        
        if(ipart < 0)
        {
            data[idx] = (float)ipart - (float)fpart/1000.0;
        }
        else
        {
            data[idx] = (float)ipart + (float)fpart/1000.0;
        }
        
        idx++;
    }

    return 1;
}

void FloatToIntSplit(float data, int *int_part, int *float_part)
{
    int ipart = (int) data;
    int fpart = 0;
    
    if(ipart >= 0)
    {
        fpart = (int)((data - (float) ipart) * 1000.0); 
    }
    else
    {
        float d2 = -data;
        fpart = (int)((d2 - (float)((int) d2)) * 1000.0); 
    }
    
    *int_part = ipart;
    *float_part = fpart;
}

void ShowFloatData(float *data, int length)
{
    int i = 0;
    
    while(i < length)
    {
        int ipart;
        int fpart;
        
        FloatToIntSplit(data[i], &ipart, &fpart);
        
        printf("%d + %d/1000.0\n", ipart, fpart);
        i++;
    }
}


void WaitReadUART2()
{
    int val = 0;
    while(1)
    {
        
        if(DoReadUART2)
        {
            DoReadUART2 = FALSE;
            printf("UART2 RX Interrupt is called...\n");
            
        }
//        WAIT_AND_READ_FROM_UART2(val);
        while(U2STAbits.URXDA > 0)
        {
            val = U2RXREG;
        
            printf("Read data %d\n", val);
            WaitMS(1);
        }
    }
}


void __ISR( _UART_2_VECTOR, IPL3AUTO) _UART2RXInterrupt_(void)
{
    
    if(mU2RXGetIntFlag())
    {
    
        DoReadUART2  = TRUE;
    //    int val = U2RXREG;
    //    printf("Data Received...%c\n", (char) val);

//        int val =  U2RXRbits.U2RXR;
//        printf("UART 2 RX interrupt called....%d\n", val);


        mU2RXClearIntFlag();
    }
    else if(mU2TXGetIntFlag())
    {
        mU2TXClearIntFlag();
    }
    else if(mU2EGetIntFlag())
    {
        mU2EClearIntFlag();
    }
}


void *AllocateMaxPossibleMemory(uint32_t *data_length)
{
    void * buffer = NULL;
    
    while(*data_length > 0)
    {
        buffer = malloc((size_t) *data_length);
        
        if(buffer == NULL)
        {
            *data_length -= 1;
        }
        else
        {
            printf("%d KB (%d bytes) memory allocated\n", *data_length/1024, *data_length);
            return buffer;
        }
    }
    
    printf("Failed to allocate memory...\n");

    return NULL;
}


void GetCurrentTime(int *sec, int *ms)
{
    time_t tm;
    
//    tm = time(NULL);
    
    
}


void ReadSPI1Slave_test()
{
//    InitSPI1Slave();
    char str[100] = "Hello World\n";
    
    SpiChnEnable(SPI_CHANNEL1, 0);
//    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_MSSEN |SPI_CONFIG_SSEN|  SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
//    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_SSEN |  SPI_CONFIG_MODE8 | SPI_CONFIG_CKP_HIGH | SPI_CONFIG_CKE_REV | SPI_CONFIG_ON));
//    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_MSSEN |SPI_CONFIG_SSEN |  SPI_CONFIG_MODE8 | SPI_CONFIG_CKE_REV | SPI_CONFIG_CKP_HIGH | SPI_CONFIG_DISSDO | SPI_CONFIG_ON));

//    SpiChnConfigure(SPI_CHANNEL1, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
    SpiChnConfigure(SPI_CHANNEL1, (SpiConfigFlags)(SPI_CONFIG_MSSEN | SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));

//    SpiChnSetBrg(SPI_CHANNEL2, brate);
    SpiChnEnable(SPI_CHANNEL1, 1);

    SPI1CONbits.ON = 1;

    int data = 0;
    int count = 0;
    while(1)
    {
        printf("%d: Reading  \n", count);
//        SpiChnPutC(SPI_CHANNEL1, 0xAA);
        data  = SpiChnGetC(SPI_CHANNEL1);
        
        int k=0; while(k++ < 1000);
        
        SpiChnPutC(SPI_CHANNEL1, 0b10111011); // 0b10111011 = 187
        
        printf("%d: %d\n", count, data);
        count++;
        count %= 256;
    }
    
}



void ReadSPI2Slave_test()
{
    
    SpiChnEnable(SPI_CHANNEL2, 0);
//    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_MSSEN |SPI_CONFIG_SSEN|  SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
//    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_SSEN |  SPI_CONFIG_MODE8 | SPI_CONFIG_CKP_HIGH | SPI_CONFIG_CKE_REV | SPI_CONFIG_ON));
//    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_MSSEN |SPI_CONFIG_SSEN |  SPI_CONFIG_MODE8 | SPI_CONFIG_CKE_REV | SPI_CONFIG_CKP_HIGH | SPI_CONFIG_DISSDO | SPI_CONFIG_ON));

//    SpiChnConfigure(SPI_CHANNEL1, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));

//    SpiChnSetBrg(SPI_CHANNEL2, brate);
    SpiChnEnable(SPI_CHANNEL2, 1);

    SPI2CONbits.ON = 1;

    WaitMS(1);
    
    int data = 0;
    int count = 0;
    while(1)
    {
        printf("%d: Reading...  ", count);
//        SpiChnPutC(SPI_CHANNEL1, 0xAA);
        data  = SpiChnGetC(SPI_CHANNEL2);
        
        int k=0; while(k++ < 1000);
  
        SpiChnPutC(SPI_CHANNEL2, data); // 0b10111011 = 187
//        SpiChnPutC(SPI_CHANNEL2, 0b10111011); // 0b10111011 = 187
        
        printf("rx %d\n", count, data);
        count++;
        count %= 256;
    }
    
}


void TestSPI1_Master()
{
    int data[2];
    
    
    while(1)
    {
        SpiChnPutC(SPI_CHANNEL1, 0);
        data[0] = SpiChnGetC(SPI_CHANNEL1);
        
        SpiChnPutC(SPI_CHANNEL1, 0);
        data[0] = SpiChnGetC(SPI_CHANNEL1);
        
        int32_t  value = ((data[0] << 8) | data[1]) & 0xFFFF;
        int32_t  vi = (value >> 2);
        
        printf("Value obtained %d, [%d, %d]\n", vi, (int)data[0], (int)data[1]);
        WaitMS(1);

    }
}


void Test_SPI2Slave_DataTransfer()
{
        int test_buffer_length = 15480;
        UInt16Value *test_buffer = NULL;
 
        test_buffer = (UInt16Value  *) AllocateMaxPossibleMemory(&test_buffer_length);

        if(test_buffer == NULL)
        {
            printf("ERROR: Cannot allocate memory \n");
            return;
        }
//        else
//        {
//            printf("Allocated memory size %d\n", test_buffer_length);
//        }
 
        int i =0;
        int count  = 0;
        
        for(i = 0; i < test_buffer_length/2; i++)
        {
            int uval = count &0xFF00;
            int lval = count & 0xFF;
            test_buffer[i].v.upper = 1;
            test_buffer[i].v.lower = 5;
            
        }
        
        int data_tx_count = 0;
        
        while(1)
        {        
        SpiChnEnable(SPI_CHANNEL2, 0);
//    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_MSSEN |SPI_CONFIG_SSEN|  SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
//    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_SSEN |  SPI_CONFIG_MODE8 | SPI_CONFIG_CKP_HIGH | SPI_CONFIG_CKE_REV | SPI_CONFIG_ON));
//    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_MSSEN |SPI_CONFIG_SSEN |  SPI_CONFIG_MODE8 | SPI_CONFIG_CKE_REV | SPI_CONFIG_CKP_HIGH | SPI_CONFIG_DISSDO | SPI_CONFIG_ON));

//    SpiChnConfigure(SPI_CHANNEL1, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
        SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));

//    SpiChnSetBrg(SPI_CHANNEL2, brate);
        SpiChnEnable(SPI_CHANNEL2, 1);

        SPI2CONbits.ON = 1;

        WaitMS(1);
        
        int vv;

//            if (test_buffer_length >= test_buffer_length)
//                continue;

            i = 0;

            printf("Data Transfer started\n");
        
            int is_first = 1;
            
            while(i < test_buffer_length/2)
            {
//                SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.upper);
                int val  = SpiChnGetC(SPI_CHANNEL2);
                if(is_first == 1)
                {
                    is_first = 0;
//                    mPORTASetBits(BIT_0 | BIT_1);
                }

                SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.upper);

//                vv = 0; while(vv++ < 100);

                val  = SpiChnGetC(SPI_CHANNEL2);
                SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.lower);
                
                test_buffer_length += 2;
                

//                vv = 0; while(vv++ < 100);

    //            printf("upper %d   lower %d\n", (int)test_buffer[i].v.upper, (int) test_buffer[i].v.lower);
                i++;
            }

            SpiChnClose(SPI_CHANNEL2);
//            mPORTAClearBits(BIT_0 | BIT_1);
            printf("Data Transfer finished\n");
        }
    
        free(test_buffer);
}