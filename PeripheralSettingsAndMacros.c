
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

int32_t     HyperADC_RX32 = 0;


BOOL IsDataReady = FALSE;

uint32_t    CurrentFrequency_T1 = 0;


void _mon_putc(char c)
{
    
// if printf output is set to __XC_UART = 2, UART2

   U2TXREG = c;
   while (U2STAbits.UTXBF);

// if printf output is set to __XC_UART = 1, UART1
   
//   U1TXREG = c;
//   while (U1STAbits.UTXBF);

}


void InitSystem()
{
    SYSTEMConfigPerformance(SYS_FREQ); 
    mOSCSetPBDIV(OSC_PB_DIV_1);     // Peripheral BUS clock is equal to SYS_FREQ, maximum possible
                                        
    INTEnableSystemMultiVectoredInt(); // Must
     
    mBMXDisableDRMWaitState();
    CheKseg0CacheOn();

    InitUART1();
    InitUART2();    // print with UART2
    __XC_UART = 2;
 
  
    InitSPI1(100000L);
    InitSPI2Slave();    
    
    printf("System speed %ld Hz\n", (long) SYS_FREQ);
    printf("Peripheral clock speed %ld Hz\n", (long) GetPeripheralClock());
    
//    SetupDebugGPIOPins();
}


//void InitSystem_Test()
//{
//    SYSTEMConfigPerformance(SYS_FREQ);  // This function sets the PB-Div to 1. Also optimises cache for 72Mhz etc..
//    mOSCSetPBDIV(OSC_PB_DIV_1);           // Therefore, configure the PB bus to run at 1/2 CPU Frequency
//                                                              // you may run at PBclk of 72Mhz if you like too (omit this step)
//                                                              // This will double the PWM frequency.
//    
//    INTEnableSystemMultiVectoredInt();
//
////    BMXDRMSZ = 0x20000;
////    BMXPFMSZ = 0x20000;
//
//    
//    /*    
////    BMXCONbits.BMXARB = 2;
//    
////    BMXDKPBA = 10 * 1024;
//            
////    BMXDUDBA = 0x10000;
////    BMXDUPBA = 0x10000;
//    
//    
////    BMXCONbits.BMXWSDRM;
//    
////    mBMXSetRAMKernProgOffset(0x80000000);
////    mBMXSetFlashUserPartition(0x10000);
//*/
//            
//       
//    mBMXDisableDRMWaitState();
//    CheKseg0CacheOn();
//
//    InitUART1();
//    InitUART2();    // print with UART2
//    __XC_UART = 2;
// 
//  
//    InitSPI1(128);
//    InitSPI2Slave();    
//    
//    
//    
//
//    printf("System speed %ld Hz\n", (long) SYS_FREQ);
//    printf("Peripheral clock speed %ld Hz\n", (long) GetPeripheralClock());
//
//}
//

void SystemReset()
{
    /* The following code illustrates a software Reset */
    // assume interrupts are disabled
    // assume the DMA controller is suspended
    // assume the device is locked
    /* perform a system unlock sequence */
    // starting critical sequence
    SYSKEY = 0x00000000; //write invalid key to force lock
    SYSKEY = 0xAA996655; //write key1 to SYSKEY
    SYSKEY = 0x556699AA; //write key2 to SYSKEY
    // OSCCON is now unlocked
    /* set SWRST bit to arm reset */
    RSWRSTSET = 1;
    /* read RSWRST register to trigger reset */
    unsigned int dummy;
    dummy = RSWRST;
    /* prevent any unwanted code execution until reset occurs*/

    NOP();
    NOP();
    NOP();
    NOP();
    
    while(1) NOP();
    
}


void SetupDebugGPIOPins()
{       
//    uint32_t *data0;
//    uint32_t data_len = 1024 * 64;
//    
//    uint32_t bmx_sz = BMXDRMSZ;
//    uint32_t bmx_pfm_sz = BMXPFMSZ;
//    
////    data0 = (uint32_t *)AllocateMaxPossibleMemory(&data_len);
//    
//    
//    printf("data_len %d\n", data_len);
//
//    printf("BMXDRMSZ %X\n", bmx_sz);
//    printf("BMXPFMSZ %X\n", bmx_pfm_sz);
//    
//
//
//    printf("BMXDUDBA %X\n", BMXDUDBA);
//    printf("BMXDUPBA %X\n", BMXDUPBA);
//
//
//
//    printf("BMXDKPBA %X\n", BMXDKPBA);
    
//    mPORTDSetPinsDigitalOut(BIT_0);
//    mPORTBSetPinsDigitalOut(BIT_14);
//    
//    mPORTGSetPinsDigitalOut(BIT_9);
//    mPORTGSetPinsDigitalOut(BIT_8);
//    
//    
    mPORTASetPinsDigitalOut(BIT_0 | BIT_1);
    mPORTAClearBits(BIT_0 | BIT_1);
//    
//    mPORTDSetPinsDigitalOut(BIT_2);
//    
//    mPORTGClearBits(BIT_8);
//    mPORTGClearBits(BIT_9);
    
//    CNPDGbits.CNPDG8 = 0;
//    CNPDGbits.CNPDG9 = 0;
//    mPORTDClearBits(BIT_0);
    
    printf("Debug pin setup... Disable this in the production version....\n");

}

void RtccSetup()
{
    
    RtccInit();
    RtccSetup();
    
    
}


void WaitMS(unsigned int ms)
{
    unsigned int count = 0;
    unsigned int i = 0;
    
    while(count++ < ms)
    {
        i = 0;
        while(i++ < MS_LOOP)
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
    
    brate = 115200L;
//    brate = 500000L;
    
//    brate = 460800;//921600L/2L;

//    brate = 38400L;
    
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
    
//    mPORTDSetPinsDigitalOut(BIT_11);    // U2TX  J10.15
//    mPORTDSetPinsDigitalIn(BIT_10 );    // U2RX J10.16

    mPORTASetPinsDigitalIn(BIT_14);    // U2RX J10.35
    mPORTASetPinsDigitalOut(BIT_15);    // U2TX  J10.36
    
    PPS_Unlock();

//    U2RXRbits.U2RXR = 0b1010; // 0b0010; // RC1 input J10.18
//    RPC4Rbits.RPC4R = 0b0001; // U2TX  UTX  J10.21

//    U2RXRbits.U2RXR = 0b0011; // 0b0010; // RD10 input J10.16
//    RPD11Rbits.RPD11R = 0b0001; // U2TX  UTX  J10.15

    U2RXRbits.U2RXR = 0b1101; // 0b0010; // RA14 input J10.35
    RPA15Rbits.RPA15R = 0b0001; // U2TX  UTX  J10.36
   
//    U2CTSRbits.U2CTSR = 0b1100; // RC3 .. Input  J10.20
//    RPC2Rbits.RPC2R = 0b0001; // U2RTS 0b0011; RTS output J10.19
    
    PPS_Lock();

    
    unsigned int brate;
    brate = 115200L;//115200L * 1L;
//    brate = 500000L;//115200L * 1L;

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
//    UARTEnable((UART_MODULE)uart_id, (UART_ENABLE_MODE) UART_ENABLE_FLAGS(UART_RX | UART_TX));

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
    mPORTGSetPinsDigitalIn(BIT_7);// SDI2 in J10.24
    
    CNPDGbits.CNPDG7 = 1; // pull down

    mPORTGSetPinsDigitalOut(BIT_8);// SDO2 out
    mPORTGSetPinsDigitalOut(BIT_9);// SS2 out
    CNPDGbits.CNPDG9 = 1; // pull down

}


void MapSPI2SlavePins()
{
    printf("Mapping SPI 2 Slave Pins\n");

    PPS_Unlock();
 
    SDI2Rbits.SDI2R = 1; // RPG7 = 0b0001 // SDI2  SDI2 in J10.24
    RPG8Rbits.RPG8R = 6; //0b0110; // SDO2
    SS2Rbits.SS2R = 1;// RPG9 input
    
    PPS_Lock();
    
    mPORTGSetPinsDigitalIn(BIT_6);// SCK2 in J10.23
    mPORTGSetPinsDigitalIn(BIT_7);// SDI2 in J10.24
//    CNPDGbits.CNPDG7 = 1; // pull down
    mPORTGClearBits(BIT_7);
    mPORTGSetPinsDigitalOut(BIT_8);// SDO2 out
//    CNPDGbits.CNPDG7 = 1;
  
    mPORTGSetPinsDigitalIn(BIT_9);// SS2 in // RG9  // J10.26
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
        OC1RS = (PR2 + 1) * ((float)duty_cycle / 100.0);    //J10.20
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
    PPS_Lock();

    OC1CON = 0x0006;
    
    PR2 = ((int)SYS_FREQ/ (pwm_frequency))-1;
    
    OC1RS = (PR2+1) * ((float)duty_cycle /100.0);
    
    T2CONbits.TCKPS = 0b101;
    
    mT2SetIntPriority(7);  
    mT2ClearIntFlag();     
    mT2IntEnable(1);       

    T2CONbits.ON = 1;
    OC1CONbits.ON = 1;
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


void InitTimer1(uint32_t freq) // freq in Hz
{
    CurrentFrequency_T1 = freq;
    
    T1CONbits.ON = 0;
    
    PR1 = (uint32_t)SYS_FREQ/ ((uint32_t)((2*freq)*( (uint32_t)128)) )-1;
    
    T1CONbits.TCKPS = 0b11; // 256
    
    mT1SetIntPriority(3);  // this timer is a low priority event
    mT1ClearIntFlag();     
    mT1IntEnable(1);       

    T1CONbits.ON = 1;

    printf("Timer-1 set to %d Hz\n", freq);
}

uint32_t GetTimer1_Freqency()
{
    return CurrentFrequency_T1;
}

void InitTimer4(uint32_t freq) // freq in Hz
{
    T4CONbits.ON = 0;
    T4CONbits.T32 = 1;
    
    PR4 = (uint32_t)SYS_FREQ/ ((uint32_t)((1*freq)*( (uint32_t)512)) )-1;

//    PR4 = 2; 
    T4CONbits.TCKPS = 0b111; // 256
    
    
    mT45SetIntPriority(3);  // this timer is a low priority event
    mT45ClearIntFlag();     
    mT45IntEnable(1);       

    T4CONbits.ON = 1;

    printf("Timer-4 set to %d Hz\n", freq);
}


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
    
    
    SpiChnEnable(SPI_CHANNEL1, 0);
    SpiChnConfigure(SPI_CHANNEL1, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
    SpiChnEnable(SPI_CHANNEL1, 1);

    SPI1CONbits.ON = 1;
}


void InitSPI2Slave()
{
//    printf("Mapping SPI 2 Slave Pins\n");
    
    MapSPI2SlavePins();
    
    
    SpiChnEnable(SPI_CHANNEL2, 0);
    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
    SpiChnEnable(SPI_CHANNEL2, 1);

    SPI2CONbits.ON = 1;

}


void InitSPI1(int freq_hz)
{
    int baud_rate = SPI_BRG_VAL(freq_hz);
    
    MapSPI1MasterPins();
    
    SPI1CONbits.ON = 0;
    
    SpiChnConfigure(SPI_CHANNEL1, (SpiConfigFlags)(SPI_CONFIG_MSTEN | SPI_CONFIG_CKE_REV |  
                                                   SPI_CONFIG_MODE8 | SPI_CONFIG_ON
                                                   ));
    SpiChnSetBrg(SPI_CHANNEL1, baud_rate);
    
    
    SpiChnEnable(SPI_CHANNEL1, 1);
    
    SPI1CONbits.ON = 1;

}


void InitSPI2(int freq_hz)
{
    int baud_rate = SPI_BRG_VAL(freq_hz);

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

    *frac_val = (int) ( (float)(fval - (float) *int_val)*1000.0);
    
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

inline BOOL IsDataWaitingUART2()
{
    return U2STAbits.URXDA != 0;
}

inline BOOL IsDataWaitingUART1()
{
    return U1STAbits.URXDA != 0;
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
        int val = (int) UARTGetDataByte(1);
        command[idx] = (char) val;
        idx++;
        k = 0; while(k++ < 1000);
        
        printf("Command received %d\n", val);
    }
    
//    U2STAbits.URXDA = 0;
    
    
    int count  = 0;
    if(command[0] > 0 && (command[1] == 0 || command[1] == 1) && idx >= 3)
    {
//        printf("******************************* Sending Acknowledgment ***************************\n");
        
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


BOOL ReadCommandFromUART1(int *command, int length)
{
    /*
     Command is always 3 bytes of information
     * byte 1 : target ID
     * byte 2: 1-> write 0 Read
     * byte 3: byte to be written
     */
    
    int idx = 0;
    
    int k;
    
    if(!U1STAbits.URXDA)
    {
        return FALSE;
    }
    
    while(U1STAbits.URXDA && (idx < length))
    {
        int val = (int) UARTGetDataByte(0);
        command[idx] = (char) val;
        idx++;
        k = 0; while(k++ < 1000);
        
//        printf("Command received %d\n", val);
    }
    
//    U2STAbits.URXDA = 0;
    
    
    int count  = 0;
    if(command[0] > 0 && (command[1] == 0 || command[1] == 1) && idx >= 3)
    {
//        printf("******************************* Sending Acknowledgment ***************************\n");
        
        while(UARTTransmitterIsReady(0) == FALSE);
        return TRUE;
    }
    
    return FALSE;
}

BOOL ReadCommandFromSPI2Slave(int *command, int length)
{
    /*
     Command is always 3 bytes of information
     * byte 1 : target ID
     * byte 2: 1-> write 0 Read
     * byte 3: byte to be written
     */
    
    int idx = 0;
    int dummy = 0;
    
    int k;
    if(SPI2STATbits.SPIRBF)
    {
        while(SPI2STATbits.SPIRBF && (idx < length))
        {
            printf("Reading spi command....\n");
            
            int val = SPI2_SLAVE_READ();
            SPI2_SLAVE_WRITE(val, dummy);
            
            command[idx] = (char) val;
            idx++;
            k = 0; while(k++ < 1000);

            printf("Commane received %d\n", val);
        }
    }
    
//    U2STAbits.URXDA = 0;
    
    
    int count  = 0;
    if(command[0] > 0 && (command[1] == 0 || command[1] == 1) && idx >= 3)
    {
//        printf("******************************* Sending Acknowledgment ***************************\n");
        
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
//        if(SPI2STATbits.SPIRBF == 0)
//        {
////            printf("Buffer is empty...\n  ");
//            continue;
//        }
        
        printf("%d: Reading...  ", count);
//        SpiChnPutC(SPI_CHANNEL1, 0xAA);
        data  = SpiChnGetC(SPI_CHANNEL2);
        SpiChnPutC(SPI_CHANNEL2, data); // 0b10111011 = 187
//        SpiChnPutC(SPI_CHANNEL2, 0b10111011); // 0b10111011 = 187

        int dummy = SPI2BUF;
        
        if(data == 0xFF)
        {
            break;
        }
        
        printf("rx %d\n", count, data);
        count++;
        count %= 256;
        WaitMS(1);

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
        int test_buffer_length = 100;//15480;
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
            test_buffer[i].v.upper = i+1;
            test_buffer[i].v.lower = i+1+10;
            
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

//            SpiChnPutC(SPI_CHANNEL2, test_buffer[0].v.upper);
//            int tv = SPI2BUF; 
            
            while(i < test_buffer_length/2)
            {
                
                int val  = SpiChnGetC(SPI_CHANNEL2);
                
                if(is_first == 1)
                {
                    is_first = 0;
//                    mPORTASetBits(BIT_0 | BIT_1);
                }
                SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.upper);
                int tmp = SPI2BUF; // clear SPI2READ buffer
 
//                vv = 0; while(vv++ < 100);
 
                val  = SpiChnGetC(SPI_CHANNEL2);
                SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.lower);
                tmp = SPI2BUF;
               
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


void Test_SPI2Slave_DataTransferWithUART2()
{
        int test_buffer_length = 100;//15480;
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
 
        
//        SpiChnEnable(SPI_CHANNEL2, 0);
//        SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
//        SpiChnEnable(SPI_CHANNEL2, 1);
//        
        SPI2CONbits.ON = 1;

//        WaitMS(1);
        
        
//        TestUART2();
        
        printf("Waiting for UART command....,\n");
        
        char command[10];
        int command_length = 3;
        
        while(1)
        {
            int didx = 0;
            if(U2STAbits.URXDA)
            {
                while(U2STAbits.URXDA && didx < 10)
                {
                    command[didx] = UARTGetDataByte(1);
                    didx++;
                }
        
                printf("Main command received.... %d, %d, %d\n", (int)command[0], (int)command[1], (int)command[2] );
                
            }
            
            if(command[0] == 99)
            {
  
                command[0] = 0;
                command[1] = 0;
                command[2] = 0;

//                printf("command received.... %d, %d, %d\n", (int)command[0], (int)command[1], (int)command[2] );
                
                
//                if(command[0] != 99)
//                {
//                    continue;
//                }

                
//                printf("command received.... %d, %d, %d\n", (int)command[0], (int)command[1], (int)command[2] );
                int i =0;
                int count  = 0;

                for(i = 0; i < test_buffer_length/2; i++)
                {
                    int uval = count &0xFF00;
                    int lval = count & 0xFF;
                    test_buffer[i].v.upper = i;
                    test_buffer[i].v.lower = i+10;

                }

                UInt16Value dlen;

                dlen._value = test_buffer_length;
                
                printf("Buffer length to send %d\n", test_buffer_length);
                

                WRITE_TO_UART2(dlen.v.upper);
                WRITE_TO_UART2(dlen.v.lower);
                
                
                printf("Two bytes sent upper === %d,     lower === %d\n", dlen.v.upper, dlen.v.lower);
                
//                continue;
                
                int data_tx_count = 0;

                int vv;

        //            if (test_buffer_length >= test_buffer_length)
        //                continue;

                i = 0;

                printf("Data Transfer started\n");

                int is_first = 1;

                while(i < test_buffer_length/2)
                {
    //                SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.upper);
                    int val1  = SpiChnGetC(SPI_CHANNEL2);
                    
                    if(val1 == 0xFF)
                        break;
                    

                    SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.upper);
                    int tm = SPI2BUF;

    //                vv = 0; while(vv++ < 100);

                    int val2  = SpiChnGetC(SPI_CHANNEL2);
                    SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.lower);
                    tm = SPI2BUF;
                    
                    if(val2 == 0xFF)
                        break;

                    
                    test_buffer_length += 2;


    //                vv = 0; while(vv++ < 100);

        //            printf("upper %d   lower %d\n", (int)test_buffer[i].v.upper, (int) test_buffer[i].v.lower);
//                    printf('i == %d\n',i);
                    i++;
                }

                printf("Data Transfer finished\n");

            }
        }

//        SpiChnClose(SPI_CHANNEL2);
//            mPORTAClearBits(BIT_0 | BIT_1);
        
        free(test_buffer);
}


void Test_SPI2Slave_DataTransferWithUART1()
{
        int test_buffer_length = 200;//15480;
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
 
        
        SpiChnEnable(SPI_CHANNEL2, 0);
        SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
        SpiChnEnable(SPI_CHANNEL2, 1);
        
        SPI2CONbits.ON = 1;

//        WaitMS(1);
        
        
//        TestUART2();
        
        
        char command[10];
        int command_length = 3;
        
        while(1)
        {
            int didx = 0;
    
            printf("Waiting for UART command....,\n");
            
            while(!U1STAbits.URXDA);
            
            if(U1STAbits.URXDA)
            {
                while(U1STAbits.URXDA && didx < 10)
                {
                    command[didx] = UARTGetDataByte(0);
                    didx++;
                }
        
                printf("Main command received.... %d, %d, %d\n", (int)command[0], (int)command[1], (int)command[2] );
                
            }
            
            if(command[0] == 99)
            {
                printf("Received command to send data by SPI2\n");
  
                command[0] = 0;
                command[1] = 0;
                command[2] = 0;
                command[3] = 0;

//                printf("command received.... %d, %d, %d\n", (int)command[0], (int)command[1], (int)command[2] );
                
                
//                if(command[0] != 99)
//                {
//                    continue;
//                }
//                printf("command received.... %d, %d, %d\n", (int)command[0], (int)command[1], (int)command[2] );
                int i =0;
                int count  = 0;

                for(i = 0; i < test_buffer_length/2; i++)
                {
                    int uval = count &0xFF00;
                    int lval = count & 0xFF;
                    test_buffer[i].v.upper = i+1;
                    test_buffer[i].v.lower = i+1+10;
                }

                UInt16Value dlen;

                dlen._value = test_buffer_length;
                
                printf("Buffer length to send %d\n", test_buffer_length);
                
                WRITE_TO_UART1(dlen.v.upper);
                WRITE_TO_UART1(dlen.v.lower);
                
                
                printf("Two bytes sent upper === %d,     lower === %d\n", (int)dlen.v.upper, (int)dlen.v.lower);
                int data_tx_count = 0;

                int vv;

        //            if (test_buffer_length >= test_buffer_length)
        //                continue;
                i = 0;

                printf("Data Transfer started\n");
                
                SendDataBySPI2Slave(test_buffer, test_buffer_length);
//                SendDataBySPI2SlaveWithTimeOut(test_buffer, test_buffer_length, (1<<10));
//
//                int is_first = 1;
//                int dummy;
//
//                while(i < test_buffer_length/2)
//                {
//    //                SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.upper);
//                    int val1  = SpiChnGetC(SPI_CHANNEL2);
//
//                    SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.upper);
//                    dummy = SPI2BUF;
//
//                    if(val1 == 0xFF)
//                        break;
//
//                    int val2  = SpiChnGetC(SPI_CHANNEL2);
//                    SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.lower);
//                    dummy = SPI2BUF;
//                    
//                    if(val2 == 0xFF)
//                        break;
//
//        //            printf("upper %d   lower %d\n", (int)test_buffer[i].v.upper, (int) test_buffer[i].v.lower);
////                    printf('i == %d\n',i);
//                    i++;
//                }
//                
//                while(SPI2STATbits.SPIRBF)
//                    dummy = SPI2BUF;

                command[0] = 0;
                command[1] = 0;
                command[2] = 0;
                command[3] = 0;
                
                printf("Data Transfer finished\n");

            }
        }

//        SpiChnClose(SPI_CHANNEL2);
//            mPORTAClearBits(BIT_0 | BIT_1);
        
        free(test_buffer);
}





void Test_SPI2Slave_DataTransferWith_SPI2Command()
{
        int test_buffer_length = 100;//15480;
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
 
        
        SpiChnEnable(SPI_CHANNEL2, 0);
        SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
        SpiChnEnable(SPI_CHANNEL2, 1);
        
        SPI2CONbits.ON = 1;

//        WaitMS(1);
//        TestUART2();
        
        printf("Waiting for UART command....,\n");
        
        char command[10];
        int command_length = 3;
        
        while(1)
        {
            int didx = 0;
            if(!SPI2STATbits.SPIRBE)
            {
                while(U2STAbits.URXDA && didx < 10)
                {
                    command[didx] = UARTGetDataByte(1);
                    didx++;
                }
        
                printf("Main command received.... %d, %d, %d\n", (int)command[0], (int)command[1], (int)command[2] );
                
            }
            
            if(command[0] == 99)
            {
                int i =0;
                int count  = 0;

                for(i = 0; i < test_buffer_length/2; i++)
                {
                    int uval = count &0xFF00;
                    int lval = count & 0xFF;
                    test_buffer[i].v.upper = i;
                    test_buffer[i].v.lower = i+10;

                }

                UInt16Value dlen;

                dlen._value = test_buffer_length;
                
                printf("Buffer length to send %d\n", test_buffer_length);
                

                WRITE_TO_UART2(dlen.v.upper);
                WRITE_TO_UART2(dlen.v.lower);
                
                
                printf("Two bytes sent upper === %d,     lower === %d\n", dlen.v.upper, dlen.v.lower);
                
//                continue;
                
                int data_tx_count = 0;

                int vv;

        //            if (test_buffer_length >= test_buffer_length)
        //                continue;

                i = 0;

                printf("Data Transfer started\n");

                int is_first = 1;

                while(i < test_buffer_length/2)
                {
    //                SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.upper);
                    int val1  = SpiChnGetC(SPI_CHANNEL2);
                    
                    if(val1 == 0xFF)
                        break;
                    

                    SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.upper);
                    int tm = SPI2BUF;

    //                vv = 0; while(vv++ < 100);

                    int val2  = SpiChnGetC(SPI_CHANNEL2);
                    SpiChnPutC(SPI_CHANNEL2, test_buffer[i].v.lower);
                    tm = SPI2BUF;
                    
                    if(val2 == 0xFF)
                        break;
                    
                    test_buffer_length += 2;

//                  vv = 0; while(vv++ < 100);
//                  printf("upper %d   lower %d\n", (int)test_buffer[i].v.upper, (int) test_buffer[i].v.lower);
//                  printf('i == %d\n',i);
                    i++;
                }

                printf("Data Transfer finished\n");

            }
        }

        SpiChnClose(SPI_CHANNEL2);
        free(test_buffer);
}


int SendDataBySPI2Slave(UInt16Value *data, int data_length_in_bytes)
{
    int i = 0;
    int word_len = data_length_in_bytes >> 1;
    
    int val, dummy;

    printf("Sending %d bytes of data\n", data_length_in_bytes);
    
    for(i = 0; i < word_len; i++)
    {
        val = SpiChnGetC(SPI_CHANNEL2);
        SpiChnPutC(SPI_CHANNEL2, (unsigned int)data[i].v.upper);
        dummy = SPI2BUF;
        
        if(val == 0xFF)
        {
            break;
        }
        
        val = SpiChnGetC(SPI_CHANNEL2);
        SpiChnPutC(SPI_CHANNEL2, (unsigned int) data[i].v.lower);
        dummy = SPI2BUF;
        
        if(val == 0xFF)
        {
            break;
        }
    }

    printf("***  %d bytes SENT  ***\n", (i << 1));

    while(SPI2STATbits.SPIRBF) // clean up read buffer
    {
        dummy = SPI2BUF;
    }
    
    // it seems to help...  not sure why is it even necessary
    SPI2CONbits.ON = 0;
    SPI2CONbits.ON = 1;
    SPI2CONbits.ON = 1;
    
    return (i << 1);
}


inline BOOL SPI2SlaveReadWithTimeOut(int *data, int time_out_loop)
{
    int wait_count = 0;
    while(!SPI2STATbits.SPITBF)
    {
        if(wait_count++ > time_out_loop)
        {
            return FALSE; // timeout
        }
    }
    
    *data = SPI2BUF;
    return TRUE; // Normal read
}


int SendDataBySPI2SlaveWithTimeOut(UInt16Value *data, int data_length_in_bytes, int time_out_delay)
{
    int i = 0;
    int word_len = data_length_in_bytes >> 1;
    
    int val, dummy;

    printf("Sending %d bytes of data\n", data_length_in_bytes);

    int wait_count = 0;
    BOOL do_stop_waiting = FALSE;
    
//    int time_out_delay = 10000;
    
    for(i = 0; i < word_len; i++)
    {
        if(SPI2SlaveReadWithTimeOut(&val, time_out_delay) == FALSE)
        {
            break;
        }

        SPI2BUF = data[i].v.upper;
        while(SPI2STATbits.SPITBF);
        dummy = SPI2BUF;
        
        if(val == 0xFF)
        {
            break;
        }
        
        if(SPI2SlaveReadWithTimeOut(&val, time_out_delay) == FALSE)
        {
            break;
        }

        SPI2BUF = data[i].v.lower;
        while(SPI2STATbits.SPITBF);
        dummy = SPI2BUF;
        
        if(val == 0xFF)
        {
            break;
        }
    }

    printf("***  %d bytes SENT  ***\n", i*2);

    while(SPI2STATbits.SPIRBF) // clean up read buffer
    {
        dummy = SPI2BUF;
    }
    
    
    // it seems to help... flushes all the states,... not sure why is it even necessary
    SPI2CONbits.ON = 0;
//    WaitMS(10);
    SPI2CONbits.ON = 1;
    
    return i*2;
}



void TestUART2()
{
    int count = 0;
    int tx = 50;

    printf("Reading UART2...\n");
//    InitUART2();
    
    while(1)
    {
        if(U2STAbits.URXDA)
        {
            int val = (int) UARTGetDataByte(1); // UART2
            
//            printf("%d:   UART2 read value %d\n", count++, val);

//            printf("%d:   Writing to UART2 %d\n", (int) tx);
            
            WRITE_TO_UART2(tx);
            
            tx += 15;
            tx %= 256;
        }
    }
}



void TestUART1DataReadWrite()
{
   int count = 0;
    int tx = 10;

    while(1)
    {
//        printf("%d:\tThis is a test...\n", count++);
//        
//        int k = 0; while(k++ < 1000)
//        {
//            asm("nop");
//        }
        
        
        if(U1STAbits.URXDA)
        {
            int val = (int) UARTGetDataByte(0); // UART2
            
//            printf("%d:   Writing to UART1 %d\n", (int) tx);
            
            tx = val;
            WRITE_TO_UART1(tx);
            printf("%d:   UART1 read value %d \t Writing to UART1 %d\n", count++, val, (int)tx);
            
            tx += 15;
            tx %= 256;
            
            int kk = 0; while(kk++ < (1<< 20));
        }

        
    }
}



void TestSPi2Slave()
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
    
        int i = 0;
        int tmp = 0;
        int data_length = 100;
   
        
        while(1)
        {
//            SPI2CONbits.ON = 1;
            printf("Waiting to send %d bytes of data\n", data_length);
            for(i = 0; i < data_length/2; i++)
            {
                int val  = SpiChnGetC(SPI_CHANNEL2);
                SpiChnPutC(SPI_CHANNEL2, i+1);
                tmp = SPI2BUF;
                
                if(val == 0xFF)
                {
                    printf("val == %d, Exiting SPI2 read loop\n", val);
                    break;
                }
                
                int v = 0; while(v++ < 10);
                
                val  = SpiChnGetC(SPI_CHANNEL2);
                SpiChnPutC(SPI_CHANNEL2, i+1+10);
                tmp = SPI2BUF;
                
                if(val  == 0xFF)
                {
                    printf("2nd level val == %d, Exiting SPI2 read loop\n", val);
                    break;
                }

                
//                printf(" i = %d\n", i);
            }
            
//            printf("%i == d\n", i);
            
            printf("Data transfered\n");
            SPI2CONbits.ON = 0;
            WaitMS(1);
            SPI2CONbits.ON = 1;
        }

        SpiChnClose(SPI_CHANNEL2);
}

inline void SPI2CleanReadBuffer()
{
    int dummy;
    
    while(SPI2STATbits.SPIRBF)
    {
        dummy = SPI2BUF;
    }
    
    while(SPI2STATbits.SPITBF)
    {
        NOP();
    }
//    {
//        dummy = SPI2BUF;
//    }
    
    
//    SPI2CONbits.ON = 0;
//    SPI2CONbits.ON = 1;
}


void TestSPi2Slave_WithSendData()
{
    
    int             buffer_length = 138;
    UInt16Value     buffer[buffer_length/2];
    UInt16Value     tx_data_len;
    
    
    char * command[10];
    int    c_idx = 0;


    SpiChnEnable(SPI_CHANNEL2, 0);
    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
    SpiChnEnable(SPI_CHANNEL2, 1);

    SPI2CONbits.ON = 1;

    printf("Waiting for UART command....\n");
//    
//    while(ReadCommandFromUART2(command, 3)==0)
//    {
//        if(command[0] == 99)
//        {
//            tx_data_len._value = buffer_length;
//            
//            WRITE_TO_UART2(tx_data_len.v.upper);
//            WRITE_TO_UART2(tx_data_len.v.lower);
//            WaitMS(2);
//            break;
//        }
//        
//        command[0] =0;
//        command[1] =0;
//        command[2] =0;
//        
//    }
    
    
    int k =0;
    for(k = 0; k < buffer_length/2; k++)
    {
        int v = k % 90;
        buffer[k].v.upper = v+1;
        buffer[k].v.lower = v+1+10;

//            buffer[k]._value = (uint16_t) ( ( (float)v + 0.25 * (1 % 5)) * 100.0 );
    }


    int count = 0;

    while(1)
    {
//            SPI2CONbits.ON = 1;
        printf("%d: Waiting to send %d bytes of data\n", count, buffer_length);

        int tx_len = SendDataBySPI2Slave(buffer, buffer_length);
//            printf("%i == d\n", i);

        printf("%d: Data transfered %d bytes\n", count, tx_len);
        count++;
    }

    SpiChnClose(SPI_CHANNEL2);
}


BOOL ReadCommandFromSPI2(int *command, int length)
{
    int idx = 0;
    
    while(SPI2STATbits.SPIRBF && idx < length)
    {
//        command[]
        
    }
    
    return FALSE;
}


void TestSPi2Slave_WithSendData_WithSPI2Command()
{
    
    int             buffer_length = 138;
    UInt16Value     buffer[buffer_length/2];
    UInt16Value     tx_data_len;
    
    
    char * command[10];
    int    c_idx = 0;


    SpiChnEnable(SPI_CHANNEL2, 0);
    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
    SpiChnEnable(SPI_CHANNEL2, 1);

    SPI2CONbits.ON = 1;

    printf("Waiting for command in SPI2 ....\n");
//    
//    while(ReadCommandFromUART2(command, 3)==0)
//    {
//        if(command[0] == 99)
//        {
//            tx_data_len._value = buffer_length;
//            
//            WRITE_TO_UART2(tx_data_len.v.upper);
//            WRITE_TO_UART2(tx_data_len.v.lower);
//            WaitMS(2);
//            break;
//        }
//        
//        command[0] =0;
//        command[1] =0;
//        command[2] =0;
//        
//    }
    
    
    int k =0;
    for(k = 0; k < buffer_length/2; k++)
    {
        int v = k % 90;
        buffer[k].v.upper = v+1;
        buffer[k].v.lower = v+1+10;

//            buffer[k]._value = (uint16_t) ( ( (float)v + 0.25 * (1 % 5)) * 100.0 );
    }

    int dummy;
    
    char len_data[3];
    
    while(1)
    {
        c_idx = 0;
        command[0] = 0;
        command[1] = 0;
        command[2] = 0;
        command[3] = 0;
        
        tx_data_len._value = buffer_length;
        len_data[0] = 1;
        len_data[1] = 2;//tx_data_len.v.upper;
        len_data[2] = 3;//tx_data_len.v.lower;
        
        if(!SPI2STATbits.SPIRBF)
            continue;

//        printf
        
        while(SPI2STATbits.SPIRBF && c_idx < 3)
        {
            int cdata = SPI2_SLAVE_READ();
//            SpiChnPutC(SPI_CHANNEL2, (unsigned int)len_data[c_idx]);
            SpiChnPutC(SPI_CHANNEL2, cdata);
//            SPI2_SLAVE_WRITE((unsigned int)len_data[c_idx], dummy);
            command[c_idx] = cdata;
            c_idx++;
        }
        
        SPI2CleanReadBuffer();
        
        int k =0;
        for(k =0 ; k < c_idx; k++)
        {
            printf("command %d\n", (int)command[k]);
            WaitMS(10);
        }
        
//        WaitMS(100);
        
        if(command[0] == 99 )
        {
            printf("Waiting to send %d bytes of data\n", buffer_length);

            int tx_len = SendDataBySPI2Slave(buffer, buffer_length);

            printf("Data transfered %d bytes\n", tx_len);
        }


    }

    SpiChnClose(SPI_CHANNEL2);
}


void TestSpi2CReadCommand()
{
    int max_command_len = 3;
    int idx =0;
    int dummy;
    
    int command[10];
    
    SpiChnEnable(SPI_CHANNEL2, 0);
    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
    SpiChnEnable(SPI_CHANNEL2, 1);

    SPI2CONbits.ON = 1;

    int flag = 0;
    int buffer[300];
    
    idx = 0;
    
    
    
    int tx_val[10];
    
    tx_val[0] = 1;
    tx_val[1] = 2;
    tx_val[2] = 3;
    tx_val[3] = 4;
    tx_val[4] = 5;
    
    
    
    int i = 1;
    while(1)
    {
//        if(idx > 4)
//            idx = 0;
        
        if(!SPI2STATbits.SPIRBF)
            continue;

        while(idx < 5)
        {
            while(SPI2STATbits.SPIRBF)
            {
                int rx0 = SpiChnGetC(SPI_CHANNEL2);
                SpiChnPutC(SPI_CHANNEL2, tx_val[idx]);
                int dummy = SPI2BUF;
                buffer[idx] = rx0;
                
                if(rx0 == 0xFF)
                {
                    idx = 10;
                    SPI2CleanReadBuffer();
                    
                    break;
                }
                idx++;

            }
        }
        
        idx =0;
        
        int j =0;
        
        
//        SPI2CONbits.ON = 0;
////        WaitMS(100);
//        
//        while(j < 5)
//        {
//            printf("idx == %d  %d\n", j,  buffer[j]);
//            WaitMS(1);
//            j++;
//        }

         SPI2CleanReadBuffer();
       
 //
//        SPI2CONbits.ON = 1;
        
//        return
//        idx = 0;
    //        printf("%d\n", rx0);
    }
    
    while(1)
    {
//        if(SPI2STATbits.SPIRBF)
//            printf("SPI2STATbits.SPITBF is TRUE\t");
//        else
//            printf("SPI2STATbits.SPITBF is FALSE\t");
        
//        if(!SPI2STATbits.SPIRBE)
//            printf("SPI2STATbits.SPIRBE is Empty\t");
//        else
//            printf("SPI2STATbits.SPIRBE is NOT empty\t");
//
//        WaitMS(1);

        int data = SPI2_SLAVE_READ();
//        SpiChnPutC(SPI_CHANNEL2,data);
        int d2 = data+5;
        SPI2_SLAVE_WRITE(5, dummy);

        printf(" %d \n ", data);
        
//        buffer[idx] = data;
        
//        if(idx > 4)
//        {
//            int k = 0;
//            while(k < idx)
//            {
//                printf(" %d \n ", buffer[k]);
//                
//            }
//            idx = -1;
//         }
//        
//        idx++;
//        
//        if(idx > 50)
            break;
//        printf(" %d \n ", data);
        //WaitMS(1);
        
    }
    
    int j = 0;
    while(j < idx)
    {
        printf(" %d   \n", buffer[j]);
        j++;
    }
    
    return ;
    
    while(1)
    {
        idx = 0;
        
        if(1)
        {
            while(idx < max_command_len)
            {
                command[idx] = SPI2_SLAVE_READ();
                SPI2_SLAVE_WRITE(0, dummy);
                idx++;
                
                SPI2CONbits.ON = 0;
                SPI2CONbits.ON = 1;
                
            }
    
            if(idx == 3)
            {
                int k = 0;
                while(k++ < max_command_len)
                {
                    printf("  %d  ", command[k]);
                }
                printf("\n");
            }

        }
    }
    
    
}



void TestSPI1Master_To_SPI2SlaveData_Transfer(uint32_t spi_speed)
{
    int dummy = 0;

    int tx_rx_data_len = 1000;
    int tx1_len = tx_rx_data_len;
    int rx1_len = tx_rx_data_len;
    int tx2_len = tx_rx_data_len;
    int rx2_len = tx_rx_data_len;

    int tx1_buffer[tx1_len];
    int rx1_buffer[rx1_len];
    int tx2_buffer[tx2_len];
    int rx2_buffer[rx2_len];
    
    
//    InitSPI1(brate);
//    InitSPI2Slave();
    
    uint32_t brate = SpiBrgVal(SYS_FREQ, spi_speed);
    
    SpiChnEnable(SPI_CHANNEL1, 0);
    SpiChnConfigure(SPI_CHANNEL1, (SpiConfigFlags)(SPI_CONFIG_MSTEN |  SPI_CONFIG_FRMEN |
                                                   /*SPI_CONFIG_CKE_REV*/ 0 | SPI_CONFIG_MODE8 | SPI_CONFIG_ON
                                                   ));
    SpiChnSetBrg(SPI_CHANNEL1, brate);
    SpiChnEnable(SPI_CHANNEL1, 1);
    
    
    
    
    
    SpiChnEnable(SPI_CHANNEL2, 0);
    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_FRMEN  |
                                                    /*SPI_CONFIG_CKE_REV*/ 0 | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
    SpiChnEnable(SPI_CHANNEL2, 1);

    SPI2CONbits.ON = 1;
    
    int tx1 = 0;
    int tx2;
    
    printf("Starting SPI1 -> SPI2 transver\n");
    
    int idx = 0;
    while(1)
    {
        idx = 0;
        while(idx < tx_rx_data_len)
        {
             tx2 = tx1 + 10;

             SpiChnPutC(SPI_CHANNEL1, tx1);

             int rx2 = SpiChnGetC(SPI_CHANNEL2);
             SpiChnPutC(SPI_CHANNEL2, tx2);
             dummy = SPI2BUF;

             int rx1 = 0;
             
//             rx1 = 0; //SpiChnGetC(SPI_CHANNEL1);
             rx1 = SpiChnGetC(SPI_CHANNEL1);
             
             tx1_buffer[idx] = tx1;
             rx1_buffer[idx] = rx1;

             tx2_buffer[idx] = tx2;
             rx2_buffer[idx] = rx2;

             tx1 = (tx1 + 1) % 200;

             idx++;

        }
        
        idx = 0;
        
        while(idx++ < tx_rx_data_len)
        {
            printf("tx1   %3d\trx2   %3d\ttx2   %3d\trx1   %3d\n", 
                                        tx1_buffer[idx],
                                        rx2_buffer[idx],
                                        tx2_buffer[idx],
                                        rx1_buffer[idx]);

            tx1_buffer[idx] = 0;
            rx1_buffer[idx] = 0;

            tx2_buffer[idx] = 0;
            rx2_buffer[idx] = 0;

            int k = 0; while(k++ < 1000);
        }
       
    }
}


void InitDMA_DMA_Spi2Slave()
{
    
    
    
    
}



void InitExtINTs()
{
    UnlockPPS();
    
    INT1Rbits.INT1R = 0b0010;   // INT1 input RB14, J10.59
    
    LockPPS();

    mPORTBSetPinsDigitalIn(BIT_14); // J11.34 INT1

    ConfigINT1(EXT_INT_ENABLE | RISING_EDGE_INT | EXT_INT_PRI_7);
    SetSubPriorityINT1(EXT_INT_SUB_PRI_0);
}


#define         HYPER_ADC_CS(val)       {LATDbits.LATD9  = val;}

void InitHyperADCPins()
{
    InitExtINTs();

//    mPORTASetPinsDigitalOut(BIT_3 | BIT_14 | BIT_15);

    mPORTDSetPinsDigitalOut(BIT_11 | BIT_8 | BIT_9); // J11... 24 25 26

    mPORTBSetPinsDigitalIn(BIT_5); // J11. 29
    mPORTBClearBits(BIT_5); // J11. 29   // SCKL SEL  = 0;
    
    mPORTDSetBits(BIT_8);  // J11.25   MultiPin High
    
    mPORTDSetBits(BIT_9); // J11.26   CS 
    

    mPORTDSetBits(BIT_11); // J11.24  Not used

    
//    mPORTBSetPinsDigitalOut(BIT_0);
    
    
//    mPORTDSetBits(BIT_10| BIT_11 | BIT_8 | BIT_9); // J11: 24 25 26
//    
//    
//    mPORTDClearBits(BIT_10); // SCLK SEL, J11: 23
//
//    mPORTBClearBits(BIT_5);
    
//    mPORTAClearBits(BIT_15); // J11.37  SCLK SEL to internal
//    
//    mPORTASetBits(BIT_14); // J11. 35for Drate0/ Fpath..... the 1 bit settings
//    
//    mPORTASetBits(BIT_3); // J11.36 SPI CS to internal
    
}

//#define HYPER_ADC_PIN_INIT()       



void TestHyperADC_SPI2Slave_Read()
{
    
    int data_length = 10000;
    int data[data_length];
    
    InitHyperADCPins();
    
    SpiChnEnable(SPI_CHANNEL2, 0);
//    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
//    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_SSEN | SPI_CONFIG_CKE_REV| SPI_CONFIG_MODE8 | SPI_CONFIG_ON));
//    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)( SPI_CONFIG_CKE_REV | SPI_CONFIG_MODE32| SPI_CONFIG_ON));
//    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_FRMEN |  SPI_CONFIG_MODE32| SPI_CONFIG_ON));
    SpiChnConfigure(SPI_CHANNEL2, (SpiConfigFlags)(SPI_CONFIG_FRMEN |
                                                   SPI_CONFIG_FSP_IN |
                                                   SPI_CONFIG_CKE_REV  | 
                                                   SPI_CONFIG_CKP_HIGH |
                                                   SPI_CONFIG_MODE32   | 
                                                   SPI_CONFIG_ON |
                                                   SPI_CONFIG_DISSDO));

    SpiChnEnable(SPI_CHANNEL2, 1);

    SPI2CONbits.ON = 1;
    
    WaitMS(10);
    
    
    int idx = 0;
    int dummy = 0;
    
//    mPORTAClearBits(BIT_3);
    
    HYPER_ADC_CS(0);
    
    printf("Toggling ....\n");
    
    uint32_t data_ready_count = 0;
    
//    mPORTD
    
//    mPORTASetPinsDigitalOut(BIT_9); // J11.51
 
    
       
//    while(1)
//    {
//        if((data_ready_count++ % 10) == 0)
//            mPORTAToggleBits(BIT_9);
//        
//        if(IsDataReady)
//        {
//            printf("Data ready called %d times\n", data_ready_count);
//
//            IsDataReady = FALSE;
//        }
//    }
 

    
//    while(1)
//    {
//        if(IsDataReady)
//        {
//            if((data_ready_count++ % 10000000L) == 0)
//            {
//                printf("Data ready called %d times\n", data_ready_count);
//            }
//
//            IsDataReady = FALSE;
//        }
//    }
    
    
    
    
//    while(1)
//    {
////        HYPER_ADC_CS(0);
// 
////        mPORTBToggleBits(BIT_0);
//
////        mPORTDToggleBits(BIT_11 | BIT_8 | BIT_9); // J11... 23 24 25 26
////        mPORTBToggleBits(BIT_5);
//        WaitMS(1);
//
////        HYPER_ADC_CS(0);
//
//    }
    
    int rx1, rx2, rx3, rx4;
    
    
    printf("Reading HyperADC data\n");
    
    data_ready_count  = 0;
            
    while(1)
    {
        if(IsDataReady | 1)
        {
//            mPORTAClearBits(BIT_9);

//             if((data_ready_count++ % 10000000L) == 0)
//            {
//                printf("Data ready called %d times\n", data_ready_count);
//            }
            
//            rx1 = SPI2BUF;
//            SPI2BUF = 1;
//            rx2 = SPI2BUF;
//            SPI2BUF = 2;
//            rx3 = SPI2BUF;
//            SPI2BUF = 4;
//            rx4 = SPI2BUF;
//            SPI2BUF = 8;
            
            
//            rx1 = SpiChnGetC(SPI_CHANNEL2);
//            SpiChnPutC(SPI_CHANNEL2, 1);
//            rx2 = SpiChnGetC(SPI_CHANNEL2);
//            SpiChnPutC(SPI_CHANNEL2, 2);
//            rx3 = SpiChnGetC(SPI_CHANNEL2);
//            SpiChnPutC(SPI_CHANNEL2, 4);
                
//            mPORTASetBits(BIT_9);
            
            rx1 = SPI2BUF;
            
//            data[idx] = ((int32_t)((rx1 << 16) + (rx2 << 8) + (rx3 ))) ;
//            data[idx] = (int32_t) rx1;

            data[idx] = (int32_t) rx1;//HyperADC_RX32;;

            idx++;
            
            if(idx >= data_length)
            {
                SPI2CONbits.ON = 0;
                int j = 0;
            
                while(j < data_length)
                {
                    printf("%d:\t%X\n", j, data[j]);
                    data[j] = 0;
                    j++;
                }

                idx = 0;

                SPI2CONbits.ON = 1;
                SPI2CONbits.ON = 1;
                SPI2CONbits.ON = 1;
            }
            
            IsDataReady = FALSE;
        }
        
        int k = 0;

//        if(idx % data_length == 0)
//        {
//            for(k = 0; k < data_length; k++)
//            {
//                printf("%X\n", data[k]);
//            }
//            idx = 0
//        }
//    
        IsDataReady = FALSE;

    }

//    data_ready_counter  = 0;
    while(1)
    {
        if(IsDataReady)
        {
            idx = 0;
            while(idx < data_length)
            {
                rx1 = SpiChnGetC(SPI_CHANNEL2);
                SpiChnPutC(SPI_CHANNEL2, 1);
                rx2 = SpiChnGetC(SPI_CHANNEL2);
                SpiChnPutC(SPI_CHANNEL2, 2);
                rx3 = SpiChnGetC(SPI_CHANNEL2);
                SpiChnPutC(SPI_CHANNEL2, 4);
 
                rx4 = SpiChnGetC(SPI_CHANNEL2);
                SpiChnPutC(SPI_CHANNEL2, 8);

                data[idx] = ((int32_t)((rx1 << 24) + (rx2 << 16) + (rx3 << 8))) >> 8;

                idx++;
            }
            
            idx = 0;
            for(idx = 0; idx < data_length; idx++)
            {
                printf("%d:\t%X\n", idx, data[idx]);
            }
            IsDataReady = FALSE;
        }
    }
}


void TestDirectDataReadWrite()
{

  
    char fstr[10000];
    
    float val = -1208.02092;
    
    ftoa(val, fstr, 4);
    
    int count = 0;
    
//    void * nv_address = (void *)0xBD010000;
    float   nv_value = 10.0; 
    
    float fdata[100];
    float rx_fdata[100];
    
    int ik = 0;
    
    while(ik < 100)
    {
        fdata[ik] = nv_value + ik + 0.01 * ik;
        ik++;
    }
    
    IntFloatValue *ifdata = (IntFloatValue *) fdata;
    
    SaveIntFloatDataToFlashMemory(ifdata, 100, 0);
    LoadFloatDataFromFlashMemory(rx_fdata, 100, 0);
    
    ShowFloatData(fdata, 100);

    
    int ms = 0;
    while(1)
    {
        ik = 0;
        for(ik = 0; ik < 100; ik++)
        {
            int d0 = (int) fdata[ik];
//            WriteUART1(d0);
            
//            WRITE_TO_UART1(d0);
//            U2TXREG = d0;
//            while (U2STAbits.UTXBF);
            
            printf("%d\n", d0);
            WaitMS(1);
//            ms = 0;
//            while(ms++ < 10000);
 
        }
        
//        ms = 0;
//        while(ms++ < 10000);
        
        WaitMS(1);
   
    }

    
}



void __ISR(_EXTERNAL_1_VECTOR, IPL7SRS) __INT1Handler()
{
    HyperADC_RX32 = SPI2BUF;
//    HyperADC_RX32 = SpiChnGetC(SPI_CHANNEL2);
    
    IsDataReady = TRUE;
    
//    printf("Data Ready triggered\n");
    mINT1ClearIntFlag();

}


Queue * createQueue(int maxElements)
{
    Queue *Q;
    Q = (Queue *)malloc(sizeof(Queue));
    Q->elements = (float *)malloc(sizeof(float)*maxElements);
    Q->size = 0;
    Q->capacity = maxElements;
    Q->front = 0;
    Q->rear = -1;
    return Q;
}


BOOL Dequeue(Queue *Q, int *front_val)
{
    /* If Queue size is zero then it is empty. So we cannot pop */
    if(Q->size==0)
    {
        printf("Queue is Empty\n");
        return FALSE;
    }
    /* Removing an element is equivalent to incrementing index of front by one */
    else
    {
        *front_val = Q->elements[Q->front];
        Q->size--;
        Q->front++;
        /* As we fill elements in circular fashion */
        if(Q->front == Q->capacity)
        {
            Q->front = 0;
        }
        return TRUE;
    }
    
    return FALSE;
}

BOOL front(Queue *Q, float *front_val)
{
    if(Q->size==0)
    {
        printf("Queue is Empty\n");
        return FALSE;
    }

    *front_val =  Q->elements[Q->front];
    return TRUE;
}


BOOL Enqueue(Queue *Q,float element)
{
    /* If the Queue is full, we cannot push an element into it as there is no space for it.*/
    if(Q->size == Q->capacity)
    {
        printf("Queue is Full\n");
        return FALSE;
    }
    else
    {
        Q->size++;
        Q->rear = Q->rear + 1;
        /* As we fill the queue in circular fashion */
        if(Q->rear == Q->capacity)
        {
                Q->rear = 0;
        }
        /* Insert the element in its rear side */ 
        Q->elements[Q->rear] = element;
        return TRUE;
    }
}


void QueueToList(Queue *Q, float *list, int lest_len)
{
    
}

