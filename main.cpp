/* 
 * File:   main.cpp
 * Author: kamal
 *
 * Created on November 3, 2015, 2:04 PM
 */

#define _SUPPRESS_PLIB_WARNING
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include "TemperatureController.h"

#include <cstdlib>
#include <plib.h>
#include <stdint.h>
#include <xc.h>
#include <time.h>

//#include <filesystem>

//#include <S>


    
#include <xc.h>

// DEVCFG3
// USERID = No Setting
#pragma config FSRSSEL = PRIORITY_7     // Shadow Register Set Priority Select (SRS Priority 7)
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2 // with HS as osc source // base closk 8MHz
#pragma config FPLLIDIV = DIV_1         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 1)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)

#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)

//// DEVCFG2 // with XT as osc source
//#pragma config FPLLIDIV = DIV_1         // PLL Input Divider (2x Divider)
//#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
//#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
//#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
//#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
//#pragma config POSCMOD = XT             // Primary Oscillator Configuration (HS osc mode)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/2)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
//#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config DEBUG = ON               // Background Debugger Enable (Debugger is Enabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)


using namespace std;



void _mon_putc(char c)
{
//   while (U2STAbits.UTXBF);
//   U2TXREG = c;

   U1TXREG = c;
   while (U1STAbits.UTXBF);
}



//
//void WaitMS(int ms)
//{
//    uint32_t ms_count = (100000L/(25*4)*8L);
//    
//    while(ms_count-- >= 0)
//    {
//    }
//}

void InitPrintToUART2()
{
    
    mPORTFSetPinsDigitalOut(BIT_12);    // CTS
    mPORTFSetPinsDigitalIn(BIT_13);     // RTS
    
    mPORTFSetPinsDigitalIn(BIT_4);      // RxD
    mPORTFSetPinsDigitalOut(BIT_5);     // TxD

    PPS_Unlock();
    {
        U2CTSRbits.U2CTSR = 0b1001; // RF12
        RPF13Rbits.RPF13R  = 1; // U2RTS 
        
        U2RXRbits.U2RXR = 2; // 0b0010; // RF4 input J11.46
        RPF5Rbits.RPF5R = 1; //0b0011; TX output J11.48
    }
    PPS_Lock();

    
    __XC_UART = 2; //and the following 3 lines are necessary for stdout redirection
            
    U2MODEbits.BRGH = 1;               // Set High speed mode
    U2BRG = UBRG(UART2_BAUD);          // Configure Baud Rate
    setbuf(stdout, NULL);
    
    U2MODEbits.ON = 1;
    
}

void MapPins()
{
    
    
    
}

float ConvertToLSDTemp(int value)
{
    return (float)value * (0.3125);
}


void Test(int val)
{
}



void print_to_uart1()
{
    
    mPORTDSetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2);
//    mPORTDSetBits(BIT_0 | BIT_1 | BIT_2);
    InitUART1();
    
    __XC_UART = 1;
    
    int count = 0;
    
    while(1)
    {
//        UART_SendNumberAsString(count++);
        count++;
//        WaitMS(1);
        
        int i = 100000;
        
        while(i-- > 0);
        
//        mPORTDToggleBits(1 << (count % 3));
//        printf("\n test hello world %d\n", count);
    }
    
}


/*
 * 
 */
int main(int argc, char** argv) 
{
    
    float temp ; 

//    SystemReset();
    
    InitSystem_Test();
//    InitSystem_Test();



//    mPORTDSetPinsDigitalOut(BIT_0);
//    mPORTBSetPinsDigitalOut(BIT_14);
//    
//    mPORTGSetPinsDigitalOut(BIT_9);
//    mPORTGSetPinsDigitalOut(BIT_8);
//    
//    
//    mPORTASetPinsDigitalOut(BIT_0 | BIT_1);
//    mPORTAClearBits(BIT_0 | BIT_1);
//    
//    mPORTDSetPinsDigitalOut(BIT_2);
//    
//    mPORTGClearBits(BIT_8);
//    mPORTGClearBits(BIT_9);
    
    
    
 
//    CNPDGbits.CNPDG8 = 0;
//    CNPDGbits.CNPDG9 = 0;
//    mPORTDClearBits(BIT_0);
//        SPI1_TempMeasurement_LM_Thermo_Test();
//    CS_INIT();
//    DPDT_INIT();
//    
    
    TemperatureSystemInit();
    
//    SPI1_TempMeasurement_LM_Thermo_Test();
//    SPI1_TempMeasurement_Test();
    
    
//    return 0;
//    SPI1_TempMeasurement_LM_Thermo_Test();

//    return 0;

    
//    InitPWM(500, 50);
    
//    InitTimer1(500);

    InitTimer1(10);
    
//    printf("Timer set to 10 Hz\n");

//    SPI1_TempMeasurement_Test();
//    SPI1_TempMeasurement_LM_Thermo_Test();
//    
//    return 0;   
//    InitSPI2(16);
    
//    WaitReadUART2();

//    
//    int ws = 0;
//    int cc = 0;
//    while(1)
//    {
//        float temp = LSD_Temperature();
//        
//        printf("%d: temperature %d\n", cc++, (int) temp);
//        WaitMS(50);
////        ws = 0; while(ws++ < 40000){asm("nop");};
////        ws = 0; while(ws++ < 40000){asm("nop");};
////        ws = 0; while(ws++ < 40000){asm("nop");};
//    }
//    
//    
//    TestUART2DataSendToRaspberryPi();
//    return 0;
    
 

//    SPI1_TempMeasurement_Test();
//    
//    int idxx = 0;
//    while(1)
//    {
//        printf("count %d\n", idxx++);
//        WaitMS(1);
//        
//    }
//    
    
//    TestUART2();
    Test_SPI2Slave_DataTransferWithUART2();
//    return 0;

//    return 0;
//    ReadSPI2Slave_test();
//    return 0;

//    TestSPI1_Master();
//    
//    return 0;
    
    
//    AdjustTemperature(35.0);
    return 0;

    time_t  tm;
    
    while(1)
    {
        mPORTGToggleBits(BIT_8 | BIT_9);
//        mPORTDToggleBits(BIT_2);
//        tm = time(&tm);

//        printf("Time == %ld\n", tm);
//        WaitMS(10);

    }
    return 0;
    
    ReadWriteUART2();
    
    

    int k = 0;
    int ix = 0;
//    
//    while(1)
//    {
//        
//        asm("nop");
//        
//        k++;
//        k %= 100;
//        SetPWMDutyCycle(1, 98);
//        
//        ix = 0; while(ix++ < 10000);
//    }
////    Adjust

    int idx = 0;

    
//    FSFILE *fs_ptr;
//    
//    fs_ptr = FSfopen()
    
    
    char fstr[10000];
    
    float val = -1208.02092;
    
    ftoa(val, fstr, 4);
    
    int count = 0;
    char *fname = "test.txt";
    FILE *fptr = fopen(fname,"w");
    
    
    void * nv_address = (void *)0xBD010000;
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
//            WriteUART2(d0);
            
            WRITE_TO_UART2(d0);
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
    return 0;        
    

    
    
    int f;
    int add_i = 0;
    
    
    int max_data_len = 50;

    unsigned int data_val_uint = nv_value;
//    
//    while(add_i < max_data_len)
//    {
//        unsigned int *tx_addr;// = ((unsigned int *) nv_address) ;
//        unsigned int fpart = (unsigned int) (((float)nv_value - (float)((int) nv_value)) *1000.0);
//        
//        tx_addr = (unsigned int *)( ((unsigned int *) nv_address) + add_i) ;
//        
//        NVMWriteWord(tx_addr,  nv_value);
//        NVMWriteWord(tx_addr+1,  fpart);
//
////        *((float *)tx_addr) = nv_value;
//        
////        if(f > 0)
////        {
////            printf("Data written correctly\n");
////        }
//
//        add_i += 2;
//        nv_value += 1.13;
//    }
//    
    
    
    while(1)
    {
        int idx;
        
        idx = 0;
        
        while(idx < max_data_len)
        {
            unsigned int *rx_address = ((unsigned int *)nv_address) + idx;
            unsigned int rx_value = *rx_address; 
            unsigned int rx_fpart_value = *(rx_address+1); 
//            float *rx_address = (( float *)nv_address) + idx;
//            float rx_value = *rx_address; 
            
            printf(" %d:\tAddress %X,\t rx_value %d fpart = %3d / 1000\n", 
                    idx, 
                    (unsigned int)rx_address, 
                    (int)rx_value, 
                    rx_fpart_value);
            
//            WaitMS(1);
            idx += 2;
        }
    }
    

    if(fptr == NULL)
    {
        while(1)
        {
            printf("Could not create file....\n");
            WaitMS(1);
        }
    }
    
    while(count++ < 1000)
    {
        float nval = val + count;
        
        ftoa(nval, fstr, 4);
        
        fprintf(fptr, "%d:\t%s\n", count,  fstr);
    
    }
    fclose(fptr);
    
    fptr =  fopen(fname,"rt");
    
    char data[100];
    while(1)
    {
        int data_num;
        fscanf(fptr,"%s", data);
        
        printf("%s\n", data);
//        printf("%s\n", fstr);
        WaitMS(1);
    
    }
    
    fclose(fptr);
    
//    ReadWriteUART2();
    
    return 0;
    
    
//    AdjustTemperature(35.0);
//    
//    return 0;
   
    
    
    while(1)
    {
        printf("Before temperature measurement\n");
        
        float tt;// = LSD_Temperature();
        tt = LSD_Temperature();
        
//        LSD_CS(0);
//        
//        mPORTGSetBits(BIT_15);
        
        printf("Temperature = %d\n",(int) tt);
    
        while(idx++ < 10)
        {
        }
//        LSD_CS(1);
        
//        mPORTGClearBits(BIT_15);

        WaitMS(1);
        
        idx = 0;
        
        
    }
    
    return 0;
            
    
    count =0;
    while(1)
    {
//        printf("Hello World!!!! %d\n", count++);
        WaitMS(1);
    }
    
    return 0;
}



