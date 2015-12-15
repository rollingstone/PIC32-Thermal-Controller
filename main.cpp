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




//
//void WaitMS(int ms)
//{
//    uint32_t ms_count = (100000L/(25*4)*8L);
//    
//    while(ms_count-- >= 0)
//    {
//    }
//}


//inline float ConvertToLSDTemp(int value)
//{
//    return (float)value * (0.3125);
//}


/*
 * 
 */
int main(int argc, char* argv[]) 
{
   // if a system reset is required comment out the following line and run the program
//    SYSTEM_SOFT_RESET();
    

    uint32_t    timer_freq = 5;//Hz 
    
    InitSystem();
    InitTimer1(timer_freq);
    TemperatureSystemInit();
    
// ***************** The following commented out function calls perform 
//    various tests. Used mainly for debug purposed
//    
//    TestSPi2Slave();
//    TestHyperADC_SPI2Slave_Read();
//    TestSPI1Master_To_SPI2SlaveData_Transfer(8L*1000000L);
    
//    TestSPi2Slave();
//    Test_SPI2Slave_DataTransfer();
//    Test_SPI2Slave_DataTransferWithUART1();
//    Test_SPI2Slave_DataTransferWith_SPI2Command();
//    ReadSPI2Slave_test();
//    TestSPI1_Master();

    
//    SPI1_TempMeasurement_LM_Thermo_Test();
//    SPI1_TempMeasurement_Test();
//    TestUART2DataSendToRaspberryPi();
//    TestSPi2Slave_WithSendData_WithSPI2Command();

//    TestSPi2Slave_WithSendData();
//    InitPWM(1500,20);
//    SetPWMDutyCycle(1, 30);
    
    AdjustTemperature(27.0);

    return 0;
}

