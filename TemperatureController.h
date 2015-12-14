/* 
 * File:   TemperatureController.h
 * Author: kamal
 *
 * Created on November 3, 2015, 6:23 PM
 */

#ifndef TEMPERATURECONTROLLER_H
#define	TEMPERATURECONTROLLER_H

#define _SUPPRESS_PLIB_WARNING


//#define _DISABLE_OPENADC10_CONFIGPORT_WARNING


#include <cstdlib>
#include <plib.h>
#include <stdint.h>
#include <math.h>
#include <xc.h>
#include "PeripheralSettingsAndMacros.h"
#include "PID.h"

#ifdef __cplusplus
extern "C" {
#endif

#define  CS_LOW                 0
#define  CS_HIGH                1

#define  LSD_CS(val)            {LATGbits.LATG15 = val;}
#define  PELTIER_TOP_CS(val)    {LATGbits.LATG12 = val;}
#define  PELTIER_BOTTOM_CS(val) {LATGbits.LATG14 = val;}
#define  ADISCO_CS(val)         {LATEbits.LATE7 = val;}
#define  FACE_PLATE_CS(val)     {LATEbits.LATE6 = val;}


#define LSD_CS_EX(val)              {LSD_CS(val); PELTIER_TOP_CS(CS_HIGH); PELTIER_BOTTOM_CS(CS_HIGH); ADISCO_CS(CS_HIGH); FACE_PLATE_CS(CS_HIGH);}
#define PELTIER_TOP_CS_EX(val)      {LSD_CS(CS_HIGH); PELTIER_TOP_CS(val); PELTIER_BOTTOM_CS(CS_HIGH); ADISCO_CS(CS_HIGH); FACE_PLATE_CS(CS_HIGH);}
#define PELTIER_BOTTOM_CS_EX(val)  {LSD_CS(CS_HIGH); PELTIER_TOP_CS(CS_HIGH); PELTIER_BOTTOM_CS(val); ADISCO_CS(CS_HIGH); FACE_PLATE_CS(CS_HIGH);}
#define ADISCO_CS_EX(val)           {LSD_CS(CS_HIGH); PELTIER_TOP_CS(CS_HIGH); PELTIER_BOTTOM_CS(CS_HIGH); ADISCO_CS(val); FACE_PLATE_CS(CS_HIGH);}
#define FACE_PLATE_CS_EX(val)       {LSD_CS(CS_HIGH); PELTIER_TOP_CS(CS_HIGH); PELTIER_BOTTOM_CS(CS_HIGH); ADISCO_CS(CS_HIGH); FACE_PLATE_CS(val);}
                    
    
#define  DPDT_1(val)             {LATAbits.LATA6 = val;}
//#define  DPDT_2(val)             { (val) > 0 ? mPORTASetBits(BIT_7): mPORTAClearBits(BIT_7); } //{LATAbits.LATA7 = val;}
#define  DPDT_2(val)             {LATAbits.LATA7 = val;}


#define  CS_1(val)              {LATEbits.LATE5 = val;}
#define  CS_2(val)              {LATEbits.LATE4 = val;}
#define  CS_3(val)              {LATEbits.LATE3 = val;}
#define  CS_4(val)              {LATEbits.LATE2 = val;}
#define  CS_5(val)              {LATEbits.LATE1 = val;}


#define  CS_INIT()              {\
                                 mPORTESetPinsDigitalOut((unsigned int)0b11111110);\
                                 mPORTGSetPinsDigitalOut(BIT_12 | BIT_14 | BIT_15);\
                                 mPORTESetBits((unsigned int)0b11111110);\
                                 mPORTESetBits(BIT_12 | BIT_14 | BIT_15);\
                                }

#define  DPDT_INIT()            {\
                                  mPORTASetPinsDigitalOut(BIT_6 | BIT_7);\
                                  mPORTAClearBits(BIT_6 | BIT_7);\
                                }


#define INT_PART(val)           ((int)val)
#define FRAC_PART(val, mult)    ((int) ((float)(val - INT_PART(val))* (float)mult))

    
void AdjustTemperature(float target_temp);
float LSD_Temperature();

void TestUART2DataSendToRaspberryPi();

float PmodTC1_Temperature();   
float LSD_Temperature();


void SPI1_TempMeasurement_Test();
void SPI1_TempMeasurement_LM_Thermo_Test();

void TemperatureSystemInit();


void AddTemperatureToList(float temp_value);
void ClearTemperatureList();
void Calculate_SD_And_MD(float target_temp, float *sd, float *md);

#ifdef __cplusplus
}
#endif

#endif	/* TEMPERATURECONTROLLER_H */

