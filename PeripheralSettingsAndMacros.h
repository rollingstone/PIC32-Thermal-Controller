/* 
 * File:   PeripheralSettingsAndMacros.h
 * Author: kamal
 *
 * Created on November 4, 2015, 11:11 AM
 */

#ifndef PERIPHERALSETTINGSANDMACROS_H
#define	PERIPHERALSETTINGSANDMACROS_H

#define _SUPPRESS_PLIB_WARNING
//#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include <plib.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
typedef union __IntFloatValue_tag {
    float fvalue; // float and int data types take equal byte space (4 byte or 2 byte)
    int   ivalue;
} IntFloatValue;    

typedef struct {
    uint8_t    lower;
    uint8_t    upper;
} Int16Bytes;

typedef union __Int16Value_tag{
    Int16Bytes  v;
    uint16_t    _value;
} UInt16Value;


typedef struct Queue
{
    int capacity;
    int size;
    int front;
    int rear;
    float *elements;
}Queue;



#define CPU_CLOCK               (80*1000000L)
//#define CPU_CLOCK               (96L*1000000L)

#define SYS_FREQ                    CPU_CLOCK
#define	GetPeripheralClock()		(SYS_FREQ/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(SYS_FREQ)

#define PBUS_CLOCK                  GetPeripheralClock()


#define MS_LOOP                 ((uint32_t)( (((double)80000 )) / ((double)800000000L / (double)SYS_FREQ)))

//#define     DPDT_PIN        
#define     BAUD_RATE_SPI   500000  // 500 kHz


//#define     PALTIER_TOP_CS           
//#define     PALTIER_BOTTOM_CS
//
//#define     ADISCO_CS
//#define     LSD_CS


#define     UBRG(baud)           (((GetPeripheralClock())/4/(baud)-1))
#define     UART2_BAUD          115200L   
#define     UART_BAUD_RATE      115200L

#define     PPS_Unlock()      {SYSKEY = 0; SYSKEY = 0xAA996655; SYSKEY = 0x556699AA; CFGCONbits.IOLOCK = 0; }
#define     PPS_Lock()        {CFGCONbits.IOLOCK = 1; SYSKEY = 0;}
    
#define     UnlockPPS()         PPS_Unlock()
#define     LockPPS()           PPS_Lock()

#define     WAIT_AND_READ_FROM_UART1(val)               {while(U1STAbits.URXDA == 0); val = U1RXREG; }
#define     WAIT_AND_READ_FROM_UART2(val)               {while(U2STAbits.URXDA == 0); val = U2RXREG; }
    
#define     WRITE_TO_UART1(data_byte)       { while(U1STAbits.UTXBF); U1TXREG = ( (unsigned int)(data_byte) ); } 
#define     WRITE_TO_UART2(data_byte)       { while(U2STAbits.UTXBF); U2TXREG = ( (unsigned int)(data_byte) ); } 
    
#define     BASE_DATA_ADDRESS_NVM   0xBD008000
#define     MAX_DATA_SIZE_NVM       (0xFFFF >> 1)    

#define     ABS_VALUE(val)          ( (val) < 0? (-val):(val) )

#define     NOP()                   asm("nop")

#define     SPI2_SLAVE_READ()                       SpiChnGetC(SPI_CHANNEL2)
#define     SPI2_SLAVE_WRITE(val, dummy)            {SpiChnPutC(SPI_CHANNEL2,(val)); dummy = SPI2BUF; }

#define     SPI2_CLEAR_READ_BUF(dummy)                        {while(SPI2STATbits.SPIRBF) dummy = SPI2BUF;}

#define     SYSTEM_SOFT_RESET()         { SoftReset(); NOP(); NOP(); NOP(); NOP(); while(1){NOP();}; }


#define     SPI_BRG_VAL(spi_clk)        ( (GetPeripheralClock() / ((spi_clk) << 1)) -1 )

// Note that 612Hz (PR2=0xffff) is the lowest pwm frequency with our configuration
// : To get lower, use a timer prescaler or use the 32-bit timer mode
#define         PWM_FREQ    16000
#define         DUTY_CYCLE  10    

#define     TIMER_1_SCALE_DOWN_FACTOR    1



void InitSystem();
//void InitSystem_Test();

void SystemReset();

void SetupDebugGPIOPins();


void InitPWM(int sample_rate, int duty_cycle);
void InitPWM_v3(int pwm_frequency, int duty_cycle);

int  SetPWMDutyCycle(int pwm_number, int duty_cycle);

void InitTimer1(uint32_t freq);
uint32_t GetTimer1_Freqency();

void InitTimer4(uint32_t freq);

void InitSPI1Slave();
void InitSPI2Slave();


void InitSPI1(int freq_hz);
void InitSPI2(int freq_hz);


int  InitUART1();
int  InitUART2();
void WaitMS(unsigned int ms);
void SplitFloat2Ints(float fval, int *int_val, int *frac_val);

void MapSPI2MasterPins();
void MapSPI2SlavePins();

void ReadWriteUART2();


    // reverses a string 'str' of length 'len'
void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char *res, int afterpoint);


int SaveFloatDataToFlashMemory(float *data, int length, unsigned int offset /* values */);
int SaveInttDataToFlashMemory(unsigned int *data, int length, unsigned int offset /* values */);
int LoadInttDataFromFlashMemory(unsigned int *data, int length, unsigned int offset /* values */);
int LoadFloatDataFromFlashMemory(float *data, int length, unsigned int offset /* values */);



int SaveIntFloatDataToFlashMemory(IntFloatValue *data, int length, unsigned int offset /* values */);
int LoadIntFloatDataFromFlashMemory(IntFloatValue *data, int length, unsigned int offset /* values */);


void FloatToIntSplit(float data, int *int_part, int *float_part);
void ShowFloatData(float *data, int length);

void WaitReadUART2();

inline BOOL IsDataWaitingUART1();
inline BOOL IsDataWaitingUART2();

BOOL ReadCommandFromUART1(int *command, int length);
BOOL ReadCommandFromUART2(int *command, int length);

BOOL ReadCommandFromSPI2(int *command, int length);

void *AllocateMaxPossibleMemory(uint32_t *data_length);

void ReadSPI1Slave_test();
void ReadSPI2Slave_test();

void TestSPI1_Master();

void Test_SPI2Slave_DataTransfer();
void Test_SPI2Slave_DataTransferWithUART2();

void Test_SPI2Slave_DataTransferWithUART1();


void Test_SPI2Slave_DataTransferWith_SPI2Command();


void TestUART2();
void TestUART1DataReadWrite();


void TestSPi2Slave();

int SendDataBySPI2Slave(UInt16Value *data, int data_length_in_bytes);
int SendDataBySPI2SlaveWithTimeOut(UInt16Value *data, int data_length_in_bytes, int time_out_loop);


void TestSPi2Slave_WithSendData();
void TestSPi2Slave_WithSendData_WithSPI2Command();

void TestSpi2CReadCommand();


inline void SPI2CleanReadBuffer();


void TestSPI1Master_To_SPI2SlaveData_Transfer(uint32_t brate);


void InitExtINTs();
void TestHyperADC_SPI2Slave_Read();



Queue * createQueue(int maxElements);
BOOL Dequeue(Queue *Q, int *front_val);
BOOL front(Queue *Q, float *front_val);
BOOL Enqueue(Queue *Q, float element);

#ifdef	__cplusplus
}
#endif

#endif	/* PERIPHERALSETTINGSANDMACROS_H */

