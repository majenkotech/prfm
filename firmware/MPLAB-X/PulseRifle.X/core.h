#ifndef _CORE_H
#define _CORE_H

#define SAMPLERATE 11025
#define MAX_CALLBACK 10

#define I2CTIMEOUT 1000

typedef void (*voidfunc) ();

#define IO0 0
#define IO1 1
#define IO2 2
#define IO3 3
#define IO4 4
#define IO5 5
#define IO6 6
#define IO7 7
#define D0 8
#define D1 9
#define D2 10
#define D3 11
#define D4 12
#define D5 13
#define D6 14
#define D7 15
#define PROG 16
#define LED0 17
#define LED1 18
#define MAX_IO 19

struct sample {
    short * sample;
    int len;
    int delay;
    int pos;
    int speed;
    int spos;
    unsigned char volume;
};

#define MAX_SAMPLES 28
extern struct sample samples[MAX_SAMPLES];

#define OUT 0
#define IN  1
#define ANA 2

#define DEBOUNCE 50

#define MCP23017_ADDRESS 0x20

// registers
#define MCP23017_IODIRA 0x00
#define MCP23017_IPOLA 0x02
#define MCP23017_GPINTENA 0x04
#define MCP23017_DEFVALA 0x06
#define MCP23017_INTCONA 0x08
#define MCP23017_IOCONA 0x0A
#define MCP23017_GPPUA 0x0C
#define MCP23017_INTFA 0x0E
#define MCP23017_INTCAPA 0x10
#define MCP23017_GPIOA 0x12
#define MCP23017_OLATA 0x14


#define MCP23017_IODIRB 0x01
#define MCP23017_IPOLB 0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTCONB 0x09
#define MCP23017_IOCONB 0x0B
#define MCP23017_GPPUB 0x0D
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPB 0x11
#define MCP23017_GPIOB 0x13
#define MCP23017_OLATB 0x15


struct io {
    volatile unsigned int *TRIS;
    volatile unsigned int *LAT;
    volatile unsigned int *PORT;
    int pin;
    int adc;
    int cn;
    int oc;
    int mode;
    int value;
    void (*function)(int, int);
    int debounce;
};

struct callback {
    voidfunc function;
    unsigned int frequency;
    unsigned int counter;
};
    

extern void setInput(int in);
extern void setDebounce(int in, int deb);
extern void setOutput(int in);
extern void setAnalog(int in);
extern void tieInput(int in, void (*func)(int, int));
extern int readInput(int i);
extern void queueSample(const short *sample, int length, int delay, int speed);
extern void queueSampleVol(const short *sample, int length, int delay, int speed, unsigned char volume);
extern void queueSingleSample(const short *sample, int length, int delay, int speed);
extern void queueSingleSampleVol(const short *sample, int length, int delay, int speed, unsigned char volume);
extern inline void sendSample(register int sample);
extern void initPlayback(int sr, unsigned char gain);
extern void initAutofire(int in, int speed, void (*func)(int, int));
extern void setAFSpeed(int speed);


extern void initDisplay();
extern void displayLeft(unsigned char l);
extern void displayRight(unsigned char r);
extern void displayRaw(unsigned char l, unsigned char r);
extern void displayValue(unsigned char v);
extern void setDecimalPoint(unsigned char l, unsigned char r);

extern void initPWM(unsigned int channel);
extern void setPWM(unsigned int channel, unsigned int duty);

extern void initCore();
extern int registerCallback(voidfunc f, unsigned int frequency);
extern void unregisterCallback(int i);
extern unsigned long millis();

extern void udelay (unsigned int usec);

#define C0_COUNT    9   /* Processor cycle count */
#define mips_write_c0_register(reg, sel, value)         \
do {                                \
    asm volatile (                      \
    "mtc0   %z0, $%1, %2 \n ehb"                            \
    : : "r" ((unsigned int) (value)), "K" (reg), "K" (sel));\
} while (0)


#endif

