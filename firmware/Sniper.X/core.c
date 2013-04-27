#include <p32xxxx.h>
#include <plib.h>

/* Timer Usage:
 *
 * Timer 1  Audio
 * Timer 2  Display
 * Timer 3  PWM
 * Timer 4  1ms ticker
 * Timer 5  Autofire
*/

#include "core.h"

struct sample samples[MAX_SAMPLES];
struct io inputs[MAX_IO] = {
    { &TRISB, &LATB, &PORTB, 0, 0, 2, 0, 0, 0, NULL, 0 },
    { &TRISB, &LATB, &PORTB, 1, 1, 3, 0, 0, 0, NULL, 0 },
    { &TRISB, &LATB, &PORTB, 2, 2, 4, 0, 0, 0, NULL, 0 },
    { &TRISB, &LATB, &PORTB, 3, 3, 5, 0, 0, 0, NULL, 0 },
    { &TRISB, &LATB, &PORTB, 4, 4, 6, 0, 0, 0, NULL, 0 },
    { &TRISB, &LATB, &PORTB, 5, 5, 7, 0, 0, 0, NULL, 0 },
    { &TRISB, &LATB, &PORTB, 6, 6, 8, 0, 0, 0, NULL, 0 },
    { &TRISB, &LATB, &PORTB, 7, 7, 9, 0, 0, 0, NULL, 0 },
    { &TRISB, &LATB, &PORTB,14,14,-1, 0, 0, 0, NULL, 0 },
    { &TRISF, &LATF, &PORTF, 4,-1,17, 0, 0, 0, NULL, 0 },
    { &TRISF, &LATF, &PORTF, 5,-1,18, 0, 0, 0, NULL, 0 },
    { &TRISD, &LATD, &PORTD, 6,-1,15, 0, 0, 0, NULL, 0 },
    { &TRISD, &LATD, &PORTD, 5,-1,14, 0, 0, 0, NULL, 0 },
    { &TRISD, &LATD, &PORTD, 3,-1,-1, 4, 0, 0, NULL, 0 },
    { &TRISD, &LATD, &PORTD, 2,-1,-1, 3, 0, 0, NULL, 0 },
    { &TRISD, &LATD, &PORTD, 1,-1,-1, 2, 0, 0, NULL, 0 },
    { &TRISC, &LATC, &PORTC,13,-1, 1, 0, 0, 0, NULL, 0 },
    { &TRISD, &LATD, &PORTD, 0,-1,-1, 1, 0, 0, NULL, 0 },
    { &TRISD, &LATD, &PORTD, 4,-1,13, 5, 0, 0, NULL, 0 },
};

unsigned char digits[] = {
    0b11010111,
    0b00000011,
    0b11001110,
    0b10001111,
    0b00011011,
    0b10011101,
    0b11011101,
    0b00010111,
    0b11011111,
    0b10011111,
};

const unsigned char slaveDigits[] = {
  0b01111110,
  0b00010010,
  0b10111100,
  0b10110110,
  0b11010010,
  0b11100110,
  0b11101110,
  0b00110010,
  0b11111110,
  0b11110110,
};

unsigned char slaveDP[8];


volatile unsigned char dp = 0;

volatile unsigned long tcount = 0;

// Initialise the change-notification inputs B0 to B5 (CN2 - CN7)
void setInput(int in)
{
    inputs[in].mode = IN;
    inputs[in].value = 1;
    *inputs[in].TRIS |= 1<<inputs[in].pin;

    if (inputs[in].adc >= 0) {
        AD1PCFG |= 1<<inputs[in].adc;
    }

    if (inputs[in].cn >= 0) {
        CNEN |= 1<< inputs[in].cn;
        CNPUE |= 1<< inputs[in].cn;
    }
}

void setDebounce(int in, int deb)
{
    inputs[in].debounce = deb;
}

void setOutput(int in)
{
    inputs[in].mode = OUT;
    inputs[in].value = 0;
    *inputs[in].TRIS &= ~(1<<inputs[in].pin);

    if (inputs[in].cn >= 0) {
        CNEN &= ~(1<< inputs[in].cn);
        CNPUE &= ~(1<< inputs[in].cn);
    }
}

void setAnalog(int in)
{
    inputs[in].mode = ANA;
    *inputs[in].TRIS |= 1<<inputs[in].pin;

    if (inputs[in].adc >= 0) {
        AD1PCFG &= ~(1<<inputs[in].adc);
    }

    if (inputs[in].cn >= 0) {
        CNEN &= ~(1<< inputs[in].cn);
        CNPUE &= ~(1<< inputs[in].cn);
    }
}

void tieInput(int in, void (*func)(int, int))
{
    inputs[in].function = func;
}

static inline int getIOState(int i)
{
    return ((*inputs[i].PORT) & (1<<inputs[i].pin)) ? 1 : 0;
}

int readInput(int i)
{
    if (inputs[i].mode == IN) {
        return getIOState(i);
    }

    return inputs[i].value;
}

void queueSample(const short *sample, int length, int delay, int speed)
{
    int i;
    for (i=0; i<MAX_SAMPLES; i++) {
        if (samples[i].sample == NULL) {
            samples[i].sample = (short *)sample;
            samples[i].len = length;
            samples[i].delay = delay;
            samples[i].pos = 0;
            samples[i].speed = speed;
            samples[i].spos = 0;
            return;
        }
    }
}

void queueSingleSample(const short *sample, int length, int delay, int speed)
{
    int i;
    int found = 0;
    for (i=0; i<MAX_SAMPLES; i++) {
        if (samples[i].sample == sample) {
            found = 1;
        }
    }
    if (found == 0) {
        queueSample(sample, length, delay, speed);
    }
}

inline void sendSample(register int sample)
{
    static int lastsample = 0;
    if (lastsample == sample) {
        return;
    }

    lastsample = sample;

    sample = sample + 32768;
    sample = sample >> 4;
    sample = sample & 0x0FFF;
    sample = sample | 0x3000;
    SPI2BUF = sample;
}

volatile int af_input = -1;
void (*af_func)(int, int) = NULL;

void initAutofire(int in, int speed, void (*func)(int, int))
{
    // Pulse Rifle Timer
    if (speed < 5) speed = 5;
    OpenTimer5( T5_ON | T5_SOURCE_INT | T5_PS_1_256, (80000000L / 256L) / speed);
    ConfigIntTimer5( T5_INT_ON | T5_INT_PRIOR_2);
    af_input = in;
    af_func = func;
}

void setAFSpeed(int speed)
{
    if (speed < 5) speed = 5;
    PR5 = (80000000L / 256L) / speed;
}

void initDisplay()
{
    TRISECLR = 0xFF;
    LATECLR = 0xFF;
    TRISFCLR = 0b11;
    LATFCLR = 0b11;
    OpenTimer2( T2_ON | T2_SOURCE_INT | T2_PS_1_4, 200);
    ConfigIntTimer2( T2_INT_ON | T2_INT_PRIOR_4);
}

volatile unsigned char left = 0;
volatile unsigned char right = 0;

void displayLeft(unsigned char l)
{
    left = l;
}

void displayRight(unsigned char r)
{
    left = r;
}

void displayRaw(unsigned char l, unsigned char r)
{
    left = l;
    right = r;
}

void displayValue(unsigned char v)
{
    unsigned char bcdl;
    unsigned char bcdh;

    bcdl = v % 10;
    bcdh = v / 10;
    displayRaw(digits[bcdh], digits[bcdl]);
}

void setDecimalPoint(unsigned char l, unsigned char r)
{
    dp = (l&1) << 1 | (r&1);
}

void __ISR(_TIMER_2_VECTOR, ipl4) _T2Interrupt(void)
{
    static int side = 0;

    if (side == 0) {
        LATFCLR = 0b11;
        LATECLR = 0xFF;
        udelay(1);
        LATESET = right;
        if (dp & 1) {
            LATESET = 0b00100000;
        }
        LATFSET = 0b01;
        side = 1;
    } else {
        LATFCLR = 0b11;
        LATECLR = 0xFF;
        udelay(1);
        LATESET = left;
        if (dp & 2) {
            LATESET = 0b00100000;
        }
        LATFSET = 0b10;
        side = 0;
    }
    mT2ClearIntFlag();
}

void __ISR(_TIMER_5_VECTOR, ipl2) _T5Interrupt(void)
{
    if (inputs[af_input].value == 0) {
        af_func(af_input, 0);
    }
    mT5ClearIntFlag();
}

void initPlayback(int sr)
{
    // Sample Playback Timer
    OpenTimer1( T1_ON | T1_SOURCE_INT | T1_PS_1_256, (80000000 / 256) / sr);
    ConfigIntTimer1( T1_INT_ON | T1_INT_PRIOR_3);

    // Set the SPI interface up so all we need do is place a word in the buffer
    // and the hardware does all the rest for us.
    OpenSPI2(FRAME_ENABLE_ON | FRAME_SYNC_OUTPUT | MASTER_ENABLE_ON | SEC_PRESCAL_1_1 | PRI_PRESCAL_1_1 | SPI_MODE16_ON | SPI_CKE_ON | SPI_SMP_ON, SPI_ENABLE );
    SpiChnSetBrg(2, 15);
    SPI2CONbits.FRMSYPW = 1;
    SPI2CONbits.FRMCNT = 0;
    SPI2CONbits.SPIFE = 1;

    INTEnableSystemMultiVectoredInt();
}

void __ISR(_TIMER_1_VECTOR, ipl3) _T1Interrupt(void)
{
    register int val = 0;
    register int i;
    register struct sample *s;

    for (i=0; i<MAX_SAMPLES; i++) {
        s = &samples[i];
        if (s->sample != NULL) {
            if (s->delay > 0) {
                s->delay--;
            } else {
                val += s->sample[s->pos];
                s->pos++;

                s->spos += s->speed;
                if (s->spos > 100) {
                    s->pos++;
                    s->spos = 0;
                }
                if (s->spos < -100) {
                    s->pos--;
                    s->spos = 0;
                }

                if (s->pos >= s->len) {
                    s->sample = NULL;
                }
            }
        }
    }

    val = val >> 2;
    if (val < -32768) {
        val = -32768;
    }
    if (val > 32767) {
        val = 32767;
    }

    sendSample(val);
    mT1ClearIntFlag();
}

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) _CNInterrupt(void)
{
    int i;
    int v;
    int stable;
    for (i=0; i<MAX_IO; i++) {
        if (inputs[i].cn < 0) {
            continue;
        }
        if (inputs[i].mode == IN) {
            v = getIOState(i);
            if (inputs[i].value != v) {
                inputs[i].value = v;
                stable = inputs[i].debounce;
                while (stable > 0) {
                    v = getIOState(i);
                    stable--;
                    if (inputs[i].value != v) {
                        inputs[i].value = v;
                        stable = inputs[i].debounce;
                    }
                }
                if (inputs[i].function) {
                    inputs[i].function(i, v);
                }
            }
        }
    }
    IFS1bits.CNIF = 0;
}

static int inline __attribute__ ((always_inline))
mips_intr_disable ()
{
    int status;
    asm volatile ("di   %0" : "=r" (status));
    return status;
}

static void inline __attribute__ ((always_inline))
mips_intr_restore (int x)
{
        /* C0_STATUS */
    mips_write_c0_register (12, 0, x);
}

/*
 * Read C0 coprocessor register.
 */
#define mips_read_c0_register(reg,sel)              \
    ({ int __value;                     \
    asm volatile (                      \
    "mfc0   %0, $%1, %2"                    \
    : "=r" (__value) : "K" (reg), "K" (sel));       \
    __value;                        \
})

/*
 * Microsecond delay routine for MIPS processor.
 */
void udelay (unsigned int usec)
{
    unsigned now = mips_read_c0_register (C0_COUNT, 0);
    unsigned final = now + usec * (80000 / 1000);

    for (;;) {
        now = mips_read_c0_register (C0_COUNT, 0);

        /* This comparison is valid only when using a signed type. */
        if ((int) (now - final) >= 0)
            break;
    }
}

static void nvm_operation (unsigned int op, unsigned int address, unsigned int data)
{
    int x;

    // Convert virtual address to physical
    NVMADDR = address & 0x1fffffff;
    NVMDATA = data;

    // Disable interrupts
    x = mips_intr_disable();

    // Enable Flash Write/Erase Operations
    NVMCON = _NVMCON_WREN_MASK | op;

    // Data sheet prescribes 6us delay for LVD to become stable.
    // To be on the safer side, we shall set 7us delay.
    udelay (7);

    NVMKEY = 0xAA996655;
    NVMKEY = 0x556699AA;
    NVMCONSET = _NVMCON_WR_MASK;

    // Wait for WR bit to clear
    while (NVMCON & _NVMCON_WR_MASK)
        continue;

    // Disable Flash Write/Erase operations
    NVMCONCLR = _NVMCON_WREN_MASK;

    // Enable interrupts
    mips_intr_restore (x);
}

const unsigned char settings_memory_block[4096] __attribute__((aligned(4096)));

void initPWM(unsigned int in)
{
    if (inputs[in].oc > 0) {
        switch (inputs[in].oc) {
            case 1:
                OC1CONbits.OCTSEL = 1;
                OC1CONbits.OCM = 0b110;
                OC1CONbits.ON = 1;
                OC1R = 0;
                break;
            case 2:
                OC2CONbits.OCTSEL = 1;
                OC2CONbits.OCM = 0b110;
                OC2CONbits.ON = 1;
                OC2R = 0;
                break;
            case 3:
                OC3CONbits.OCTSEL = 1;
                OC3CONbits.OCM = 0b110;
                OC3CONbits.ON = 1;
                OC3R = 0;
                break;
            case 4:
                OC4CONbits.OCTSEL = 1;
                OC4CONbits.OCM = 0b110;
                OC4CONbits.ON = 1;
                OC4R = 0;
                break;
            case 5:
                OC5CONbits.OCTSEL = 1;
                OC5CONbits.OCM = 0b110;
                OC5CONbits.ON = 1;
                OC5R = 0;
                break;
        }
    }
}

void setPWM(unsigned int in, unsigned int duty)
{
    if (inputs[in].oc > 0) {
        switch (inputs[in].oc) {
            case 1:
                OC1RS = duty;
                break;
            case 2:
                OC2RS = duty;
                break;
            case 3:
                OC3RS = duty;
                break;
            case 4:
                OC4RS = duty;
                break;
            case 5:
                OC5RS = duty;
                break;
        }
    }
}

struct callback callbacks[MAX_CALLBACK];

void initCore()
{
    int i;
    OpenTimer4( T4_ON | T4_SOURCE_INT | T4_PS_1_64, 1250);
    ConfigIntTimer4( T4_INT_ON | T4_INT_PRIOR_4);

    for (i=0; i<MAX_CALLBACK; i++) {
        callbacks[i].function = NULL;
        callbacks[i].frequency = 0;
        callbacks[i].counter = 0;
    }

    AD1CON1bits.FORM = 0b000;
    AD1CON1bits.SSRC = 0b111;
    AD1CON1bits.ASAM = 1;

    AD1CON2bits.VCFG = 0b000;
    AD1CON2bits.CSCNA = 1;
    AD1CON2bits.SMPI = 15;
    
    AD1CON3bits.ADRC = 0;
    AD1CON3bits.SAMC = 0b11111;
    AD1CON3bits.ADCS = 0b11111111;

    AD1CSSL = 0b11111111;

    AD1CON1bits.ON = 1;

 //   IFS1bits.CNIF = 0;
 //   IEC1bits.CNIE = 1;
 //   IPC6bits.CNIP = 2;
 //   CNCONbits.ON    =   1;
 //   CNCONbits.SIDL  =   0;

    OpenTimer3( T3_ON | T3_SOURCE_INT | T3_PS_1_1, 256);
}

void __ISR(_TIMER_4_VECTOR, ipl4) _T4Interrupt(void)
{
    int i,v;
    volatile unsigned int *buf = &ADC1BUF0;

    tcount++;

    for (i=0; i<MAX_IO; i++) {
        if (inputs[i].mode == ANA) {
            if (inputs[i].adc >= 0) {
                v = *(buf + inputs[i].adc);
                if (v != inputs[i].value) {
                    inputs[i].value = v;
                    if (inputs[i].function) {
                        inputs[i].function(i, inputs[i].value);
                    }
                }
            }
        }
        if (inputs[i].mode == IN) {
            v = getIOState(i);
            if (inputs[i].value != v) {
                inputs[i].value = v;
                if (inputs[i].function) {
                    inputs[i].function(i, inputs[i].value);
                }
            }
        }
    }

    for (i=0; i<MAX_CALLBACK; i++) {
        if (callbacks[i].function) {
            if (callbacks[i].counter == 0) {
                callbacks[i].counter = callbacks[i].frequency;
                callbacks[i].function();
            } else {
                callbacks[i].counter--;
            }
        }
    }
    mT4ClearIntFlag();
}

int registerCallback(voidfunc f, unsigned int frequency)
{
    int i;
    for (i=0; i<MAX_CALLBACK; i++) {
        if (callbacks[i].function == NULL) {
            callbacks[i].function = f;
            callbacks[i].frequency = frequency;
            callbacks[i].counter = frequency;
            return i;
        }
    }
    return -1;
}

void unregisterCallback(int i)
{
    if (i < 0 || i >= MAX_CALLBACK) {
        return;
    }
    callbacks[i].function = NULL;
    callbacks[i].counter = 0;
    callbacks[i].frequency = 0;
}

unsigned long millis()
{
    return tcount;
}

unsigned long i2c_baud(unsigned long f)
{
    unsigned long t;
    t = ((80000000UL / 8) * 104) / 125000000UL;
    return (80000000 / (2 * f) - t) - 2;
}

void initI2C()
{
    I2CSetFrequency(I2C5, 80000000UL, 100000);
    I2CEnable(I2C5, 1);
}

void i2c_startTransfer(unsigned char restart)
{
    I2C_STATUS status;
    if (restart) {
        I2CRepeatStart(I2C5);
    } else {
        while( !I2CBusIsIdle(I2C5) ); 
    }
    do
    {
        status = I2CGetStatus(I2C5);

    } while ( !(status & I2C_START) );
}

void i2c_transmitOneByte(unsigned char data)
{
    unsigned long timeout;
    // Wait for the transmitter to be ready
    timeout = 1000;
    while((!I2CTransmitterIsReady(I2C5)) && (--timeout > 0));
    if (timeout == 0) return;

    // Transmit the byte
    if(I2CSendByte(I2C5, data) == I2C_MASTER_BUS_COLLISION) {
        return;
    }

    // Wait for the transmission to finish
    timeout = 1000;
    while((!I2CTransmissionHasCompleted(I2C5)) && (--timeout > 0));
    if (timeout == 0) return;

    return;
}

void i2c_stopTransfer()
{
    unsigned long timeout;
    I2C_STATUS  status;

    // Send the Stop signal
    I2CStop(I2C5);

    // Wait for the signal to complete
    timeout = 1000;
    do
    {
        status = I2CGetStatus(I2C5);

    } while ( (!(status & I2C_STOP) ) && (--timeout > 0));
    if (timeout == 0) return;
}

void i2c_transmit(unsigned char address, unsigned char *data, unsigned char len)
{
    unsigned char i;
    unsigned long timeout;
    I2C_7_BIT_ADDRESS   SlaveAddress;
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, address, I2C_WRITE);
    i2c_startTransfer(1);
    i2c_transmitOneByte(SlaveAddress.byte);
    timeout = 1000;
    while ((!I2CByteWasAcknowledged(I2C5)) && (--timeout > 0));
    if (timeout == 0) return;
    for (i=0; i<len; i++) {
        i2c_transmitOneByte(data[i]);
        timeout = 1000;
        while ((!I2CByteWasAcknowledged(I2C5)) && (--timeout > 0));
        if (timeout == 0) return;
    }
    i2c_stopTransfer();
}

unsigned char slaveLeft, slaveRight;

void initSlave(unsigned char display)
{
    unsigned char data[2];
    initI2C();
    data[1] = 0; 
    data[0] = MCP23017_IODIRA;
    i2c_transmit(MCP23017_ADDRESS | display, data, 2);
    data[0] = MCP23017_IODIRB;
    i2c_transmit(MCP23017_ADDRESS | display, data, 2);
    data[0] = MCP23017_OLATA;
    i2c_transmit(MCP23017_ADDRESS | display, data, 2);
    data[0] = MCP23017_OLATB;
    i2c_transmit(MCP23017_ADDRESS | display, data, 2);
}

void slaveSetLeftDigit(unsigned char display, unsigned char d)
{
    slaveLeft = d;
    unsigned char data[2] = {MCP23017_OLATA, d};
    i2c_transmit(MCP23017_ADDRESS | display, data, 2);
}

void slaveSetRightDigit(unsigned char display, unsigned char d)
{
    slaveRight = d;
    unsigned char data[2] = {MCP23017_OLATB, d};
    i2c_transmit(MCP23017_ADDRESS | display, data, 2);
}

void slaveSetValue(unsigned char display, unsigned char v)
{
    slaveSetLeftDigit(display, slaveDigits[v / 10] | (slaveDP[display] & 0x02 ? 1 : 0));
    slaveSetRightDigit(display, slaveDigits[v % 10] | (slaveDP[display] & 0x01 ? 1 : 0));
}

void slaveSetDecimalPoint(unsigned char display, unsigned char l, unsigned char r)
{
    slaveDP[display] = (l ? 2 : 0) | (r ? 1 : 0);
    slaveSetLeftDigit(display, (slaveLeft & 0xFE) | (slaveDP[display] & 0x02 ? 1 : 0));
    slaveSetRightDigit(display, (slaveRight & 0xFE) | (slaveDP[display] & 0x01 ? 1 : 0));
}
