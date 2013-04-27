#include <p32xxxx.h>
#include <plib.h>

#include "core.h"
#ifndef PR_SOUND_DISABLED
#include "sounds/pr.h"
#endif

#ifndef CLICK_SOUND_DISABLED
#include "sounds/click.h"
#endif

#ifndef GLFIRE_SOUND_DISABLED
#include "sounds/glfire2.h"
#endif

#ifndef GLBOOM_SOUND_DISABLED
#include "sounds/glboom.h"
#endif

#ifndef INS_SOUND_ENABLED
#include "sounds/reload_ins.h"
#endif

#ifndef REM_SOUND_DISABLED
#include "sounds/reload_rem.h"
#endif

#ifndef PUMPA_SOUND_DISABLED
#include "sounds/pumpa.h"
#endif

#ifndef PUMPB_SOUND_DISABLED
#include "sounds/pumpb.h"
#endif

#ifndef PUMPA_EMPTY_SOUND_DISABLED
#include "sounds/pump_emptya.h"
#endif 

#ifndef PUMPB_EMPTY_SOUND_DISABLED
#include "sounds/pump_emptyb.h"
#endif 

// Default settings - these can be over-written with -D directives in
// the Makefile.

#ifndef GRENADES
#define GRENADES 5
#endif

#ifndef PR_FIRE_RATE
#define PR_FIRE_RATE 20
#endif

#ifndef PR_MAX
#define PR_MAX 99
#endif

#ifndef PR_LOAD
#define PR_LOAD 95
#endif

#ifndef PR_BURST
#define PR_BURST 3
#endif

#ifndef PR_BEND_START
#define PR_BEND_START 3
#endif

#ifndef PR_BEND_LEN
#define PR_BEND_LEN 6
#endif

#ifndef PR_BEND_SPEED
#define PR_BEND_SPEED 1
#endif

#define DISP_E 0b00001000
#define sDISP_E 0b10000000

#define FM_SINGLE 0
#define FM_BURST 1
#define FM_AUTO 2

#ifndef B_PR_TRIGGER
#define B_PR_TRIGGER IO0
#endif

#ifndef B_MAGAZINE
#define B_MAGAZINE IO1
#endif

#ifndef B_GL_TRIGGER
#define B_GL_TRIGGER IO2
#endif

#ifndef B_GL_PRIME
#define B_GL_PRIME IO3
#endif

// A speed modification value for pulse rifle sample playback
volatile int pr_speed = 0;
volatile int pr_original_speed = 0;

// Number of ammo in the gun
volatile int pr_ammo = PR_LOAD;
volatile int gl_ammo = GRENADES;
volatile int gl_primed = 0;
volatile int fire_mode = FM_BURST;

// Number of shots fired in this salvo
volatile int shots_fired = 0;

// Muzzle flash PWM duty
volatile int mf = 0;
volatile int gf = 0;

// The current display mode.  0 = PR, 1 = GL
#define DISP_PR 0
#define DISP_GL 1

#define DISPLAY_TIMEOUT 300

volatile int display_mode = 0;
volatile int display_info = 0;

volatile int display_timeout = 0;

// This is tied to the initial pulling of the trigger.
// A random playback speed is selected for the pulse rifle
// firing sample.
void pr_init(int in, int v)
{
    if (v == 0) {
        pr_speed = (rand() % 32) - 8;
        pr_original_speed = pr_speed;
        shots_fired = 0;
        display_mode = DISP_PR;
    }
}

void set_pr_ammo(unsigned char ammo)
{
    pr_ammo = ammo;
    displayValue(pr_ammo);
#ifdef SLAVE_ENABLED
    slaveSetValue(0, pr_ammo);    
#endif
}

void set_gl_ammo(unsigned char ammo)
{
    gl_ammo = ammo;
    displayValue(gl_ammo);
#ifdef SLAVE_ENABLED
    slaveSetValue(0, gl_ammo);    
#endif
    display_timeout = DISPLAY_TIMEOUT;
}

// The autofire system calls this routine for each bullet that is
// fired.  Playback speed is modified by the value decided upon in the
// pr_init routine when the trigger was pulled.
void pr_fire(int in, int v)
{
    // If we are reloading, don't do anything.
    if (pr_ammo > PR_LOAD) {
        return;
    }

    if (fire_mode == FM_SINGLE && shots_fired >= 1) {
        return;
    }

    if (fire_mode == FM_BURST && shots_fired >= PR_BURST) {
        return;
    }

    // If there is no ammo, play the click, otherwise play the
    // firing sample
    if (pr_ammo == 0) {
#ifndef CLICK_SOUND_DISABLED
        queueSample(click, click_len, 0, 0);
#endif
    } else {
#ifndef PR_SOUND_DISABLED
        queueSample(pr, pr_len, 0, pr_speed);
#endif
        shots_fired++;
        set_pr_ammo(pr_ammo - 1);
        if ((shots_fired >= PR_BEND_START) && (shots_fired < PR_BEND_START+PR_BEND_LEN)) {
            if (pr_original_speed >= -4) {
                pr_speed -= PR_BEND_SPEED;
            } else {
                pr_speed += PR_BEND_SPEED;
            }
        }
        mf = 255;
    }
}

// Reload the magazine.  Reset all ammo values to their
// maximum values.
void reload_mag(int in, int v)
{
    unsigned long l;
    display_mode = DISP_PR;
    if (v == 0) {
#ifndef INS_SOUND_DISABLED
        queueSample(reload_ins, reload_ins_len, 0, 0);
#endif
        set_pr_ammo(PR_MAX);
#ifdef GL_AMMO_IN_MAGAZINE
        set_gl_ammo(GRENADES);
#endif
    } else {
#ifndef REM_SOUND_DISABLED
        queueSample(reload_rem, reload_rem_len, 0, 0);
#endif
        set_pr_ammo(0);
#ifdef GL_AMMO_IN_MAGAZINE
        set_gl_ammo(0);
        display_timeout = 0;
#endif
        displayRaw(DISP_E, DISP_E);
#ifdef SLAVE_ENABLED
        slaveSetLeftDigit(0, sDISP_E);
        slaveSetRightDigit(0, sDISP_E);
#endif
    }
}

void reloadCount()
{
    if (pr_ammo <= PR_LOAD) {
        return;
    }

    set_pr_ammo(pr_ammo - 1);
}

// Fire a grenade.  Immediately play a "thunk" firing sound, and queue
// a delayed "boom" exploding sound after a random amount of time.
void gl_fire(int in, int v)
{
    if (v == 0) {
        display_mode = DISP_GL;
        if (gl_primed == 0) {
#ifndef CLICK_SOUND_DISABLED
            queueSample(click, click_len, 0, 0);
#endif
            return;
        }

        gl_primed = 0;
        setDecimalPoint(0, 0);
#ifdef SLAVE_ENABLED
        slaveSetDecimalPoint(0, 0, 0);
#endif

#ifndef GLFIRE_SOUND_DISABLED
        queueSample(glfire2, glfire2_len, 0, 0);
#endif
#ifndef GLBOOM_SOUND_DISABLED
        queueSample(glboom, glboom_len, 2000 + (rand() % 10000), 0);
#endif
        gf = 255;
    }
}

void gl_prime(int in, int v)
{
    display_mode = DISP_GL;
    if (v == 0) {
        if ((gl_ammo > 0) || (gl_primed == 1)) {
#ifndef PUMPA_SOUND_DISABLED
            queueSample(pumpa, pumpa_len, 0, 0);    
#endif
        } else {
#ifndef PUMPA_EMPTY_SOUND_DISABLED
            queueSample(pump_emptya, pump_emptya_len, 0, 0);    
#endif
        }
        gl_primed = 0;
        setDecimalPoint(0, 0);
#ifdef SLAVE_ENABLED
        slaveSetDecimalPoint(0, 0, 0);
#endif
    } else {
        if (gl_ammo > 0) {
            gl_primed = 1;
            setDecimalPoint(1, 0);
#ifdef SLAVE_ENABLED
            slaveSetDecimalPoint(0, 1, 0);
#endif
            set_gl_ammo(gl_ammo - 1);
#ifndef PUMPB_SOUND_DISABLED
            queueSample(pumpb, pumpb_len, 0, 0);    
#endif
        } else {
            gl_primed = 0;
            setDecimalPoint(0, 0);
#ifdef SLAVE_ENABLED
            slaveSetDecimalPoint(0, 0, 0);
#endif
#ifndef PUMPB_EMPTY_SOUND_DISABLED
            queueSample(pump_emptyb, pump_emptyb_len, 0, 0);    
#endif
        }
    }
}

void muzzle_flash()
{
    if (mf > 0) {
        mf -= 8;
        if (mf < 0) {
            mf = 0;
        }
        setPWM(LED0, mf);
    }

    if (gf > 0) {
        gf--;
        setPWM(LED1, gf);
    }
}

void set_fire_mode(int i, int v)
{
    int i1, i2;
    i1 = readInput(4);
    i2 = readInput(5);

    fire_mode = ((i1 << 1) | i2) - 1;
    switch (fire_mode) {
        case FM_AUTO:
            display_info = 0b0101111111000001;
            break;
        case FM_BURST:
            display_info = 0b1101100011000001;
            break;
        case FM_SINGLE:
            display_info = 0b1001110100000001;
            break;
    }
}

void switch_display()
{
    if (display_timeout > 0) {
        display_timeout --;
        if (display_timeout == 0) {
            set_pr_ammo(pr_ammo);
        }
    }
}

// This is the main initialization routine.
int main(void) 
{
    // Initialize the core functions and ticks
    initCore();

    // Start up audio playback at 11025 Hz sample rate
    initPlayback(11025);

    // Enable the LED display
    initDisplay();

//    registerCallback(update_display, 50);
    registerCallback(switch_display, 10);

    setAnalog(0);

#ifdef SLAVE_ENABLED
    initSlave(0);
#endif

    // Set IO channel 0 to input mode and tie it to the pr_init function
    setInput(B_PR_TRIGGER);
    setDebounce(B_PR_TRIGGER, 1000);
    tieInput(B_PR_TRIGGER, pr_init);

    initPWM(LED0);
    initPWM(LED1);
    registerCallback(muzzle_flash, 1);

    // Configure auto-fire on IO channel 0 at 20 rounds per second.  Tie
    // it to the pr_fire function
    initAutofire(B_PR_TRIGGER, PR_FIRE_RATE, pr_fire);

    // Set IO channel 1 to input and tie it to reload_mag function
    setInput(B_MAGAZINE);
    setDebounce(B_MAGAZINE, 1000);
    tieInput(B_MAGAZINE, reload_mag);
    registerCallback(reloadCount, 100);

    // Set IO channel 2 to input and tie it to the gl_fire function
    setInput(B_GL_TRIGGER);
    setDebounce(B_GL_TRIGGER, 1000);
    tieInput(B_GL_TRIGGER, gl_fire);

    setInput(B_GL_PRIME);
    setDebounce(B_GL_PRIME, 1000);
    tieInput(B_GL_PRIME, gl_prime);

    setInput(IO4);
    setInput(IO5);
    tieInput(IO4, set_fire_mode);
    tieInput(IO5, set_fire_mode);

    gl_ammo = GRENADES;
    set_pr_ammo(PR_LOAD);
    
    // Force a reload the magazine
    reload_mag(IO1, readInput(IO1));

    set_fire_mode(0,0);

    while (1);
}

