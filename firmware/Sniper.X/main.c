#include <p32xxxx.h>
#include <plib.h>

#include "core.h"
#include "sounds/pr.h"
#include "sounds/click.h"
#include "sounds/glfire2.h"
#include "sounds/sniper1.h"
#include "sounds/sniper2.h"
#include "sounds/sniper3.h"
#include "sounds/reload_ins.h"
#include "sounds/reload_rem.h"
#include "sounds/sniper.h"
#include "sounds/ttsauto.h"

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

#ifndef PR_BEND_START
#define PR_BEND_START 3
#endif

#ifndef PR_BEND_LEN
#define PR_BEND_LEN 6
#endif

#ifndef PR_BEND_SPEED
#define PR_BEND_SPEED 1
#endif

#define DISP_E 0b11011100

#define FM_AUTO 0
#define FM_SNIPER 1

// A speed modification value for pulse rifle sample playback
volatile int pr_speed = 0;
volatile int pr_original_speed = 0;

// Number of ammo in the gun
volatile int pr_ammo = PR_LOAD;
volatile int gl_ammo = GRENADES;
volatile int fire_mode = FM_AUTO;

// Number of shots fired in this salvo
volatile int shots_fired = 0;

// Muzzle flash PWM duty
volatile int mf = 0;
volatile int gf = 0;

// The current display mode.  0 = PR, 1 = GL
#define DISP_PR 0
#define DISP_GL 1
volatile int display_mode = 0;
volatile int display_info = 0;

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

// The autofire system calls this routine for each bullet that is
// fired.  Playback speed is modified by the value decided upon in the
// pr_init routine when the trigger was pulled.
void pr_fire(int in, int v)
{
    // If we are reloading, don't do anything.
    if (pr_ammo > PR_LOAD) {
        return;
    }

    if (fire_mode == FM_SNIPER && shots_fired >= 1) {
        return;
    }

    // If there is no ammo, play the click, otherwise play the
    // firing sample
    if (pr_ammo == 0) {
        queueSample(click, click_len, 0, 0);
    } else {
        if (fire_mode == FM_AUTO) {
            queueSample(pr, pr_len, 0, pr_speed);
        } else {
            switch (rand() % 3) {
                case 0:
                    queueSample(sniper1, sniper1_len, 0, 0);
                    break;
                case 1:
                    queueSample(sniper2, sniper2_len, 0, 0);
                    break;
                case 2:
                    queueSample(sniper3, sniper3_len, 0, 0);
                    break;
            }
        }
        shots_fired++;
        pr_ammo--;
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
        pr_ammo = PR_MAX;
        gl_ammo = GRENADES;
        queueSingleSample(reload_ins, reload_ins_len, 0, 0);
    } else {
        queueSingleSample(reload_rem, reload_rem_len, 0, 0);
        pr_ammo = 0;
        gl_ammo = 0;
    }
}

void reloadCount()
{
    if (pr_ammo <= PR_LOAD) {
        return;
    }

    pr_ammo--;
}

// Fire a grenade.  Immediately play a "thunk" firing sound, and queue
// a delayed "boom" exploding sound after a random amount of time.
void gl_fire(int in, int v)
{
    if (v == 0) {
        display_mode = DISP_GL;
        if (gl_ammo > 0) {
            queueSample(glfire2, glfire2_len, 0, 0);
            gf = 255;
            gl_ammo--;
        }
    }
}

void gl_load(int in, int v)
{
    display_mode = DISP_GL;
    if (v == 0) {
        if (gl_ammo < GRENADES) {
            gl_ammo++;
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

// A callback function to keep the display updated right.  This is the only function that
// will ever do any displaying of numbers.
void update_display()
{
    static int del = 0;
    static int display_extra = 0;

    if (display_info > 0) {
        display_extra = display_info;
        display_info = 0;
        del = 50;
        return;
    }


    if (del > 0) {
        del--;
        displayRaw((display_extra >> 8) & 0xFF, display_extra & 0xFF);
        return;
    }

    
    // Is the magazine inserted?
    if (readInput(1) == 1) {
        displayRaw(DISP_E, DISP_E);
        return;
    }
    if (display_mode == DISP_PR) {
        displayValue(pr_ammo);
    } else {
        displayValue(gl_ammo);
    }
}

void set_fire_mode(int i, int v)
{
    if (v == 0) {
        fire_mode = 1 - fire_mode;
    }

    switch (fire_mode) {
        case FM_AUTO:
            queueSingleSample(ttsauto, ttsauto_len, 0, 0);
            display_info = 0b0101111111000001;
            break;
        case FM_SNIPER:
            queueSingleSample(sniper, sniper_len, 0, 0);
            display_info = 0b1001110101001001;
            break;
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

    registerCallback(update_display, 10);

    setAnalog(0);

    // Set IO channel 0 to input mode and tie it to the pr_init function
    setInput(IO0);
    setDebounce(IO5, 1000);
    tieInput(IO0, pr_init);

    initPWM(LED0);
    initPWM(LED1);
    registerCallback(muzzle_flash, 1);

    // Configure auto-fire on IO channel 0 at 20 rounds per second.  Tie
    // it to the pr_fire function
    initAutofire(IO0, PR_FIRE_RATE, pr_fire);

    // Set IO channel 1 to input and tie it to reload_mag function
    setInput(IO1);
    setDebounce(IO1, 1000);
    tieInput(IO1, reload_mag);
    registerCallback(reloadCount, 100);

    // Set IO channel 2 to input and tie it to the gl_fire function
    setInput(IO2);
    setDebounce(IO2, 1000);
    tieInput(IO2, gl_fire);

    setInput(IO3);
    setDebounce(IO3, 1000);
    tieInput(IO3, gl_load);

    setInput(IO4);
    tieInput(IO4, set_fire_mode);

    // Force a reload the magazine
    reload_mag(IO1, readInput(IO1));

    while (1);
}

