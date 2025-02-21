/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
//we added this code////////
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <xdc/cfg/global.h>//hosafty kdy sh semaphore t3vod
#include <stdio.h>
#include <string.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>
/* TI-RTOS Header files */
#include <ti/board/board.h>
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>
#include <stdint.h>
#include <ti/drv/gpio/test/led_blink/src/GPIO_log.h>//added the absolute path so it works
#include <ti/drv/gpio/test/led_blink/src/GPIO_board.h>
#include "Safe_code6.h"
#include <math.h>

#include "IIR_680_bandpass_4coeff.h"
#include "IIR_760_bandpass_4coeff.h"
#include "IIR_842_bandpass_4coeff.h"
#include "IIR_931_bandpass_4coeff.h"
#include "IIR_1200_bandpass_4coeff.h"
#include "IIR_1326_bandpass_4coeff.h"
#include "IIR_1467_bandpass_4coeff.h"
#include "IIR_1623_bandpass_4coeff.h"

#define INPUT_SIZE 4011
#define N_DTMF_LOW 4
#define N_DTMF_HIGH 4

/*
 =================================
 ========= FUNCTIONS =============
 =================================
 */
void F_iir_bandpass_680(int idx);
void F_iir_bandpass_760(int idx);
void F_iir_bandpass_842(int idx);
void F_iir_bandpass_931(int idx);
void F_iir_bandpass_1200(int idx);
void F_iir_bandpass_1326(int idx);
void F_iir_bandpass_1467(int idx);
void F_iir_bandpass_1623(int idx);
static void Board_initGPIO(void);
void calc_DTMF(int row, int col);
int max_value_idx(float arr[], int freq_type);
/*
 ============================================
 ========= VARIABLE DECLARATION =============
 ============================================
 */
//each array stores the corresponding filtered output according to sample index
float freq_output[4011];
float freq680_env[4011];
float freq760_env[4011];
float freq842_env[4011];
float freq931_env[4011];
float freq1200_env[4011];
float freq1326_env[4011];
float freq1467_env[4011];
float freq1623_env[4011];
//values used by the coeffs to do bandpass filtering and cascading
float d00[8], d01[8], d02[8], d00_1[8], d01_1[8], d02_1[8];
float xn[8], xn_1[8], y0[8], y0_1[8];
//stores max frequency value in the correct index where the index represent the frequency value
//e.g. idx 0 = 697hz ,idx 1 = 770hz....
float max_freq_value[8];

char string_arr[6];//string array that stores the "output" numbers after filtering

int t = 0;
int lowfreq_max = -1, highfreq_max = -1;
int func_type = 0;
int i = 0, j = 0;
int count = 0;
int high_idx = 0, low_idx = 0, id, indx = 0, idx = 0;
int window_size = 800; //iterate over window_size indexes at once inorder to match a pair of freqs.
int string_idx = 0; //string_arr index
int max_row = 0, max_col = 0;
float ALPHA = 0.03;  // Smoothing factor for envelope

// Mapping DTMF digits
const char dtmf_keys[4][4] = {
      { '1', '2', '3', 'A' },
      { '4', '5', '6', 'B' }, {
        '7', '8', '9', 'C' },
      { '*', '0', '#', 'D' }
};


/*
 *  ======== main ========
 */
Int main()
{
    //init
    Board_initGPIO();
    GPIO_init();

    //initLUT();
    BIOS_start(); /* does not return */
    return (0);
}
/*
 ============================================
 ========= FUNCTIONS DECLARATION=============
 ============================================
 *  ======== timerFunc ========
 *  This function runs every PERIOD ms in the context of a Hwi thread.
 */
//timer function gets called every 100 microseconds
void timer_func(UArg arg)  //hwi
{
    //call the flag for swi_LED
    Swi_post(swi1);
}

void swi1_func(void)
{
    //semaphore calls the task
    Semaphore_post(sem1);
}

void task1_func(void)
{
    while (1)
    {
        Semaphore_pend(sem1, BIOS_WAIT_FOREVER);
        switch (func_type)//switch case in order to debug each frequency individually
        {
        case 1:
            F_iir_bandpass_680(0);
            break;
        case 2:
            F_iir_bandpass_760(1);
            break;
        case 3:
            F_iir_bandpass_842(2);
            break;
        case 4:
            F_iir_bandpass_931(3);
            break;
        case 5:
            F_iir_bandpass_1200(4);
            break;
        case 6:
            F_iir_bandpass_1326(5);
            break;
        case 7:
            F_iir_bandpass_1467(6);
            break;
        case 8:
            F_iir_bandpass_1623(7);
            break;
        default:
            F_iir_bandpass_680(0);
            F_iir_bandpass_760(1);
            F_iir_bandpass_842(2);
            F_iir_bandpass_931(3);
            F_iir_bandpass_1200(4);
            F_iir_bandpass_1326(5);
            F_iir_bandpass_1467(6);
            F_iir_bandpass_1623(7);
            break;
        }
        if (t > 0 && (t % window_size == 0 || t == INPUT_SIZE - 1))
        {  //windows of 800
             max_row = max_value_idx(max_freq_value, 0);
             max_col = max_value_idx(max_freq_value, 4);
            calc_DTMF(max_row, max_col);
            for (i = 0; i < 8; i++)
            {
                max_freq_value[i] = 0;
            }
        }
        if (++t >= INPUT_SIZE)
        {
            t = 0;
            string_idx = 0;
//            for(i=0;i < 6;i++){
//                Log_print1(Diags_INFO,"Calculated number is: %c",string_arr[i]);
//            }
        }
    }
}
//match the pair of frequencies to the correct number dialed
void calc_DTMF(int row, int col)
{
    string_arr[string_idx] = dtmf_keys[row][col];
    //Log_print1(Diags_INFO,"Calculated number is: %c",string_arr[string_idx]);
    System_printf("Calculated number is: %c \n",string_arr[string_idx]);
    string_idx++;
}
//according to freq_type return the index of the highest/most dominant frequency in that type
int max_value_idx(float arr[], int freq_type)
{
    int max_idx = 0 + freq_type;
    for (i = 1 + freq_type; i < 4 + freq_type; i++)
    {
        if (arr[i] > arr[max_idx])
        {
            max_idx = i;
        }
    }
    return max_idx - freq_type;
}

void F_iir_bandpass_680(int idx)
{
//    float d00, d01, d02, d00_1, d01_1, d02_1;
//    float xn, xn_1, y0, y0_1;
    // Clean IIR every time the buffer is fulfilled
    if (t == 0)
    {
        d00[0] = 0;
        d01[0] = 0;
        d02[0] = 0;
        d00_1[0] = 0;
        d01_1[0] = 0;
        d02_1[0] = 0;
    }

    // Read the input sample from the signal buffer
    xn[0] = SC6[t];
    y0[0] = 0;

    // First stage of IIR bandpass
    d00[0] = xn[0] - (d02[0] * DEN_680[1][2]) - (d01[0] * DEN_680[1][1]);
    y0[0] = (d00[0] * NUM_680[1][0]) + (d01[0] * NUM_680[1][1]) + (d02[0] * NUM_680[1][2]);
    d02[0] = d01[0];
    d01[0] = d00[0];

    // Second stage of IIR bandpass
    xn_1[0] = y0[0] * NUM_680[0][0];
    d00_1[0] = xn_1[0] - (d02_1[0] * DEN_680[3][2]) - (d01_1[0] * DEN_680[3][1]);
    y0_1[0] = (d00_1[0] * NUM_680[3][0]) + (d01_1[0] * NUM_680[3][1])
            + (d02_1[0] * NUM_680[3][2]);
    d02_1[0] = d01_1[0];
    d01_1[0] = d00_1[0];

    float abs_y0 = fabs(y0_1[0] * NUM_680[0][0]); // Take absolute value
    freq_output[t] = y0_1[0] * NUM_680[0][0];
    if (t == 0)
    {
        freq680_env[t] = abs_y0; // Low-pass filtering
        max_freq_value[idx] = freq680_env[t];
    }
    else
    {
        freq680_env[t] = (ALPHA * abs_y0) + ((1 - ALPHA) * freq680_env[t - 1]); // Low-pass filtering
        max_freq_value[idx] = freq680_env[t] > max_freq_value[idx] ? freq680_env[t] : max_freq_value[idx];
    }
}

void F_iir_bandpass_760(int idx)
{
//    float d00, d01, d02, d00_1, d01_1, d02_1;
//    float xn, xn_1, y0, y0_1;
    // Clean IIR every time the buffer is fulfilled
    if (t == 0)
    {
        d00[1] = 0;
        d01[1] = 0;
        d02[1] = 0;
        d00_1[1] = 0;
        d01_1[1] = 0;
        d02_1[1] = 0;
    }

    // Read the input sample from the signal buffer
    xn[1] = SC6[t];
    y0[1] = 0;

    // First stage of IIR bandpass
    d00[1] = xn[1] - (d02[1] * DEN_760[1][2]) - (d01[1] * DEN_760[1][1]);
    y0[1] = (d00[1] * NUM_760[1][0]) + (d01[1] * NUM_760[1][1]) + (d02[1] * NUM_760[1][2]);
    d02[1] = d01[1];
    d01[1] = d00[1];

    // Second stage of IIR bandpass
    xn_1[1] = y0[1] * NUM_760[0][0];
    d00_1[1] = xn_1[1] - (d02_1[1] * DEN_760[3][2]) - (d01_1[1] * DEN_760[3][1]);
    y0_1[1] = (d00_1[1] * NUM_760[3][0]) + (d01_1[1] * NUM_760[3][1]) + (d02_1[1] * NUM_760[3][2]);
    d02_1[1] = d01_1[1];
    d01_1[1] = d00_1[1];

    float abs_y0 = fabs(y0_1[1] * NUM_760[0][0]); // Take absolute value
    freq_output[t] = y0_1[1] * NUM_760[0][0];
    if (t == 0)
    {
        freq760_env[t] = abs_y0; // Low-pass filtering
        max_freq_value[idx] = freq760_env[t];
    }
    else
    {
        freq760_env[t] = (ALPHA * abs_y0) + ((1 - ALPHA) * freq760_env[t - 1]); // Low-pass filtering
        max_freq_value[idx] = freq760_env[t] > max_freq_value[idx] ? freq760_env[t] : max_freq_value[idx];
    }
}

void F_iir_bandpass_842(int idx)
{
//    float d00, d01, d02, d00_1, d01_1, d02_1;
//    float xn, xn_1, y0, y0_1;
// Clean IIR every time the buffer is fulfilled
    if (t == 0)
    {
        d00[2] = 0;
        d01[2] = 0;
        d02[2] = 0;
        d00_1[2] = 0;
        d01_1[2] = 0;
        d02_1[2] = 0;
    }

// Read the input sample from the signal buffer
    xn[2] = SC6[t];
    y0[2] = 0;

// First stage of IIR bandpass
    d00[2] = xn[2] - (d02[2] * DEN_842[1][2]) - (d01[2] * DEN_842[1][1]);
    y0[2] = (d00[2] * NUM_842[1][0]) + (d01[2] * NUM_842[1][1]) + (d02[2] * NUM_842[1][2]);
    d02[2] = d01[2];
    d01[2] = d00[2];

// Second stage of IIR bandpass
    xn_1[2] = y0[2] * NUM_842[0][0];
    d00_1[2] = xn_1[2] - (d02_1[2] * DEN_842[3][2]) - (d01_1[2] * DEN_842[3][1]);
    y0_1[2] = (d00_1[2] * NUM_842[3][0]) + (d01_1[2] * NUM_842[3][1])
            + (d02_1[2] * NUM_842[3][2]);
    d02_1[2] = d01_1[2];
    d01_1[2] = d00_1[2];

    float abs_y0 = fabs(y0_1[2] * NUM_842[0][0]); // Take absolute value
    freq_output[t] = y0_1[2] * NUM_842[0][0];
    if (t == 0)
    {
        freq842_env[t] = abs_y0; // Low-pass filtering
        max_freq_value[idx] = freq842_env[t];
    }
    else
    {
        freq842_env[t] = (ALPHA * abs_y0) + ((1 - ALPHA) * freq842_env[t - 1]); // Low-pass filtering
        max_freq_value[idx] = freq842_env[t] > max_freq_value[idx] ? freq842_env[t] : max_freq_value[idx];
    }
}

void F_iir_bandpass_931(int idx)
{
//    float d00, d01, d02, d00_1, d01_1, d02_1;
//    float xn, xn_1, y0, y0_1;
    // Clean IIR every time the buffer is fulfilled
    if (t == 0)
    {
        d00[3] = 0;
        d01[3] = 0;
        d02[3] = 0;
        d00_1[3] = 0;
        d01_1[3] = 0;
        d02_1[3] = 0;
    }

    // Read the input sample from the signal buffer
    xn[3] = SC6[t];
    y0[3] = 0;

    // First stage of IIR bandpass
    d00[3] = xn[3] - (d02[3] * DEN_931[1][2]) - (d01[3] * DEN_931[1][1]);
    y0[3] = (d00[3] * NUM_931[1][0]) + (d01[3] * NUM_931[1][1]) + (d02[3] * NUM_931[1][2]);
    d02[3] = d01[3];
    d01[3] = d00[3];

    // Second stage of IIR bandpass
    xn_1[3] = y0[3] * NUM_931[0][0];
    d00_1[3] = xn_1[3] - (d02_1[3] * DEN_931[3][2]) - (d01_1[3] * DEN_931[3][1]);
    y0_1[3] = (d00_1[3] * NUM_931[3][0]) + (d01_1[3] * NUM_931[3][1]) + (d02_1[3] * NUM_931[3][2]);
    d02_1[3] = d01_1[3];
    d01_1[3] = d00_1[3];

    float abs_y0 = fabs(y0_1[3] * NUM_931[0][0]); // Take absolute value
    freq_output[t] = y0_1[3] * NUM_931[0][0];
    if (t == 0)
    {
        freq931_env[t] = abs_y0; // Low-pass filtering
        max_freq_value[idx] = freq931_env[t];
    }
    else
    {
        freq931_env[t] = (ALPHA * abs_y0) + ((1 - ALPHA) * freq931_env[t - 1]); // Low-pass filtering
        max_freq_value[idx] = freq931_env[t] > max_freq_value[idx] ? freq931_env[t] : max_freq_value[idx];
    }
}

void F_iir_bandpass_1200(int idx)
{
//    float d00, d01, d02, d00_1, d01_1, d02_1;
//    float xn, xn_1, y0, y0_1;
    // Clean IIR every time the buffer is fulfilled
    if (t == 0)
    {
        d00[4] = 0;
        d01[4] = 0;
        d02[4] = 0;
        d00_1[4] = 0;
        d01_1[4] = 0;
        d02_1[4] = 0;
    }

    // Read the input sample from the signal buffer
    xn[4] = SC6[t];
    y0[4] = 0;

    // First stage of IIR bandpass
    d00[4] = xn[4] - (d02[4] * DEN_1200[1][2]) - (d01[4] * DEN_1200[1][1]);
    y0[4] = (d00[4] * NUM_1200[1][0]) + (d01[4] * NUM_1200[1][1])
            + (d02[4] * NUM_1200[1][2]);
    d02[4] = d01[4];
    d01[4] = d00[4];

    // Second stage of IIR bandpass
    xn_1[4] = y0[4] * NUM_1200[0][0];
    d00_1[4] = xn_1[4] - (d02_1[4] * DEN_1200[3][2]) - (d01_1[4] * DEN_1200[3][1]);
    y0_1[4] = (d00_1[4] * NUM_1200[3][0]) + (d01_1[4] * NUM_1200[3][1]) + (d02_1[4] * NUM_1200[3][2]);
    d02_1[4] = d01_1[4];
    d01_1[4] = d00_1[4];

    float abs_y0 = fabs(y0_1[4] * NUM_1200[0][0]); // Take absolute value
    freq_output[t] = y0_1[4] * NUM_1200[0][0];
    if (t == 0)
    {
        freq1200_env[t] = abs_y0; // Low-pass filtering
        max_freq_value[idx] = freq1200_env[t];
    }
    else
    {
        freq1200_env[t] = (ALPHA * abs_y0) + ((1 - ALPHA) * freq1200_env[t - 1]); // Low-pass filtering
        max_freq_value[idx] = freq1200_env[t] > max_freq_value[idx] ? freq1200_env[t] : max_freq_value[idx];
    }
}

void F_iir_bandpass_1326(int idx)
{
//    float d00, d01, d02, d00_1, d01_1, d02_1;
//    float xn, xn_1, y0, y0_1;
    // Clean IIR every time the buffer is fulfilled
    if (t == 0)
    {
        d00[5] = 0;
        d01[5] = 0;
        d02[5] = 0;
        d00_1[5] = 0;
        d01_1[5] = 0;
        d02_1[5] = 0;
    }

    // Read the input sample from the signal buffer
    xn[5] = SC6[t];
    y0[5] = 0;

    // First stage of IIR bandpass
    d00[5] = xn[5] - (d02[5] * DEN_1326[1][2]) - (d01[5] * DEN_1326[1][1]);
    y0[5] = (d00[5] * NUM_1326[1][0]) + (d01[5] * NUM_1326[1][1])
            + (d02[5] * NUM_1326[1][2]);
    d02[5] = d01[5];
    d01[5] = d00[5];

    // Second stage of IIR bandpass
    xn_1[5] = y0[5] * NUM_1326[0][0];
    d00_1[5] = xn_1[5] - (d02_1[5] * DEN_1326[3][2]) - (d01_1[5] * DEN_1326[3][1]);
    y0_1[5] = (d00_1[5] * NUM_1326[3][0]) + (d01_1[5] * NUM_1326[3][1]) + (d02_1[5] * NUM_1326[3][2]);
    d02_1[5] = d01_1[5];
    d01_1[5] = d00_1[5];

    float abs_y0 = fabs(y0_1[5] * NUM_1326[0][0]); // Take absolute value
    freq_output[t] = y0_1[5] * NUM_1326[0][0];
    if (t == 0)
    {
        freq1326_env[t] = abs_y0; // Low-pass filtering
        max_freq_value[idx] = freq1326_env[t];
    }
    else
    {
        freq1326_env[t] = (ALPHA * abs_y0) + ((1 - ALPHA) * freq1326_env[t - 1]); // Low-pass filtering
        max_freq_value[idx] = freq1326_env[t] > max_freq_value[idx] ? freq1326_env[t] : max_freq_value[idx];
    }
}

void F_iir_bandpass_1467(int idx)
{
//    float d00, d01, d02, d00_1, d01_1, d02_1;
//    float xn, xn_1, y0, y0_1;
    // Clean IIR every time the buffer is fulfilled
    if (t == 0)
    {
        d00[6] = 0;
        d01[6] = 0;
        d02[6] = 0;
        d00_1[6] = 0;
        d01_1[6] = 0;
        d02_1[6] = 0;
    }

    // Read the input sample from the signal buffer
    xn[6] = SC6[t];
    y0[6] = 0;

    // First stage of IIR bandpass
    d00[6] = xn[6] - (d02[6] * DEN_1467[1][2]) - (d01[6] * DEN_1467[1][1]);
    y0[6] = (d00[6] * NUM_1467[1][0]) + (d01[6] * NUM_1467[1][1]) + (d02[6] * NUM_1467[1][2]);
    d02[6] = d01[6];
    d01[6] = d00[6];

    // Second stage of IIR bandpass
    xn_1[6] = y0[6] * NUM_1467[0][0];
    d00_1[6] = xn_1[6] - (d02_1[6] * DEN_1467[3][2]) - (d01_1[6] * DEN_1467[3][1]);
    y0_1[6] = (d00_1[6] * NUM_1467[3][0]) + (d01_1[6] * NUM_1467[3][1]) + (d02_1[6] * NUM_1467[3][2]);
    d02_1[6] = d01_1[6];
    d01_1[6] = d00_1[6];

    float abs_y0 = fabs(y0_1[6] * NUM_1467[0][0]); // Take absolute value
    freq_output[t] = y0_1[6] * NUM_1467[0][0];
    if (t == 0)
    {
        freq1467_env[t] = abs_y0; // Low-pass filtering
        max_freq_value[idx] = freq1467_env[t];
    }
    else
    {
        freq1467_env[t] = (ALPHA * abs_y0) + ((1 - ALPHA) * freq1467_env[t - 1]); // Low-pass filtering
        max_freq_value[idx] = freq1467_env[t] > max_freq_value[idx] ? freq1467_env[t] : max_freq_value[idx];
    }
}

void F_iir_bandpass_1623(int idx)
{
//    float d00, d01, d02, d00_1, d01_1, d02_1;
//    float xn, xn_1, y0, y0_1;
    // Clean IIR every time the buffer is fulfilled
    if (t == 0)
    {
        d00[7] = 0;
        d01[7] = 0;
        d02[7] = 0;
        d00_1[7] = 0;
        d01_1[7] = 0;
        d02_1[7] = 0;
    }

    // Read the input sample from the signal buffer
    xn[7] = SC6[t];
    y0[7] = 0;

    // First stage of IIR bandpass
    d00[7] = xn[7] - (d02[7] * DEN_1623[1][2]) - (d01[7] * DEN_1623[1][1]);
    y0[7] = (d00[7] * NUM_1623[1][0]) + (d01[7] * NUM_1623[1][1]) + (d02[7] * NUM_1623[1][2]);
    d02[7] = d01[7];
    d01[7] = d00[7];

    // Second stage of IIR bandpass
    xn_1[7] = y0[7] * NUM_1623[0][0];
    d00_1[7] = xn_1[7] - (d02_1[7] * DEN_1623[3][2]) - (d01_1[7] * DEN_1623[3][1]);
    y0_1[7] = (d00_1[7] * NUM_1623[3][0]) + (d01_1[7] * NUM_1623[3][1]) + (d02_1[7] * NUM_1623[3][2]);
    d02_1[7] = d01_1[7];
    d01_1 [7]= d00_1[7];

    float abs_y0 = fabs(y0_1[7] * NUM_1623[0][0]); // Take absolute value
    freq_output[t] = y0_1[7] * NUM_1623[0][0];
    if (t == 0)
    {
        freq1623_env[t] = abs_y0; // Low-pass filtering
        max_freq_value[idx] = freq1623_env[t];
    }
    else
    {
        freq1623_env[t] = (ALPHA * abs_y0) + ((1 - ALPHA) * freq1623_env[t - 1]); // Low-pass filtering
        max_freq_value[idx] = freq1623_env[t] > max_freq_value[idx] ? freq1623_env[t] : max_freq_value[idx];
    }
}

static void Board_initGPIO(void)
{
    Board_initCfg boardCfg;

#if defined(SOC_OMAPL138)
    GPIO_v0_HwAttrs gpio_cfg;

    /* Get the default SPI init configurations */
    GPIO_socGetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);

    /* Setup GPIO interrupt configurations */
    GPIO_socSetBankInt(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL);

//        boardCfg = BOARD_INIT_PINMUX_CONFIG |
//            BOARD_INIT_MODULE_CLOCK |
//            BOARD_INIT_UART_STDIO;
    boardCfg = BOARD_INIT_PINMUX_CONFIG;
    Board_init(boardCfg);
#endif
}
