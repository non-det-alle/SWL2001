/*!
 * \file      smtc_hal_rtc.c
 *
 * \brief     RTC Hardware Abstraction Layer implementation
 *
 * MIT License
 *
 * Copyright (c) 2024 Alessandro Aimi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <time.h>
#include "smtc_hal_rtc.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

// clang-format off

// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static struct timespec hal_rtc_starttime;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_rtc_init(void)
{
    clock_gettime(CLOCK_REALTIME, &hal_rtc_starttime);
}

uint32_t hal_rtc_get_time_s(void)
{
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);

    return now.tv_sec - hal_rtc_starttime.tv_sec - (now.tv_nsec < hal_rtc_starttime.tv_nsec);
}

uint32_t hal_rtc_get_time_100us(void)
{
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);

    return (now.tv_sec - hal_rtc_starttime.tv_sec) * 1e4 + (now.tv_nsec - hal_rtc_starttime.tv_nsec) / 1e5 + .5;
}
uint32_t hal_rtc_get_time_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);

    return (now.tv_sec - hal_rtc_starttime.tv_sec) * 1e3 + (now.tv_nsec - hal_rtc_starttime.tv_nsec) / 1e6 + .5;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
