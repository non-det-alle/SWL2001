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
#include <signal.h>
#include "smtc_hal_rtc.h"

#include "smtc_hal_mcu.h"

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

static timer_t hal_rtc_tid;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

void hal_rtc_wakeup_timer_handler(int sig, siginfo_t *si, void *uc);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_rtc_init(void)
{
    clock_gettime(RT_CLOCK, &hal_rtc_starttime);

    struct sigevent sev;
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    if (timer_create(RT_CLOCK, &sev, &hal_rtc_tid) == -1)
    {
        mcu_panic();
    }

    struct sigaction sa;
    sa.sa_sigaction = hal_rtc_wakeup_timer_handler;
    sigemptyset(&sa.sa_mask); // sigs to be blocked during handler execution
    sa.sa_flags = SA_SIGINFO; // | SA_RESTART;
    if (sigaction(SIGRTMIN, &sa, NULL) == -1)
    {
        mcu_panic();
    }
}

void hal_rtc_de_init(void)
{
    if (timer_delete(hal_rtc_tid) == -1)
    {
        // no reset to avoid error-looping
        mcu_panic_trace();
    }
}

uint32_t hal_rtc_get_time_s(void)
{
    struct timespec now;
    clock_gettime(RT_CLOCK, &now);

    return now.tv_sec - hal_rtc_starttime.tv_sec - (now.tv_nsec < hal_rtc_starttime.tv_nsec);
}

uint32_t hal_rtc_get_time_100us(void)
{
    struct timespec now;
    clock_gettime(RT_CLOCK, &now);

    return (now.tv_sec - hal_rtc_starttime.tv_sec) * 1e4 + (now.tv_nsec - hal_rtc_starttime.tv_nsec) / 1e5 + .5;
}
uint32_t hal_rtc_get_time_ms(void)
{
    struct timespec now;
    clock_gettime(RT_CLOCK, &now);

    return (now.tv_sec - hal_rtc_starttime.tv_sec) * 1e3 + (now.tv_nsec - hal_rtc_starttime.tv_nsec) / 1e6 + .5;
}

void hal_rtc_wakeup_timer_set_ms(const int32_t milliseconds)
{
    struct itimerspec its;
    its.it_value.tv_sec = milliseconds / 1000;
    its.it_value.tv_nsec = milliseconds % 1000 * 1000000;
    its.it_interval = ZERO;
    if (timer_settime(hal_rtc_tid, 0, &its, NULL) == -1)
    {
        mcu_panic();
    }
}

void hal_rtc_wakeup_timer_stop(void)
{
    struct itimerspec its;
    its.it_value = ZERO;
    its.it_interval = ZERO;
    if (timer_settime(hal_rtc_tid, 0, &its, NULL) == -1)
    {
        mcu_panic();
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void hal_rtc_wakeup_timer_handler(int sig, siginfo_t *si, void *uc)
{
    hal_mcu_wakeup();
}

/* --- EOF ------------------------------------------------------------------ */
