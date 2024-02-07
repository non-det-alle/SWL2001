/*!
 * \file      smtc_hal_lp_timer.c
 *
 * \brief     Implements Low Power Timer utilities functions.
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

#include <stdint.h>  // C99 types
#include <stdbool.h> // bool type

#include "smtc_hal_lp_timer.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_rtc.h"

#include <time.h>
#include <signal.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define HAL_LP_TIMER_NB 2 //!< Number of supported low power timers

#define ZERO ((struct timespec){.tv_sec = 0, .tv_nsec = 0})

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static int lptim_signo[HAL_LP_TIMER_NB];

static timer_t lptim_handle[HAL_LP_TIMER_NB];

static hal_lp_timer_irq_t lptim_tmr_irq[HAL_LP_TIMER_NB] = {
    {
        .context = NULL,
        .callback = NULL,
    },
#if (HAL_LP_TIMER_NB > 1)
    {
        .context = NULL,
        .callback = NULL,
    },
#endif
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

void hal_pl_timer_handler(int sig, siginfo_t *si, void *uc);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_lp_timer_init(hal_lp_timer_id_t id)
{
    int signo = SIGRTMIN + id;
    if (signo > SIGRTMAX)
    {
        mcu_panic();
    }
    lptim_signo[id] = signo;

    struct sigevent sev;
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = signo;
    sev.sigev_value.sival_int = id; // pass id here
    if (timer_create(RT_CLOCK, &sev, &lptim_handle[id]) == -1)
    {
        mcu_panic();
    }

    hal_lp_timer_irq_enable(id); // establish handler for callback
    lptim_tmr_irq[id] = (hal_lp_timer_irq_t){.context = NULL, .callback = NULL};
}

void hal_lp_timer_de_init(hal_lp_timer_id_t id)
{
    hal_lp_timer_irq_disable(id);
    if (timer_delete(lptim_handle[id]) == -1)
    {
        // no panic to avoid error-looping
        exit(-2);
    }
}

void hal_lp_timer_start(hal_lp_timer_id_t id, const uint32_t milliseconds, const hal_lp_timer_irq_t *tmr_irq)
{
    struct itimerspec its;
    its.it_value.tv_sec = milliseconds / 1000;
    its.it_value.tv_nsec = milliseconds % 1000 * 1000000;
    its.it_interval = ZERO;
    if (timer_settime(lptim_handle[id], 0, &its, NULL) == -1)
    {
        mcu_panic();
    }

    lptim_tmr_irq[id] = *tmr_irq; // callback assignment
}

void hal_lp_timer_stop(hal_lp_timer_id_t id)
{
    struct itimerspec its;
    its.it_value = ZERO;
    if (timer_settime(lptim_handle[id], 0, &its, NULL) == -1)
    {
        mcu_panic();
    }

    lptim_tmr_irq[id] = (hal_lp_timer_irq_t){.context = NULL, .callback = NULL};
}

void hal_lp_timer_irq_enable(hal_lp_timer_id_t id)
{
    /* Implemented by changing sigaction to execute handler */
    struct sigaction sa;
    sa.sa_sigaction = hal_pl_timer_handler;
    sigemptyset(&sa.sa_mask); // sigs to be blocked during handler execution
    sa.sa_flags = SA_SIGINFO; // | SA_RESTART;
    if (sigaction(lptim_signo[id], &sa, NULL) == -1)
    {
        mcu_panic();
    }
}

void hal_lp_timer_irq_disable(hal_lp_timer_id_t id)
{
    /* Implemented by changing sigaction to ignore */
    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigemptyset(&sa.sa_mask); // sigs to be blocked during handler execution
    sa.sa_flags = 0;
    if (sigaction(lptim_signo[id], &sa, NULL) == -1)
    {
        mcu_panic();
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void hal_pl_timer_handler(int sig, siginfo_t *si, void *uc)
{
    int id = si->si_value.sival_int;
    if (lptim_tmr_irq[id].callback != NULL)
    {
        lptim_tmr_irq[id].callback(lptim_tmr_irq[id].context);
    }
}

/* --- EOF ------------------------------------------------------------------ */
