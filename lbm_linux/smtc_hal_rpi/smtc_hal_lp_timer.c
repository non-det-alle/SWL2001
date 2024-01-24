/*!
 * \file      smtc_hal_lp_timer.c
 *
 * \brief     Implements Low Power Timer utilities functions.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>  // C99 types
#include <stdbool.h> // bool type

#include "smtc_hal_lp_timer.h"
#include "smtc_hal_mcu.h"

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

#define SIG(t) (SIGRTMIN + (t) > SIGRTMAX ? mcu_panic() : SIGRTMIN + (t))

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

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
    hal_lp_timer_irq_enable(id); // establish handler

    struct sigevent sev;
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIG(id);
    sev.sigev_value.sival_int = id; // pass id here
    if (timer_create(RPI_RTC, &sev, &lptim_handle[id]) == -1)
    {
        mcu_panic();
    }

    lptim_tmr_irq[id] = (hal_lp_timer_irq_t){.context = NULL, .callback = NULL};
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
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    if (sigaction(SIG(id), &sa, NULL) == -1)
    {
        mcu_panic()
    }
}

void hal_lp_timer_irq_disable(hal_lp_timer_id_t id)
{
    /* Implemented by changing sigaction to ignore */
    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigemptyset(&sa.sa_mask); // sigs to be blocked during handler execution
    sa.sa_flags = 0;
    if (sigaction(SIG(id), &sa, NULL) == -1)
    {
        mcu_panic()
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
