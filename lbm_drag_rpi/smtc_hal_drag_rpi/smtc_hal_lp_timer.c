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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

typedef struct hal_lp_timer_s
{
    int signo;
    timer_t handle;
    hal_lp_timer_irq_t tmr_irq;
    bool blocked;
    bool pending;
} hal_lp_timer_t;

static hal_lp_timer_t lptim[HAL_LP_TIMER_NB] = {
    {
        .tmr_irq = {
            .context = NULL,
            .callback = NULL,
        },
        .blocked = false,
        .pending = false,
    },
    {
        .tmr_irq = {
            .context = NULL,
            .callback = NULL,
        },
        .blocked = false,
        .pending = false,
    },
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
    // + 1 is for rtc wakeup timer
    int signo = SIGRTMIN + id + 1;
    if (signo > SIGRTMAX)
    {
        mcu_panic();
    }
    lptim[id].signo = signo;

    struct sigaction sa; // establish handler for callback
    sa.sa_sigaction = hal_pl_timer_handler;
    sigemptyset(&sa.sa_mask); // sigs to be blocked during handler execution
    sa.sa_flags = SA_SIGINFO;
    if (sigaction(signo, &sa, NULL) == -1)
    {
        mcu_panic();
    }

    struct sigevent sev;
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = signo;
    sev.sigev_value.sival_int = id; // pass id here
    if (timer_create(RT_CLOCK, &sev, &lptim[id].handle) == -1)
    {
        mcu_panic();
    }
}

void hal_lp_timer_de_init(hal_lp_timer_id_t id)
{
    if (timer_delete(lptim[id].handle) == -1)
    {
        // no reset to avoid error-looping
        mcu_panic_trace();
    }

    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(lptim[id].signo, &sa, NULL) == -1)
    {
        // no reset to avoid error-looping
        mcu_panic_trace();
    }
}

void hal_lp_timer_start(hal_lp_timer_id_t id, const uint32_t milliseconds, const hal_lp_timer_irq_t *tmr_irq)
{
    lptim[id].tmr_irq = *tmr_irq; // callback assignment

    struct itimerspec its;
    its.it_value.tv_sec = milliseconds / 1000;
    its.it_value.tv_nsec = milliseconds % 1000 * 1000000;
    its.it_interval = ZERO;
    if (timer_settime(lptim[id].handle, 0, &its, NULL) == -1)
    {
        mcu_panic();
    }
}

void hal_lp_timer_stop(hal_lp_timer_id_t id)
{
    lptim[id].tmr_irq = (hal_lp_timer_irq_t){.context = NULL, .callback = NULL};

    struct itimerspec its;
    its.it_value = ZERO;
    its.it_interval = ZERO;
    if (timer_settime(lptim[id].handle, 0, &its, NULL) == -1)
    {
        mcu_panic();
    }
}

void hal_lp_timer_irq_enable(hal_lp_timer_id_t id)
{
    lptim[id].blocked = false;

    if (lptim[id].pending && lptim[id].tmr_irq.callback != NULL)
    {
        lptim[id].tmr_irq.callback(lptim[id].tmr_irq.context);
        lptim[id].pending = false;
    }
}

void hal_lp_timer_irq_disable(hal_lp_timer_id_t id)
{
    lptim[id].blocked = true;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void hal_pl_timer_handler(int sig, siginfo_t *si, void *uc)
{
    int id = si->si_value.sival_int;

    if (lptim[id].blocked)
    {
        lptim[id].pending = true;
        return;
    }

    if (lptim[id].tmr_irq.callback != NULL)
    {
        lptim[id].tmr_irq.callback(lptim[id].tmr_irq.context);
    }
}

/* --- EOF ------------------------------------------------------------------ */
