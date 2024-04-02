/*!
 * \file      smtc_hal_mcu.c
 *
 * \brief     MCU Hardware Abstraction Layer implementation
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

#include "smtc_hal_mcu.h"
#include "modem_pinout.h"

#include "smtc_hal_rtc.h"
#include "smtc_hal_spi.h"
#include "smtc_hal_lp_timer.h"
#include <pigpio.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

bool sleeping = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void mcu_gpio_init(void);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_mcu_critical_section_begin(uint32_t *mask)
{
    // not sure we can do anything meaningful here
}

void hal_mcu_critical_section_end(uint32_t *mask)
{
}

void hal_mcu_disable_irq(void)
{
    // not sure we can do anything meaningful here
}

void hal_mcu_enable_irq(void)
{
}

void hal_mcu_init(void)
{
    // Initialize GPIOs
    mcu_gpio_init();

    // Initialize SPI for radio
    hal_spi_init(RADIO_SPI_ID, RADIO_SPI_MOSI, RADIO_SPI_MISO, RADIO_SPI_SCLK);

    // Initialize Low Power Timer
    hal_lp_timer_init(HAL_LP_TIMER_ID_1);
    hal_lp_timer_init(HAL_LP_TIMER_ID_2);

    // Initialize RTC (for real time and wut)
    hal_rtc_init();
}

void hal_mcu_reset(void)
{
    // Cleanup for restart

    // De-initialize RTC
    hal_rtc_de_init();

    // De-initialize Low Power Timers
    hal_lp_timer_de_init(HAL_LP_TIMER_ID_1);
    hal_lp_timer_de_init(HAL_LP_TIMER_ID_2);

    // De-initialize SPI
    hal_spi_de_init(RADIO_SPI_ID);

    // Clears all set GPIO interrupts
    hal_gpio_irq_disable();

    // Terminate GPIO control
    gpioTerminate();

    exit(-1);
}

void hal_mcu_wait_us(const int32_t microseconds)
{
    // non stoppable by signals
    gpioDelay(microseconds);
}

void hal_mcu_set_sleep_for_ms(const int32_t milliseconds)
{
    if (milliseconds <= 0)
    {
        return;
    }

    hal_rtc_wakeup_timer_set_ms( milliseconds );
    sleeping = true;
    while (sleeping)
    {
        // Check every 500 us, no need to be more accurate
        gpioDelay(500);
    }
    // stop timer after sleep process
    hal_rtc_wakeup_timer_stop( );
}

void hal_mcu_wakeup(void)
{
    sleeping = false;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void mcu_gpio_init(void)
{
    if (gpioInitialise() < 0)
    {
        // pigpio initialisation failed.
        mcu_panic();
    }

    hal_gpio_init_out(RADIO_NSS, 1);
    hal_gpio_init_in(RADIO_DIO_0, BSP_GPIO_PULL_MODE_DOWN, BSP_GPIO_IRQ_MODE_RISING, NULL);
    hal_gpio_init_in(RADIO_DIO_1, BSP_GPIO_PULL_MODE_DOWN, BSP_GPIO_IRQ_MODE_RISING_FALLING, NULL);
    hal_gpio_init_in(RADIO_DIO_2, BSP_GPIO_PULL_MODE_DOWN, BSP_GPIO_IRQ_MODE_RISING, NULL);
    hal_gpio_init_in(RADIO_NRST, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_OFF, NULL);
}

/* --- EOF ------------------------------------------------------------------ */
