/*!
 * \file      smtc_hal_gpio.c
 *
 * \brief     GPIO Hardware Abstraction Layer implementation
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

#include "smtc_hal_gpio.h"
#include <pigpio.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#define OFF 3

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * Conversion arrays for pigpio.h settings
 */
static const uint8_t modes[] = {OFF, RISING_EDGE, FALLING_EDGE, EITHER_EDGE};
static const uint8_t pulls[] = {PI_PUD_OFF, PI_PUD_UP, PI_PUD_DOWN};

/*!
 * Array holding IRQ modes for set gpio pins
 *
 * Note: global and static arrays are automatically initialized to 0
 */
static hal_gpio_irq_mode_t gpio_irq_mode[P_NUM];

/*!
 * Array holding attached IRQ gpio data context
 */
static hal_gpio_irq_t const *gpio_irq[P_NUM];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * GPIO IRQ callback
 */
void hal_gpio_irq_callback(int gpio, int level, uint32_t tick);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

//
// MCU input pin Handling
//

void hal_gpio_init_in(const hal_gpio_pin_names_t pin, const hal_gpio_pull_mode_t pull_mode,
                      const hal_gpio_irq_mode_t irq_mode, hal_gpio_irq_t *irq)
{
    if (irq != NULL)
    {
        irq->pin = pin;
    }

    gpioWrite(pin, PI_CLEAR);
    gpioSetPullUpDown(pin, pulls[pull_mode]);
    gpioSetMode(pin, PI_INPUT);

    uint8_t mode = modes[irq_mode];
    if ((mode == RISING_EDGE) || (mode == FALLING_EDGE) ||
        (mode == EITHER_EDGE))
    {
        hal_gpio_irq_attach(irq);
        gpioSetISRFunc(pin, mode, 0, hal_gpio_irq_callback);
        gpio_irq_mode[pin - 0x2u] = irq_mode;
    }
}

void hal_gpio_init_out(const hal_gpio_pin_names_t pin, const uint32_t value)
{
    gpioWrite(pin, (value != 0) ? PI_SET : PI_CLEAR);
    gpioSetPullUpDown(pin, PI_PUD_OFF);
    gpioSetMode(pin, PI_OUTPUT);
}

void hal_gpio_irq_attach(const hal_gpio_irq_t *irq)
{
    if ((irq != NULL) && (irq->callback != NULL))
    {
        gpio_irq[irq->pin - 0x2u] = irq;
    }
}

void hal_gpio_irq_detach(const hal_gpio_irq_t *irq)
{
    if (irq != NULL)
    {
        gpio_irq[irq->pin - 0x2u] = NULL;
    }
}

void hal_gpio_irq_enable(void)
{
    for (size_t i = 0; i < P_NUM; i++)
    {
        if (gpio_irq[i] != NULL && gpio_irq_mode[i] != BSP_GPIO_IRQ_MODE_OFF)
        {
            gpioSetISRFunc(gpio_irq[i]->pin, modes[gpio_irq_mode[i]], 0, hal_gpio_irq_callback);
        }
    }
}

void hal_gpio_irq_disable(void)
{
    for (size_t i = 0; i < P_NUM; i++)
    {
        if (gpio_irq[i] != NULL)
        {
            gpioSetISRFunc(gpio_irq[i]->pin, 0, 0, NULL);
        }
    }
}

//
// MCU pin state control
//

void hal_gpio_set_value(const hal_gpio_pin_names_t pin, const uint32_t value)
{
    gpioWrite(pin, (value != 0) ? PI_SET : PI_CLEAR);
}

uint32_t hal_gpio_get_value(const hal_gpio_pin_names_t pin)
{
    return gpioRead(pin);
}

void hal_gpio_clear_pending_irq(const hal_gpio_pin_names_t pin)
{
    // ?
}

void hal_gpio_enable_clock(const hal_gpio_pin_names_t pin)
{
   // used to enable SPI pins clk, but already enabled on RPi?
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void hal_gpio_irq_callback(int gpio, int level, uint32_t tick)
{
    uint8_t callback_index = gpio - 0x2u;
    if ((gpio_irq[callback_index] != NULL) && (gpio_irq[callback_index]->callback != NULL))
    {
        gpio_irq[callback_index]->callback(gpio_irq[callback_index]->context);
    }
}

/* --- EOF ------------------------------------------------------------------ */
