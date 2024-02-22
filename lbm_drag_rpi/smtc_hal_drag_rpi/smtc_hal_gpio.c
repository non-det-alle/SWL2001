/*!
 * \file      smtc_hal_gpio.c
 *
 * \brief     GPIO Hardware Abstraction Layer implementation
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
#include <stdlib.h>  // exit

#include "smtc_hal_gpio.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
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
 * GPIO initialize
 */
void hal_gpio_init(const hal_gpio_pin_names_t pin, const uint32_t value,
                   const uint32_t pull_mode, const uint32_t io_mode);

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

    hal_gpio_init(pin, PI_CLEAR, pulls[pull_mode], PI_INPUT);

    gpio_irq_mode[pin - 0x2u] = irq_mode;

    hal_gpio_irq_attach(irq);
}

void hal_gpio_init_out(const hal_gpio_pin_names_t pin, const uint32_t value)
{
    hal_gpio_init(pin, value, PI_PUD_OFF, PI_OUTPUT);
}

void hal_gpio_irq_attach(const hal_gpio_irq_t *irq)
{
    if ((irq == NULL) || (irq->callback == NULL))
    {
        return;
    }

    uint8_t irq_mode = gpio_irq_mode[irq->pin - 0x2u];
    if (irq_mode == BSP_GPIO_IRQ_MODE_OFF)
    {
        return;
    }

    gpio_irq[irq->pin - 0x2u] = irq;
    if (gpioSetISRFunc(irq->pin, modes[irq_mode], 0, hal_gpio_irq_callback) != 0)
    {
        mcu_panic();
    }
}

void hal_gpio_irq_detach(const hal_gpio_irq_t *irq)
{
    if (irq == NULL)
    {
        return;
    }

    if (gpioSetISRFunc(irq->pin, 0, 0, NULL) != 0)
    {
        // no reset to avoid error-looping
        mcu_panic_trace();
    }
    gpio_irq[irq->pin - 0x2u] = NULL;
}

void hal_gpio_irq_enable(void)
{
    for (size_t i = 0; i < P_NUM; i++)
    {
        if (gpio_irq[i] != NULL && gpio_irq_mode[i] != BSP_GPIO_IRQ_MODE_OFF)
        {
            if (gpioSetISRFunc(gpio_irq[i]->pin, modes[gpio_irq_mode[i]], 0, hal_gpio_irq_callback) != 0)
            {
                mcu_panic();
            }
        }
    }
}

void hal_gpio_irq_disable(void)
{
    for (size_t i = 0; i < P_NUM; i++)
    {
        if (gpio_irq[i] != NULL)
        {
            if (gpioSetISRFunc(gpio_irq[i]->pin, 0, 0, NULL) != 0)
            {
                // no reset to avoid error-looping
                mcu_panic_trace();
            }
        }
    }
}

//
// MCU pin state control
//

void hal_gpio_set_value(const hal_gpio_pin_names_t pin, const uint32_t value)
{
    if (gpioWrite(pin, (value != 0) ? PI_SET : PI_CLEAR) != 0)
    {
        mcu_panic();
    }
}

uint32_t hal_gpio_get_value(const hal_gpio_pin_names_t pin)
{
    int value = gpioRead(pin);
    if (value == PI_BAD_GPIO)
    {
        mcu_panic();
    }
    return value;
}

void hal_gpio_clear_pending_irq(const hal_gpio_pin_names_t pin)
{
    // IRQ cannot pend
}

void hal_gpio_enable_clock(const hal_gpio_pin_names_t pin)
{
    // supposedly already enabled by pigpio
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void hal_gpio_init(const hal_gpio_pin_names_t pin, const uint32_t value,
                   const uint32_t pull_mode, const uint32_t io_mode)
{
    hal_gpio_set_value(pin, value);
    if (gpioSetPullUpDown(pin, pull_mode) != 0)
    {
        mcu_panic();
    }
    if (gpioSetMode(pin, io_mode) != 0)
    {
        mcu_panic();
    }
}

void hal_gpio_irq_callback(int gpio, int level, uint32_t tick)
{
    uint8_t callback_index = gpio - 0x2u;
    if ((gpio_irq[callback_index] != NULL) && (gpio_irq[callback_index]->callback != NULL))
    {
        gpio_irq[callback_index]->callback(gpio_irq[callback_index]->context);
    }
}

/* --- EOF ------------------------------------------------------------------ */
