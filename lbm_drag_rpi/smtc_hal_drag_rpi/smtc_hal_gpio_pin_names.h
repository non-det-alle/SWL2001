/*!
 * \file      smtc_hal_gpio_pin_names.h
 *
 * \brief     Defines NucleoL476 platform pin names
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

#ifndef __SMTC_HAL_GPIO_PIN_NAMES_H__
#define __SMTC_HAL_GPIO_PIN_NAMES_H__

#ifdef __cplusplus
extern "C"
{
#endif

    /*
     * -----------------------------------------------------------------------------
     * --- DEPENDENCIES ------------------------------------------------------------
     */

    /*
     * -----------------------------------------------------------------------------
     * --- PUBLIC MACROS -----------------------------------------------------------
     */

    /*
     * -----------------------------------------------------------------------------
     * --- PUBLIC CONSTANTS --------------------------------------------------------
     */
    #define P_NUM 26

    /*
     * -----------------------------------------------------------------------------
     * --- PUBLIC TYPES ------------------------------------------------------------
     */
    typedef enum gpio_pin_names_e
    {
        // GPIO
        P_2 = 0x02,
        P_3 = 0x03,
        P_4 = 0x04,
        P_5 = 0x05,
        P_6 = 0x06,
        P_7 = 0x07,
        P_8 = 0x08,
        P_9 = 0x09,
        P_10 = 0x0A,
        P_11 = 0x0B,
        P_12 = 0x0C,
        P_13 = 0x0D,
        P_14 = 0x0E,
        P_15 = 0x0F,
        P_16 = 0x10,
        P_17 = 0x11,
        P_18 = 0x12,
        P_19 = 0x13,
        P_20 = 0x14,
        P_21 = 0x15,
        P_22 = 0x16,
        P_23 = 0x17,
        P_24 = 0x18,
        P_25 = 0x19,
        P_26 = 0x1A,
        P_27 = 0x1B,
        // Not connected
        NC = -1
    } hal_gpio_pin_names_t;

    /*
     * -----------------------------------------------------------------------------
     * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
     */

#ifdef __cplusplus
}
#endif

#endif // __SMTC_HAL_GPIO_PIN_NAMES_H__

/* --- EOF ------------------------------------------------------------------ */
