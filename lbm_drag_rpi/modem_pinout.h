/*!
 * \file      modem_pinout.h
 *
 * \brief     modem specific pinout
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
#ifndef __MODEM_PIN_NAMES_H__
#define __MODEM_PIN_NAMES_H__

#ifdef __cplusplus
extern "C"
{
#endif

    /*
     * -----------------------------------------------------------------------------
     * --- DEPENDENCIES ------------------------------------------------------------
     */

#include "smtc_hal_gpio_pin_names.h"

    /*
     * -----------------------------------------------------------------------------
     * --- PUBLIC MACROS -----------------------------------------------------------
     */

    /*
     * -----------------------------------------------------------------------------
     * --- PUBLIC CONSTANTS --------------------------------------------------------
     */

    /********************************************************************************/
    /*                         Application     dependant                            */
    /********************************************************************************/
    // clang-format off

//Radio specific pinout and peripherals
#define RADIO_NRST              P_17
#define RADIO_SPI_MOSI          P_10
#define RADIO_SPI_MISO          P_9
#define RADIO_SPI_SCLK          P_11
#define RADIO_NSS               P_25
#define RADIO_DIO_0             P_4
#define RADIO_DIO_1             P_23
#define RADIO_DIO_2             P_24

#define RADIO_SPI_ID            0

// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */
#ifdef __cplusplus
}
#endif

#endif //__MODEM_PIN_NAMES_H__