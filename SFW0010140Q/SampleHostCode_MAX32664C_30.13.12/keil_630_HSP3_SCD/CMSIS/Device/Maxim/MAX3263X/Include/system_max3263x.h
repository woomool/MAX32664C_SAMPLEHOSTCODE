/**
 * @file
 * @brief MAX3263X System Clock Configuration and System Initialization.
 */
 /* ****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 * $Date: 2018-01-22 13:44:02 -0600 (Mon, 22 Jan 2018) $
 * $Revision: 32945 $
 *
 **************************************************************************** */

/* **** Includes **** */
#include <stdint.h>

/* Define to prevent redundant inclusion */
#ifndef _SYSTEM_MAX3263X_H_
#define _SYSTEM_MAX3263X_H_

#ifdef __cplusplus
extern "C" {
#endif



/**
 * @ingroup product_name
 * @{
 */

/* ****************************************************************************
  Define clocks
  *************************************************************************** */
#ifndef HFXIN_FREQ
/**
 *  @internal External HFXIN frequency.
 */
#define HFXIN_FREQ        8000000
#endif

#ifndef RO_FREQ
#define RO_FREQ           96000000  /**< High Frequency Internal Relaxation Oscillator used as the default System Clock Source */
#endif

extern uint32_t SystemCoreClock;     /*!< CMSIS System Clock Frequency (Core Clock)  */

/**
 * Initializes the system.
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and set up the SystemCoreClock variable.
 */
extern void SystemInit(void);

/**
 * Update SystemCoreClock variable.
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from the device hardware.
 */
extern void SystemCoreClockUpdate(void);

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000) // ARM clang compiler is version 6+
__attribute__((weak)) void SystemInit(void);
__attribute__((weak)) int PreInit(void);
__attribute__((weak)) int Board_Init(void);
#else
__weak void SystemInit(void);
__weak int PreInit(void);
__weak int Board_Init(void);
#endif



#ifdef __cplusplus
}
#endif

#endif /* _SYSTEM_MAX3263X_H_ */
