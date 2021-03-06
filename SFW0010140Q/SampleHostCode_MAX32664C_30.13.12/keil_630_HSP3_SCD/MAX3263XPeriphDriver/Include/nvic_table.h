/**
 * @file
 * @brief      NVIC utility function and type declarations.
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
 * $Date: 2018-03-19 09:35:56 -0500 (Mon, 19 Mar 2018) $
 * $Revision: 34027 $
 *
 *************************************************************************** */

/* Define to prevent redundant inclusion */
#ifndef _NVIC_TABLE_H
#define _NVIC_TABLE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup 	syscfg 
 * @defgroup    mxc_nvic NVIC Functions
 * @brief 		Utility functions for setting an IRQ handler dynamically
 * @{
 */ 

/**
 * @brief      Type alias for an IRQ handler. 
 * @details    Type alias for an IRQ handler function with prototype:. 
 * @code
 *      void irq_handler(void);
 * @endcode
 *                     
 */
typedef void (*irq_fn)(void);

/**
 * @brief      Set an IRQ hander function for an IRQ specified by @p irqn.
 * @details    If the IRQ table is in flash, this function will copy the IRQ table to RAM.
 *
 * @param      irqn         ARM external IRQ number, see #IRQn_Type
 * @param      irq_handler  Function to be called at IRQ context
 */
#if defined ( __GNUC__ )
int NVIC_SetVector(IRQn_Type irqn, irq_fn irq_handler);
#else
void NVIC_SetVector(IRQn_Type irqn, irq_fn irq_handler);
#endif
/**
 * @brief Copy NVIC vector table to RAM and set NVIC to RAM based table.
 *
 */
void NVIC_SetRAM(void);

#if !defined ( __GNUC__ )
/**
 * @brief      Get Interrupt Vector
 * @details    Reads an interrupt vector from interrupt vector table. The
 *             interrupt number can be positive to specify a device specific
 *             interrupt, or negative to specify a processor exception.
 * @param[in]  IRQn  Interrupt number.
 * @return     Address of interrupt handler function
 */
uint32_t NVIC_GetVector(IRQn_Type IRQn);
#endif

/**@} end of group mxc_nvic */

#ifdef __cplusplus
}
#endif

#endif /* _NVIC_TABLE_H */
