/*! ----------------------------------------------------------------------------
 * @file    deca_mutex.c
 * @brief   IRQ interface / mutex implementation
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 */

#include <deca_device_api.h>
#include <port.h>

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexon()
 *
 * Description: This function should disable interrupts. This is called at the start of a critical section.
 * It returns the irq state before disable, this value is used to re-enable in decamutexoff call.
 *
 * returns the state of the DW3000 interrupt
 */
decaIrqStatus_t decamutexon(void)
{
    decaIrqStatus_t s = port_GetEXT_IRQStatus();

    if (s)
    {
        port_DisableEXT_IRQ();
    }
    return s;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexoff()
 *
 * Description: This function should re-enable interrupts, or at least restore their state as saved by decamutexon.
 * This is called at the end of a critical section.
 */
void decamutexoff(decaIrqStatus_t s)
{
    if (s)
    {
        port_EnableEXT_IRQ();
    }
}
