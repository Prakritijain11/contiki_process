/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*!*****************************************************************************
 *  @file       PWMTimerCC26XX.c
 *  @brief      CC26XX/CC13XX implementation of ti/drivers/PWM.h
 *
 *  # Overview #
 *  CC26XX/CC13XX PWM driver using the built-in GPTimer.
 *
 *  # Note #
 *  PWMTimerCC26XX driver calculations for duty cycle are inverted
 *  (for example 25% duty outputs 75%) and output is inverted again in the pad for
 *  correct behavior. This simplifies calculations as period/ timer load value
 *  can be updated independently of duty cycle / match value.
 *
 *  The driver also requires the GPTimer bit GPT.TnMR.TnPLO to be set.
 *  Using this, the PWM output will be always high when load=match and always
 *  low when match>load. Setting match > load is used for 0% duty cycle
 *
 *******************************************************************************
 */


#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/Log.h>

#include <ti/sysbios/BIOS.h>

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

#include "PWM.h"
#include "ti/drivers/pwm/PWMTimerCC26XX.h"
#include "ti/drivers/timer/GPTimerCC26XX.h"

/* PWMTimerCC26XX defines */
#define PWM_COUNT_MAX    0xFFFFFF /* GPTimer has maximum 24 bits incl prescaler */


/* PWMTimerCC26XX functions */
void       PWMTimerCC26XX_close(PWM_Handle handle);
int        PWMTimerCC26XX_control(PWM_Handle handle, unsigned int cmd, void * arg);
void       PWMTimerCC26XX_init(PWM_Handle handle);
PWM_Handle PWMTimerCC26XX_open(PWM_Handle handle, PWM_Params *params);
int        PWMTimerCC26XX_setDuty(PWM_Handle handle, uint32_t dutyValue);
int        PWMTimerCC26XX_setPeriod(PWM_Handle handle, uint32_t periodValue);
void       PWMTimerCC26XX_start(PWM_Handle handle);
void       PWMTimerCC26XX_stop(PWM_Handle handle);

/* PWMTimerCC26XX internal functions */
static uint32_t         PWMTimerCC26XX_getperiodCounts(PWM_Period_Units periodUnit, uint32_t periodValue);
static uint32_t         PWMTimerCC26XX_getdutyCounts(uint32_t periodCounts, PWM_Duty_Units dutyUnit, uint32_t dutyValue);

/* PWM function table for PWMTimerCC26XX implementation */
const PWM_FxnTable PWMTimerCC26XX_fxnTable =
{
    PWMTimerCC26XX_close,
    PWMTimerCC26XX_control,
    PWMTimerCC26XX_init,
    PWMTimerCC26XX_open,
    PWMTimerCC26XX_setDuty,
    PWMTimerCC26XX_setPeriod,
    PWMTimerCC26XX_start,
    PWMTimerCC26XX_stop,
};


/* PIN configuration used for PWMTimerCC26XX implementation. Handle is shared
   across all PWM peripherals
 */
static PIN_Handle hPins = NULL;
static PIN_State  pinState;

/*!
 *  @brief PWM CC26XX initialization
 *
 *  This is a dummy function since driver implementation assumes
 *  the handle->object->isOpen flag is set to 0 at boot
 *
 *  @pre    Calling context: Hwi, Swi, Task, Main
 *
 *  @param handle  A SPI_Handle
 *
 */
void PWMTimerCC26XX_init(PWM_Handle handle)
{
}


/* Open the specific PWM peripheral with the settings given in params.
   Will return a PWM handle if successfull, NULL if failed.
   PWM will output configured idle level when opened.
   Function sets a dependency on the underlying timer and adds the PWM pin to
   its internal PIN handle.
 */
PWM_Handle PWMTimerCC26XX_open(PWM_Handle handle, PWM_Params *params)
{
    PWMTimerCC26XX_HwAttrs *hwAttrs = handle->hwAttrs;
    PWMTimerCC26XX_Object  *object  = handle->object;

    /* Check if PWM already open  */
    uint32_t key = Hwi_disable();
    if (object->isOpen)
    {
        Hwi_restore(key);
        Log_error1("PWM_open(%x):  Unit already in use.", (UArg) handle);
        return NULL;
    }
    object->isOpen = 1;
    Hwi_restore(key);

    /* Open timer resource */
    GPTimerCC26XX_Params timerParams;
    GPTimerCC26XX_Params_init(&timerParams);
    timerParams.width          = GPT_CONFIG_16BIT;
    timerParams.mode           = GPT_MODE_PWM;
    timerParams.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
    GPTimerCC26XX_Handle hTimer = GPTimerCC26XX_open(hwAttrs->gpTimerUnit, &timerParams);

    /* Fail if cannot open timer */
    if (hTimer == NULL)
    {
        Log_error2("PWM_open(%x): Timer unit (%d) already in use.", (UArg) handle, hwAttrs->gpTimerUnit);
        object->isOpen = false;
        return NULL;
    }

    /* Open pin resource */

    PIN_Config pinConfig;

    /* Initial open of pin handle with no pins in it. Should never fail. */
    if (hPins == NULL)
    {
        pinConfig = PIN_TERMINATE;
        hPins     = PIN_open(&pinState, &pinConfig);
    }

    /* Invert PWM idle level since pin output is inverted */
    uint32_t idleLevel = PIN_GPIO_HIGH;
    if (params->idleLevel == PWM_IDLE_HIGH)
    {
        idleLevel = PIN_GPIO_LOW;
    }

    /* Generate pin config for PWM pin.
     *  Output is inverted to make PWM duty calculations independent of period
     */
    pinConfig = hwAttrs->pwmPin | PIN_INPUT_DIS | PIN_GPIO_OUTPUT_EN | idleLevel |
                PIN_INV_INOUT | PIN_PUSHPULL | PIN_DRVSTR_MAX;

    /* Fail if cannot add pin */
    if (PIN_add(hPins, pinConfig) != PIN_SUCCESS)
    {
        Log_error2("PWM_open(%x): PIN (%d) already in use.", (UArg) handle, hwAttrs->pwmPin);
        GPTimerCC26XX_close(hTimer);
        object->isOpen = false;
        return NULL;
    }


    /* Store configuration to object */
    object->periodUnit  = params->periodUnits;
    object->periodValue = params->periodValue;
    object->dutyUnit    = params->dutyUnits;
    object->dutyValue   = params->dutyValue;
    object->idleLevel   = params->idleLevel;
    object->hTimer      = hTimer;


    /* Configure PWM period*/
    uint32_t period = object->periodValue;

    if (PWMTimerCC26XX_setPeriod(handle, period) != PWM_STATUS_SUCCESS)
    {
        Log_error1("PWM_open(%x): Failed setting period", (UArg) handle);
        PIN_remove(hPins, hwAttrs->pwmPin);
        GPTimerCC26XX_close(hTimer);
        object->isOpen = false;
        return NULL;
    }

    /* Corner case: If using duty unit PWM_DUTY_FRACTION or PWM_DUTY_US with
       value 0, then duty is already set by setPeriod. Skip if so. */
    PWM_Duty_Units unit  = object->dutyUnit;
    uint32_t      value = object->dutyValue;

    if (!(unit == PWM_DUTY_FRACTION || (value == 0 && unit == PWM_DUTY_US)))
    {
        if (PWMTimerCC26XX_setDuty(handle, value) != PWM_STATUS_SUCCESS)
        {
            Log_error1("PWM_open(%x): Failed setting duty cycle", (UArg) handle);
            PIN_remove(hPins, hwAttrs->pwmPin);
            GPTimerCC26XX_close(hTimer);
            object->isOpen = false;
            return NULL;
        }
    }

    Log_info1("PWM_open(%x): Opened with great success!", (UArg) handle);
    return handle;
}


/* PWMTimerCC26XX_setPeriod -
   Sets / update PWM period. Unit must already be defined in object.
   Also updates duty cycle if using fractional duty cycle
 */
int PWMTimerCC26XX_setPeriod(PWM_Handle handle, uint32_t periodValue)
{
    PWMTimerCC26XX_Object *object = handle->object;
    /* Copy current period unit and store new period */
    uint32_t newperiodCounts = PWMTimerCC26XX_getperiodCounts(object->periodUnit, periodValue);

    /* Fail if period is out of range */
    if (newperiodCounts > PWM_COUNT_MAX || newperiodCounts == 0)
    {
        Log_error2("PWM(%x): Period (%d) is out of range", (UArg) handle, periodValue);
        return PWM_STATUS_INVALID_PERIOD;
    }


    PWM_Duty_Units dutyUnit  = object->dutyUnit;
    uint32_t      dutyValue = object->dutyValue;
    /*  Corner case: 0% duty cycle when using fractional or time unit
        Update duty cycle, needs to be higher than new period counts for 0% duty */
    if (dutyUnit == PWM_DUTY_FRACTION || (dutyValue == 0 && dutyUnit == PWM_DUTY_US))
    {
        object->dutyCounts = PWMTimerCC26XX_getdutyCounts(newperiodCounts, dutyUnit, dutyValue);
        GPTimerCC26XX_setMatchValue(object->hTimer, object->dutyCounts);
    }
    /* Fixed time duty cyle other than 0. No checking when using PWM_DUTY_COUNTS */
    else if (dutyUnit != PWM_DUTY_COUNTS)
    {
        /* Compare to new period and fail if invalid */
        if (newperiodCounts < object->dutyCounts)
        {
            Log_error2("PWM(%x): Period is shorter than duty (%d)", (UArg) handle, periodValue);
            return PWM_STATUS_INVALID_PERIOD;
        }
    }

    /* Store new period and update timer */
    object->periodValue  = periodValue;
    object->periodCounts = newperiodCounts;
    GPTimerCC26XX_setLoadValue(object->hTimer, newperiodCounts);

    Log_info1("PWM_setPeriod(%x): Period set with great success!", (UArg) handle);
    return PWM_STATUS_SUCCESS;
}

/* PWMTimerCC26XX_setDuty -
   Sets / update PWM duty. Unit must already be defined in object.
   Period must already be configured in object before calling this API.
 */
int PWMTimerCC26XX_setDuty(PWM_Handle handle, uint32_t dutyValue)
{
    PWMTimerCC26XX_Object *object = handle->object;
    /* Copy current period unit and store new period */
    PWM_Duty_Units     dutyUnit = object->dutyUnit;
    uint32_t newdutyCounts     = PWMTimerCC26XX_getdutyCounts(object->periodCounts, dutyUnit, dutyValue);

    /* Fail if duty cycle count is out of range */
    if (newdutyCounts > PWM_COUNT_MAX)
    {
        Log_error2("PWM(%x): Duty (%d) is out of range", (UArg) handle, dutyValue);
        return PWM_STATUS_INVALID_PERIOD;
    }

    /* Error checking:
     *  Unit PWM_DUTY_FRACTION will always be within range
     *  Unit PWM_DUTY_US with value 0 will always be correct(set by getdutyCounts)
     *  Unit PWM_DUTY_US value != 0 needs error checking
     *  Unit PWM_DUTY_COUNTS is not checked
     */
    if (dutyUnit == PWM_DUTY_US &&
        dutyValue != 0 &&
        newdutyCounts > object->periodCounts)
    {
        Log_error2("PWM(%x): Duty (%d) is larger than period", (UArg) handle, dutyValue);
        return PWM_STATUS_INVALID_DUTY;
    }

    /* Store new duty cycle and update timer */
    object->dutyValue  = dutyValue;
    object->dutyCounts = newdutyCounts;
    GPTimerCC26XX_setMatchValue(object->hTimer, newdutyCounts);

    Log_info1("PWM_setDuty(%x): Duty set with great success!", (UArg) handle);
    return PWM_STATUS_SUCCESS;
}

/* Return period in timer counts */
static uint32_t PWMTimerCC26XX_getperiodCounts(PWM_Period_Units periodUnit, uint32_t periodValue)
{
    Types_FreqHz freq;
    BIOS_getCpuFreq(&freq);

    uint32_t periodCounts;

    switch (periodUnit)
    {
    case PWM_PERIOD_US:
        periodCounts = ((uint64_t)freq.lo * (uint64_t)periodValue / 1000000) - 1;
        break;
    case PWM_PERIOD_HZ:
        periodCounts = (freq.lo / periodValue) - 1;
        break;
    case PWM_PERIOD_COUNTS:
    /* Fall through */
    default:
        periodCounts = periodValue;
        break;
    }
    return periodCounts;
}

/* Return duty cycle in timer counts */
static uint32_t PWMTimerCC26XX_getdutyCounts(uint32_t periodCounts, PWM_Duty_Units dutyUnit, uint32_t dutyValue)
{
    Types_FreqHz freq;
    BIOS_getCpuFreq(&freq);

    uint32_t dutyCounts;

    /* Corner case, 0% duty cycle. Set timer count higher than period */
    if (dutyValue == 0 &&
        (dutyUnit == PWM_DUTY_US || dutyUnit == PWM_DUTY_FRACTION))
    {
        dutyCounts = periodCounts + 1;
    }
    else
    {
        switch (dutyUnit)
        {
        case PWM_DUTY_US:
            dutyCounts = ((uint64_t)freq.lo * (uint64_t)dutyValue / 1000000) - 1;
            break;
        case PWM_DUTY_FRACTION:
            dutyCounts = (uint64_t)dutyValue * (uint64_t)periodCounts /
                         PWM_DUTY_FRACTION_MAX;
            break;
        case PWM_DUTY_COUNTS:
        /* Fall through */
        default:
            dutyCounts = dutyValue;
            break;
        }
    }
    return dutyCounts;
}

/* Stop PWM output for given PWM peripheral. PWM pin will be routed
   to the GPIO module to provide Idle level and timer is stopped
 */
void PWMTimerCC26XX_stop(PWM_Handle handle)
{
    PWMTimerCC26XX_HwAttrs *hwAttrs = handle->hwAttrs;
    PWMTimerCC26XX_Object  *object  = handle->object;

    GPTimerCC26XX_stop(object->hTimer);
    /* Route PWM pin to GPIO module */
    PINCC26XX_setMux(hPins, hwAttrs->pwmPin, IOC_PORT_GPIO);
}

/* Start PWM output for given PWM peripheral.
   PWM pin will be routed to the Timer module and timer is started
 */
void PWMTimerCC26XX_start(PWM_Handle handle)
{
    PWMTimerCC26XX_HwAttrs *hwAttrs = handle->hwAttrs;
    PWMTimerCC26XX_Object  *object  = handle->object;

    /* Route PWM pin to timer output */
    GPTimerCC26XX_PinMux pinMux = GPTimerCC26XX_getPinMux(object->hTimer);
    PINCC26XX_setMux(hPins, hwAttrs->pwmPin, pinMux);
    GPTimerCC26XX_start(object->hTimer);
}

/* Close the specific PWM peripheral. A running PWM must be stopped first.
   PWM output will revert to output value given in PIN_init if any is defined.
 */
void PWMTimerCC26XX_close(PWM_Handle handle)
{
    PWMTimerCC26XX_HwAttrs *hwAttrs = handle->hwAttrs;
    PWMTimerCC26XX_Object  *object  = handle->object;

    /* Restore PWM pin to GPIO module with default configuration from PIN_init */
    PIN_remove(hPins, hwAttrs->pwmPin);
    /* Close and delete timer handle */
    GPTimerCC26XX_close(object->hTimer);
    object->hTimer = NULL;

    /* Clear isOpen flag */
    uint32_t key = Hwi_disable();
    object->isOpen = 0;
    Hwi_restore(key);
}

/* Driver specific control options. PWM peripheral must be opened before
   using this API */
int PWMTimerCC26XX_control(PWM_Handle handle, unsigned int cmd, void * arg)
{
    PWMTimerCC26XX_Object *object = handle->object;
    int stat = PWM_STATUS_SUCCESS;

    switch (cmd)
    {
    case PWMTimerCC26XX_CMD_DEBUG_STALL:
        GPTimerCC26XX_configureDebugStall(object->hTimer, *(GPTimerCC26XX_DebugMode *)arg);
        break;
    default:
        stat = PWM_STATUS_UNDEFINEDCMD;
        break;
    }
    return stat;
}
