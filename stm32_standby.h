#ifndef _STM32_STANDBY_RTC_WAKEUP_H_
#define _STM32_STANDBY_RTC_WAKEUP_H_

#include "mbed.h"
#include "mbed_debug.h"     // Use debug() instead of printf() so we don't include UART drivers in release builds
#include "rtc_api_hal.h"

enum WakeupType {
    WAKEUP_RESET,
    WAKEUP_TIMER,
    WAKEUP_PIN
};

static RTC_HandleTypeDef RtcHandle;


// static void MX_RTC_Init(void)
// {
//   RTC_TimeTypeDef sTime = {0};
//   RTC_DateTypeDef DateToUpdate = {0};
//   RTC_AlarmTypeDef sAlarm = {0};

//   hrtc.Instance = RTC;
//   hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
//   hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
//   if (HAL_RTC_Init(&hrtc) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   sTime.Hours = 0x1;
//   sTime.Minutes = 0x0;
//   sTime.Seconds = 0x0;

//   if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
//   DateToUpdate.Month = RTC_MONTH_JANUARY;
//   DateToUpdate.Date = 0x1;
//   DateToUpdate.Year = 0x0;

//   if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /**Enable the Alarm A 
//   */
//   sAlarm.AlarmTime.Hours = 0x1;
//   sAlarm.AlarmTime.Minutes = 0x0;
//   sAlarm.AlarmTime.Seconds = 0x10;
//   sAlarm.Alarm = RTC_ALARM_A;
//   if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }




// static void rtc_set_wake_up_timer_s(uint32_t delta)
// {
//     uint32_t clock = RTC_WAKEUPCLOCK_CK_SPRE_16BITS;

//     // HAL_RTCEx_SetWakeUpTimer_IT will assert that delta is 0xFFFF at max
//     if (delta > 0xFFFF) {
//         delta -= 0x10000;
//         clock = RTC_WAKEUPCLOCK_CK_SPRE_17BITS;
//     }

//     RtcHandle.Instance = RTC;

//     HAL_StatusTypeDef status = HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, delta, clock);

//     if (status != HAL_OK) {
//         debug("Set wake up timer failed: %d\n", status);
//         NVIC_SystemReset();
//      }
// }

// static WakeupType get_wakeup_type() {
//     if(READ_BIT(RTC->ISR, RTC_ISR_WUTF))
//         return WAKEUP_TIMER;

//     // this is set by timer too, but that's checked already
//     // above.
//     if(READ_BIT(PWR->CSR, PWR_CSR_WUF))
//         return WAKEUP_PIN;

//     return WAKEUP_RESET;
// }

void standby(int seconds) {
    core_util_critical_section_enter();

    // Clear wakeup flag, just in case.
    SET_BIT(PWR->CR, PWR_CR_CWUF);

    // Enable wakeup timer.
    // rtc_set_wake_up_timer_s(seconds);

    // Enable debug interface working in standby. Causes power consumption to increase drastically while in standby.
    //HAL_DBGMCU_EnableDBGStandbyMode();

    HAL_PWR_EnterSTANDBYMode();

    // this should not happen...
    rtc_deactivate_wake_up_timer();
    core_util_critical_section_exit();

    debug("Continued after getting out of STANDBY mode - this should not happen\n");

    // something went wrong, let's reset
    NVIC_SystemReset();
}

#endif // _STM32_STANDBY_RTC_WAKEUP_H_
