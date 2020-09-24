#ifndef LP_TIMER_INIT_H
#define LP_TIMER_INIT_H

// General application timer settings.
// Value of the RTC1 PRESCALER register. 7 => 4096Hz counter for millis and lp_timer 
// Min lp_timer time is 2mS 
// micros increment in intervals of 244uS
#define APP_TIMER_PRESCALER 7
// Size of timer operation queues.
#define APP_TIMER_OP_QUEUE_SIZE 20 

// Scheduler settings
//#define SCHED_MAX_EVENT_DATA_SIZE sizeof(uint32_t)
// allow for both scheduled timer event and your own scheduled events
#define SCHED_MAX_EVENT_DATA_SIZE MAX(APP_TIMER_SCHED_EVT_SIZE, sizeof(uint32_t))
#define SCHED_QUEUE_SIZE 8
#define SCHED_COMP_QUEUE_SIZE 4
// queue goes from 0 to 8 for normal sched and 9 to 13 for comp,
// loose one index to detect empty queue (start == end)

// uncomment this in utility/app_schedule.h to enable SCHEDULER TRIGGER QUEUE profiler
//#define SCHEDULER_PROFILER
// then call uint16_t app_sched_queue_utilization_get()  to see the max queue size, excluding lp_comparator

#include "utility/app_scheduler.h"

// include collection of nordic macros
#include "utility/nordic_common.h"

// include nrf error defines, only NRF_SUCCESS used here but you can use these to check error returns
// which in general are not checked in this code/library
#include "nrf_error.h"

// include scheduled timer, uses LF clock and RTC1  millis(), micros(), delay() also use RTC1 
// uses SWI0_IRQn and SWI0_IRQHandler
#include "utility/app_timer_appsh.h"

// included LF clock handling. Note: this shares an interrupt with nrf_drv_power but that is not used in these examples/library so no special handling needed
#include "utility/nrf_drv_clock.h"


#ifdef __cplusplus
extern "C" {
#endif


// setup the nrf52 SDK timer and scheduler 
// uses defines APP_TIMER_PRESCALER , APP_TIMER_OP_QUEUE_SIZE 
// and SCHED_MAX_EVENT_DATA_SIZE,  SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE
// from lp_timer_init.h header
void lp_timer_init();

// starts the softdevice and LF clock
void BLEPeripheralStartSoftDevice();

// Function returns true if called from main context (CPU in thread
// mode), and returns false if called from an interrupt context.
bool is_main_context ( void );

#ifdef __cplusplus
}
#endif

#endif // #define LP_TIMER_INIT_H
