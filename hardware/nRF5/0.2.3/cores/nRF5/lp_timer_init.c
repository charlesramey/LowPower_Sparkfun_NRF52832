
#include <nrf52.h>
#include <nrf_soc.h>

#include "lp_timer_init.h"

void cprint(const char* str);
void cprintNum(const char* str, const uint32_t num);


/*****
// use this code in you sketch for debugging C code files
extern "C" void cprint(const char* str) {
#ifdef DEBUG
  Serial.println(str);
#else 
  // nothing here
#endif
}

extern "C" void cprintNum(const char* str, uint32_t num) {
#ifdef DEBUG
  Serial.print(str);
  Serial.print(' ');
  Serial.println(num);
#else 
  // nothing here
#endif
}

***********/

// note these methods have "C" code calling conventions
void clearFPU_IRQ();
void app_sched_execute(void);
void app_sched_comp_execute(void);

void BLEPeripheralInstancePoll();

// see https://devzone.nordicsemi.com/f/nordic-q-a/12433/fpu-divide-by-0-and-high-current-consumption
// call this is using floating point processor to clear any exceptions that may be raised
void clearFPU_IRQ() {
  // may not be needed if not using Floating Point Processor
  // Set bit 7 and bits 4..0 in the mask to one (0x ...00 1001 1111)
#define FPU_EXCEPTION_MASK 0x0000009F
  //https://devzone.nordicsemi.com/f/nordic-q-a/13670/solved-nrf52-sd_app_evt_wait-will-not-go-to-sleep
  //  The FPU will generate an exception when dividing by 0 due to overflow/underflow.
  //  This exception will trigger the FPU interrupt.
  //  If the interrupt is not cleared/handled the CPU will not be able to go to sleep.

  /* Clear exceptions and PendingIRQ from the FPU unit */
  __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));
  (void) __get_FPSCR(); // cortexM4 i.e. nrf52832 needs read after write to commit of change
  NVIC_ClearPendingIRQ(FPU_IRQn); //this fixes low power as the stuck IRQ is cleared
}

void sleep() {
  waitForTrigger();
  processTrigger();
}

void waitForTrigger() {
  clearFPU_IRQ(); // clear any FPU exceptions your loop code (or other code) generated
  sd_app_evt_wait(); // sleep here waiting for next event/interrupt
  // do this now so that we will detect any interrupt generated while we are processing the ble events and timing events
  NVIC_ClearPendingIRQ(SD_EVT_IRQn); //this fixes low power as the stuck IRQ is cleared
}

void processTrigger() {
  BLEPeripheralInstancePoll(); // handle BLE msgs
  app_sched_comp_execute(); // then handle comparator events first max 4 events
  app_sched_execute(); // then handle queued events, e.g. timers
}

// Function returns true if called from main context (CPU in thread
// mode), and returns false if called from an interrupt context.
bool is_main_context ( void ) {
  static const uint8_t ISR_NUMBER_THREAD_MODE = 0;
  uint8_t isr_number = __get_IPSR();
  if ((isr_number ) == ISR_NUMBER_THREAD_MODE) {
    return true;
  } else {
    return false;
  }
}


// setup the nrf52 SDK timer and scheduler 
// uses defines APP_TIMER_PRESCALER , APP_TIMER_OP_QUEUE_SIZE 
// and SCHED_MAX_EVENT_DATA_SIZE,  SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE
// COMP_QUEUE_SIZE must be < SCHED_QUEUE_SIZE
// from lp_timer_init.h header
void lp_timer_init() {
	// this version on INIT creates two queues in one array to seperate comparator triggers from 
	// overloading other triggers.
  APP_SCHED_COMP_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE, SCHED_COMP_QUEUE_SIZE);
  // Initialize the application timer module.
  APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);
  // wiring.c init() starts RTC2
  // init() (in wiring.c) called from main() before calling lp_timer.init
}

