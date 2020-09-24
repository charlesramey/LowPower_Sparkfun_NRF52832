//lp_timer.h
#ifndef LP_TIMER_H
#define LP_TIMER_H

#include <stdint.h>

#include "utility/app_timer.h"
#include "utility/app_scheduler.h"
#include "Print.h"


class lp_timer {
public:
  lp_timer();
  // min timeout_mS is 2 if <2 then timeout_mS is set to 2
  // max timeout_mS is 4095000, if >4095000 then timeout_mS is set to 4095000
  uint32_t startTimer(uint32_t timeout_mS, void (*handler)(void));
  uint32_t startDelay(uint32_t timeout_mS, void (*handler)(void));
  
// options for mode are defined in utility/app_timer.h
//    APP_TIMER_MODE_SINGLE_SHOT,                 /**< The timer will expire only once. */
//    APP_TIMER_MODE_REPEATED                     /**< The timer will restart each time it expires. */  
  uint32_t start(uint32_t timeout_mS, void (*handler)(void),app_timer_mode_t mode);
  void setDebugStream(Print* debugOut);
  bool isRunning();
  uint32_t getTimeout();
  bool isRepeating(); // true if running and repeating else false if stopped OR single shot
  uint32_t stop();
  const static uint32_t MAX_TIMEOUT = 4095000;
  void (*_timeoutHandler)(void);
  
protected:
	Print* debugOut;
	
private:
  uint32_t init(void (*timeout_handler)(void) , app_timer_mode_t mode);
  bool repeating;
  bool created;
  uint32_t timeout;
  app_timer_t timer_data;
  app_timer_t* p_timer_data;
	
};

#endif // #ifndef LP_TIMER_H
