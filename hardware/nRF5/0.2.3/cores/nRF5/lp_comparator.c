// lp_comparator.c

#define NRF52_SERIES
// <o> LPCOMP_CONFIG_IRQ_PRIORITY  - Interrupt priority
// <i> Priorities 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest)
// <1=> 1
// <2=> 2
// <3=> 3
#define LPCOMP_CONFIG_IRQ_PRIORITY 3

// <o> LPCOMP_CONFIG_REFERENCE  - Reference voltage
 
// <0=> Supply 1/8 
// <1=> Supply 2/8 
// <2=> Supply 3/8 
// <3=> Supply 4/8 
// <4=> Supply 5/8 
// <5=> Supply 6/8 
// <6=> Supply 7/8 
// <8=> Supply 1/16 (nRF52) 
// <9=> Supply 3/16 (nRF52) 
// <10=> Supply 5/16 (nRF52) 
// <11=> Supply 7/16 (nRF52) 
// <12=> Supply 9/16 (nRF52) 
// <13=> Supply 11/16 (nRF52) 
// <14=> Supply 13/16 (nRF52) 
// <15=> Supply 15/16 (nRF52) 
// <7=> External Ref 0 
// <65543=> External Ref 1 

#include "utility/nrf_lpcomp.h"
#include "utility/nrf_drv_lpcomp.h"
#include "lp_comparator.h"
#include "nrf_error.h"

//void cprint(const char* str);
//void cprintNum(const char* str, const uint32_t num);

extern const uint32_t g_ADigitalPinMap[];

enum LPCOMP_REF lp_comp_ref;

typedef void (*pfod_lpcomp_handler_fun)(int);
// in c++ file
static pfod_lpcomp_handler_fun _lpcomp_handler; // this has C++ linkage so just declare it in your sketch and pass as arg

/** LPCOMP configuration.
  typedef struct {
    nrf_lpcomp_ref_t            reference; // LPCOMP reference
    nrf_lpcomp_detect_t         detection; // LPCOMP detection type
    nrf_lpcomp_hysteresis_t     hyst;      // LPCOMP hysteresis
  } nrf_lpcomp_config_t;
**/
/**
  typedef enum {
    NRF_LPCOMP_DETECT_CROSS = LPCOMP_ANADETECT_ANADETECT_Cross, // Generate ANADETEC on crossing, both upwards and downwards crossing
    NRF_LPCOMP_DETECT_UP    = LPCOMP_ANADETECT_ANADETECT_Up,    // Generate ANADETEC on upwards crossing only
    NRF_LPCOMP_DETECT_DOWN  = LPCOMP_ANADETECT_ANADETECT_Down   // Generate ANADETEC on downwards crossing only
  } nrf_lpcomp_detect_t;
**/

void pfod_lpcom_schedule_event_handler(void* nullPtr, uint16_t isHigh) {
   _lpcomp_handler((int)isHigh);
}

void sampleAndSendEvent() {
  NRF_LPCOMP->TASKS_SAMPLE = 1;
  uint32_t result = nrf_lpcomp_result_get();
  uint8_t cmpResult = 0;
  if (result != 0) {
    cmpResult = 1;
  }
  app_sched_comp_event_put(cmpResult, pfod_lpcom_schedule_event_handler);
}

/**
   @brief LPCOMP event handler is called when LPCOMP detects voltage drop.

   This function is called from interrupt context so it is very important
   to return quickly. Don't put busy loops or any other CPU intensive actions here.
   It is also not allowed to call soft device functions from it (if LPCOMP IRQ
   priority is set to APP_IRQ_PRIORITY_HIGH).
*/
static void pfod_lpcomp_event_handler(nrf_lpcomp_event_t event) {
  //	NRF_LPCOMP_EVENT_READY or NRF_LPCOMP_EVENT_CROSS
  sampleAndSendEvent();
}

// call this is you want to use the ADC
// you will need to start lpcomp again afterwards
void lp_comparator_stop() {
  nrf_drv_lpcomp_uninit();
}

/**
  // comparator pins for nRF52832
  // INPUT0 P0_02
  // INPUT1 P0_03
  // INPUT2 P0_04
  // INPUT3 P0_05
  // INPUT4 P0_28
  // INPUT5 P0_29
  // INPUT6 P0_30
  // INPUT7 P0_31
  
 uint32_t lp_comparator_start(uint32_t ulPin, lp_ref refVolts, void (*handler_fun)(int) )
 NOTE ulPin is specified as A0 to A7 mapped to chip pin from g_ADigitalPinMap[] in variants
 for NanoV2 and Skylab NanoV2 replacement only A0 to A5 are available for lp_comparator

****/

/********
  typedef struct {
    nrf_lpcomp_config_t    hal;                // LPCOMP HAL configuration
    nrf_lpcomp_input_t     input;              // Input to be monitored
    uint8_t                interrupt_priority; // LPCOMP interrupt priority
  } nrf_drv_lpcomp_config_t;
**********/

// to used 1/2 VDD as ref
// lp_comparator_start(A0, REF_8_16Vdd, handler)
// used ADC MUX so cannot read ADC from another pin while using comparator
// call lp_comparator_stop first
// can use getChipTemperature() while lp_comparator is active
// NOTE ulPin is A0 to A7 mapped to chip pin from g_ADigitalPinMap
// for NanoV2 and Skylab NanoV2 replacement only A0 to A5 available
uint32_t lp_comparator_start(uint32_t ulPin, lp_ref refVolts, void (*handler_fun)(int) ) {

  if (handler_fun == NULL) {
    return NRF_ERROR_INVALID_PARAM;
  }
  _lpcomp_handler = (pfod_lpcomp_handler_fun)handler_fun;
  nrf_lpcomp_ref_t triggerLevel = (nrf_lpcomp_ref_t)refVolts;
  
  nrf_lpcomp_input_t input = NRF_LPCOMP_INPUT_0;
  ulPin = g_ADigitalPinMap[ulPin]; // picked up from board variant.cpp
  if (ulPin == 2) {
    input = NRF_LPCOMP_INPUT_0; // Input 0
  } else if (ulPin == 3) {
    input = NRF_LPCOMP_INPUT_1; // Input 1
  } else if (ulPin == 4 ) {
    input = NRF_LPCOMP_INPUT_2; // Input 2
  } else if (ulPin == 5) {
    input = NRF_LPCOMP_INPUT_3; // Input 3
  } else if (ulPin == 28) {
    input = NRF_LPCOMP_INPUT_4; // Input 4
  } else if (ulPin == 29) {
    input = NRF_LPCOMP_INPUT_5; // Input 5
  } else if (ulPin == 30) {
    input = NRF_LPCOMP_INPUT_6; // Input 6
  } else if (ulPin == 31) {
    input = NRF_LPCOMP_INPUT_7;  // Input 7
  } else {
    // invalid pin do not initialize
    return NRF_ERROR_INVALID_PARAM; // invalid input arg
  }
  nrf_lpcomp_config_t config_trigger;
  config_trigger.reference = triggerLevel; // default cross
  config_trigger.detection = NRF_LPCOMP_DETECT_CROSS; // this will actually do up/down/ready
  config_trigger.hyst =  NRF_LPCOMP_HYST_50mV;

  nrf_drv_lpcomp_config_t config;
  config.hal =  config_trigger;
  config.input   = input;
  config.interrupt_priority = LPCOMP_CONFIG_IRQ_PRIORITY;                                                                  \

  // initialize LPCOMP driver, from this point LPCOMP will be active and provided
  // event handler will be executed when defined action is detected
  // NOTE: nrf_drv_lpcomp_init has been modified to translate
  // NRF_LPCOMP_DETECT_CROSS into 
  // LPCOMP_INTENSET_UP_Msk|LPCOMP_INTENSET_DOWN_Msk|LPCOMP_INTENSET_READY_Msk
  uint32_t err_code = nrf_drv_lpcomp_init(&config, pfod_lpcomp_event_handler);

  if (err_code == 0) {
    nrf_drv_lpcomp_enable();
  } else {
  	nrf_drv_lpcomp_uninit();
  }
  return err_code;
}
