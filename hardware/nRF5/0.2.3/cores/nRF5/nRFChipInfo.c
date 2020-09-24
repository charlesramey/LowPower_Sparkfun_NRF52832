#include <nrf.h>
#include "nRFChipInfo.h"

/********  EXAMPLE USAGE
#include <nRFChipInfo.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  for (int i=10; i>0; i--) {
    Serial.print(i); Serial.print(' ');
    delay(500);
  }
  Serial.println();
  Serial.print("Part: nRF");   Serial.println(nRF52PartNo(),HEX);
  Serial.print("Variant: ");  Serial.println((char*)nRF52Variant());
  Serial.print("Ram: "); Serial.print(nRF52RamKb()); Serial.println("Kb");
  Serial.print("Flash: "); Serial.print(nRF52FlashKb()); Serial.println("Kb");
  Serial.println("Setup finished");

}

void loop() {
  // nothing here
}
**********************/

#ifdef __cplusplus
extern "C" {
#endif

uint32_t nRF52PartNo() {
  return NRF_FICR->INFO.PART;
}

static uint8_t variant[5]; // allow for null

uint8_t* nRF52Variant() {
  uint32_t info = NRF_FICR->INFO.VARIANT;
  uint8_t *p = (uint8_t *)&info;
  variant[0] = p[3];
  variant[1] = p[2];
  variant[2] = p[1];
  variant[3] = p[0];
  variant[4] = 0;
  return variant;
}

int nRF52FlashKb() {
  uint32_t flash = NRF_FICR->INFO.FLASH;
  return (int)flash;
}
int nRF52RamKb() {
  uint32_t ram = NRF_FICR->INFO.RAM;
  return (int)ram;
}


#ifdef __cplusplus
}
#endif
