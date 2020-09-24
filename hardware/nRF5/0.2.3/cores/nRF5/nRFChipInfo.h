#ifndef _NRF_CHIP_INFO_H_
#define _NRF_CHIP_INFO_H_

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

uint32_t nRF52PartNo(void);
uint8_t* nRF52Variant(void);
int nRF52FlashKb(void);
int nRF52RamKb(void);

#ifdef __cplusplus
}
#endif

#endif // _NRF_CHIP_INFO_H_

