/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x1c000, LENGTH = 0x64000
  RAM (rwx) :  ORIGIN = 0x20002080, LENGTH = 0xdf80
  UICR_NFCPINS(r) : ORIGIN = 0x1000120C, LENGTH = 0x04
}

SECTIONS
{
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
  
.uicrNfcPinsAddress : 
  { 
     KEEP(*(.uicrNfcPinsAddress)) 
  } > UICR_NFCPINS
  
} INSERT AFTER .data;

INCLUDE "nrf5x_common.ld"