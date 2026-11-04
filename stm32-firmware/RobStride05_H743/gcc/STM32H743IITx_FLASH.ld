/* RobStride05 H743 GCC 链接脚本（对照 MDK-ARM/RobStride05_H743.sct）
 * FLASH 2MB @ 0x08000000
 * DTCM  128KB @ 0x20000000：所有 RW/ZI（FreeRTOS heap、任务栈、应用状态）
 * D2 SRAM1 4KB @ 0x30000000：仅 WS2812 DMA 缓冲，NOLOAD（SRAM1 时钟由 BSP 打开前不得访问）
 */
ENTRY(Reset_Handler)

_estack = ORIGIN(DTCM) + LENGTH(DTCM);

_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x1000;

MEMORY
{
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  DTCM   (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  D2SRAM (rw)  : ORIGIN = 0x30000000, LENGTH = 4K
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } >FLASH

  /* WS2812 DMA 缓冲：不加载、不清零，也不参与 CRT 初始化 */
  .ws2812_dma (NOLOAD) :
  {
    . = ALIGN(32);
    KEEP(*(WS2812_DMA))
    . = ALIGN(32);
  } >D2SRAM

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >DTCM AT> FLASH

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >DTCM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCM

  /DISCARD/ :
  {
    libc.a(*)
    libm.a(*)
    libgcc.a(*)
    libnosys.a(*)
  }
}
