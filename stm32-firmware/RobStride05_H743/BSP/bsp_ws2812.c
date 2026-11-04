#include "bsp_ws2812.h"
#include "app_config.h"
#include "main.h"

#if APP_LED_COUNT < 1 || APP_LED_COUNT > 60
#error APP_LED_COUNT_must_be_1_to_60_for_the_reserved_4KB_DMA_region
#endif
#define LED_RESET_SLOTS 322U /* >=400 us LOW, including preload pipeline. */
#define LED_WORDS (2U+APP_LED_COUNT*24U+LED_RESET_SLOTS)
#define LED_DMA_FLAGS (DMA_LIFCR_CTCIF0|DMA_LIFCR_CHTIF0|DMA_LIFCR_CTEIF0|DMA_LIFCR_CDMEIF0|DMA_LIFCR_CFEIF0)
#define LED_DMA_ERRORS (DMA_LISR_TEIF0|DMA_LISR_DMEIF0|DMA_LISR_FEIF0)

/* Scatter places this ZI section in UNINIT D2 SRAM1 at 0x30000000.
 * CRT MUST NOT zero D2 SRAM before its clock is enabled in Init().
 * Never place this buffer in DTCM; cache OFF does not make DTCM DMA-accessible. */
__attribute__((section("WS2812_DMA"),zero_init,aligned(32)))
static uint16_t waveform[LED_WORDS];
static bool ready,busy;
static uint32_t error,start_ms;
static uint16_t zero_ticks,one_ticks;

static void output_low(void)
{
 TIM3->DIER=0U;
 TIM3->CR1&=~TIM_CR1_CEN;
 TIM3->CCR1=0U;
 TIM3->EGR=TIM_EGR_UG; /* Load zero into the active CCR before stopping. */
 TIM3->SR=0U;
}
static uint32_t fail(uint32_t reason)
{
 output_low();
 DMA1_Stream0->CR&=~DMA_SxCR_EN; /* No busy wait or shared DMA reset. */
 ready=false;busy=false;error=reason;
 return LED_TX_ERROR;
}
bool BspWs2812_Init(void)
{
 RCC_ClkInitTypeDef clock;GPIO_InitTypeDef pin={0};
 uint32_t latency,hz,addr=(uint32_t)waveform;
 ready=false;busy=false;error=LED_ERR_NONE;
 if(SCB->CCR&(SCB_CCR_IC_Msk|SCB_CCR_DC_Msk)) {
  error=LED_ERR_CACHE;return false; /* Never change the user's cache policy. */
 }
 HAL_RCC_GetClockConfig(&clock,&latency);
 hz=HAL_RCC_GetPCLK1Freq();
 if(clock.APB1CLKDivider!=RCC_APB1_DIV1)hz*=2U;
 if(hz<80000000U || hz>240000000U || hz%800000U ||
    hz%1000000U || (RCC->CFGR&RCC_CFGR_TIMPRE)) {
  error=LED_ERR_CLOCK;return false; /* Preserve all system clocks. */
 }
 if(addr<0x30000000U || addr+sizeof(waveform)>0x30001000U) {
  error=LED_ERR_MEMORY;return false;
 }
 __HAL_RCC_DMA1_CLK_ENABLE();
 if(DMA1_Stream0->CR&DMA_SxCR_EN) {
  error=LED_ERR_STREAM_BUSY;return false;
 }
 __HAL_RCC_D2SRAM1_CLK_ENABLE();
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_TIM3_CLK_ENABLE();
 __DSB();
 /* Only TIM3 is reset; TIM2, CAN, I2C and other DMA streams are untouched. */
 __HAL_RCC_TIM3_FORCE_RESET();__HAL_RCC_TIM3_RELEASE_RESET();
 TIM3->PSC=0U;TIM3->ARR=hz/800000U-1U; /* Keep the strip at 800 kHz. */
 zero_ticks=(uint16_t)((hz/1000000U*32U+50U)/100U); /* ~320 ns. */
 one_ticks=(uint16_t)((hz/1000000U*85U+50U)/100U);  /* 850 ns. */
 TIM3->CCMR1=TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1PE;
 TIM3->CCER=TIM_CCER_CC1E;TIM3->CR1=TIM_CR1_ARPE;
 output_low();
 DBGMCU->APB1LFZ1|=DBGMCU_APB1LFZ1_DBG_TIM3; /* TIM2 remains unfrozen. */
 DMA1_Stream0->CR=DMA_MEMORY_TO_PERIPH|DMA_MINC_ENABLE|
                    DMA_PDATAALIGN_HALFWORD|DMA_MDATAALIGN_HALFWORD|
                    DMA_PRIORITY_LOW;
 DMA1_Stream0->FCR=0U; /* Direct mode; no interrupts. */
 DMA1_Stream0->PAR=(uint32_t)&TIM3->CCR1;
 DMA1->LIFCR=LED_DMA_FLAGS;
 DMAMUX1_Channel0->CCR=DMA_REQUEST_TIM3_UP;
 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
 pin.Pin=GPIO_PIN_6;pin.Mode=GPIO_MODE_AF_PP;pin.Pull=GPIO_PULLDOWN;
 pin.Speed=GPIO_SPEED_FREQ_HIGH;pin.Alternate=GPIO_AF2_TIM3;
 HAL_GPIO_Init(GPIOA,&pin);
 ready=true;return true;
}
uint32_t BspWs2812_Poll(uint32_t now_ms)
{
 uint32_t flags;
 if(!ready)return LED_TX_ERROR;
 if(!busy)return LED_TX_IDLE;
 flags=DMA1->LISR;
 if(flags&LED_DMA_ERRORS)return fail(LED_ERR_DMA);
 if((flags&DMA_LISR_TCIF0) && !(DMA1_Stream0->CR&DMA_SxCR_EN)) {
  /* DMA's tail already held the pin low for >=400 us. */
  output_low();DMA1->LIFCR=LED_DMA_FLAGS;busy=false;return LED_TX_IDLE;
 }
 if((uint32_t)(now_ms-start_ms)>10U)return fail(LED_ERR_TIMEOUT);
 return LED_TX_BUSY;
}
static uint8_t scale(uint8_t value,uint8_t brightness)
{
 uint32_t v=value;
#if APP_LED_GAMMA_ENABLED
 v=(v*v+127U)/255U;
#endif
 return (uint8_t)((v*brightness+127U)/255U);
}
bool BspWs2812_Send(const LedPixel *pixels,uint8_t brightness,
                    uint32_t rgb_order,uint32_t now_ms)
{
 uint32_t i,c,bit,n=2U;uint8_t bytes[3];
 if(!ready || busy || (DMA1_Stream0->CR&DMA_SxCR_EN))return false;
 waveform[0]=0U;waveform[1]=0U;
 for(i=0U;i<APP_LED_COUNT;i++) {
  bytes[0]=scale(rgb_order==1U?pixels[i].r:pixels[i].g,brightness);
  bytes[1]=scale(rgb_order==1U?pixels[i].g:pixels[i].r,brightness);
  bytes[2]=scale(pixels[i].b,brightness);
  for(c=0U;c<3U;c++)for(bit=0U;bit<8U;bit++)
   waveform[n++]=(bytes[c]&(0x80U>>bit))?one_ticks:zero_ticks;
 }
 /* Each bit: 1.25 us. T0H=.32 us, T1H=.85 us. GRB, MSB first. */
 while(n<LED_WORDS)waveform[n++]=0U; /* Bounded buffer fill, no timing wait. */
 output_low();DMA1->LIFCR=LED_DMA_FLAGS;
 DMA1_Stream0->M0AR=(uint32_t)waveform;
 DMA1_Stream0->NDTR=LED_WORDS;
 __DMB(); /* SRAM writes visible before DMA; caches remain OFF. */
 start_ms=now_ms;busy=true;
 DMA1_Stream0->CR|=DMA_SxCR_EN;
 TIM3->DIER=TIM_DIER_UDE;
 TIM3->CR1|=TIM_CR1_CEN;
 return true;
}
uint32_t BspWs2812_LastError(void) {return error;}
