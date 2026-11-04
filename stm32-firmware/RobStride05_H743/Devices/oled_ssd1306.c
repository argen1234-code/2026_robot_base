#include "oled_ssd1306.h"
#include "app_config.h"
#include "bsp_i2c.h"
#include "oled_font.h"
#include <string.h>
static uint8_t pixels[8][128];
static uint32_t operation,segment;
static bool send_segment(void)
{
 uint8_t cmd[4]={0,0,0,0x10};uint8_t data[129];uint32_t page=segment/2U;
 if(!(segment&1U)) {
  cmd[1]=(uint8_t)(0xB0U+page);return BspI2c_TransmitBegin(APP_OLED_ADDRESS_7BIT,cmd,sizeof(cmd));
 }
 data[0]=0x40;memcpy(data+1,pixels[page],128);
 return BspI2c_TransmitBegin(APP_OLED_ADDRESS_7BIT,data,sizeof(data));
}
bool Oled_InitBegin(void)
{
 static const uint8_t init[]={0,0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0,0x40,0x8D,0x14,
  0x20,2,0xA1,0xC8,0xDA,0x12,0x81,0x7F,0xD9,0xF1,0xDB,0x40,0xA4,0xA6,0xAF};
 if(!BspI2c_TransmitBegin(APP_OLED_ADDRESS_7BIT,init,sizeof(init)))return false;
 operation=1;return true;
}
bool Oled_FlushBegin(void)
{segment=0;if(!send_segment())return false;operation=2;return true;}
uint32_t Oled_Poll(void)
{
 uint32_t r;if(!operation)return I2C_IDLE;
 r=BspI2c_Poll();if(r==I2C_BUSY)return r;
 if(r!=I2C_OK){operation=0;return I2C_FAIL;}
 if(operation==1 || ++segment==16U){operation=0;return I2C_OK;}
 if(!send_segment()){operation=0;return I2C_FAIL;}
 return I2C_BUSY;
}
void Oled_Clear(void){memset(pixels,0,sizeof(pixels));}
void Oled_Line(uint8_t row,const char *text)
{
 uint32_t x=0;if(row>=8)return;
 for(uint32_t i=0;text[i] && x+5<128;i++,x+=6) {
  const uint8_t *g=s_glyphs[0].column;
  for(uint32_t j=0;j<sizeof(s_glyphs)/sizeof(s_glyphs[0]);j++)
   if(s_glyphs[j].character==text[i]){g=s_glyphs[j].column;break;}
  memcpy(&pixels[row][x],g,5);
 }
}
