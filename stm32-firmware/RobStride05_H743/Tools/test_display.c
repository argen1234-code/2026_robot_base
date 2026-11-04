/* Host check of the RTOS OLED page.
 *
 * Compiles the real App/app_display.c against a stubbed OLED transport, so the
 * eight rows can be read as text without a screen. It checks
 *   1. no row exceeds the 21-character budget (5x7 glyph + 1 blank column),
 *   2. every character of every row exists in Devices/oled_font.h (a missing
 *      glyph would silently render as a blank),
 *   3. the row layout stays inside the budget for extreme values.
 *
 * Build (no firmware, no hardware):
 *   gcc -std=c99 -Wall -Wextra -Werror -Wno-format-truncation \
 *       Tools/test_display.c App/app_display.c -IApp -IBSP -IDevices \
 *       -o Tools/test_display.exe && Tools/test_display.exe
 */
#include <stdio.h>
#include <string.h>
#include "app_display.h"
#include "oled_font.h"

#define COLUMNS 21U
static char page[8][COLUMNS+8U];
static unsigned too_long, bad_glyph;

void Oled_Clear(void) { memset(page,0,sizeof(page)); }

void Oled_Line(uint8_t row,const char *text)
{
 unsigned i;
 if(row>=8U) return;
 snprintf(page[row],sizeof(page[row]),"%s",text);
 if(strlen(text)>COLUMNS) too_long++;
 for(i=0U;text[i];i++) {
  unsigned j;int found=0;
  for(j=0U;j<sizeof(s_glyphs)/sizeof(s_glyphs[0]);j++)
   if(s_glyphs[j].character==text[i]) {found=1;break;}
  if(!found) bad_glyph++;
 }
}

static void dump(const char *title)
{
 unsigned i;
 printf("--- %s ---\n",title);
 for(i=0U;i<8U;i++) printf("|%-21s| %u\n",page[i],(unsigned)strlen(page[i]));
}

/* CPU shares in permille: 0.8%, 1.2%, 0.1%, 0.6% */
static AppDisplayTask tasks[4]={
 {"MOTOR", "BLK", 1436U, 8U},
 {"OLED",  "RUN", 2680U, 12U},
 {"HEALTH","BLK", 780U, 1U},
 {"LED",   "RDY", 1300U, 6U}
};

static void scenario(const char *title,const MotorStatus *motor,
                     uint32_t uptime,uint32_t heap,uint32_t heap_min,uint32_t switches)
{
 AppDisplayRtos rtos;
 rtos.uptime_s=uptime;
 rtos.heap_free_b=heap;
 rtos.heap_min_b=heap_min;
 rtos.switch_count=switches;
 rtos.task=tasks;
 rtos.task_count=4U;
 MotorStatus other=*motor;other.configured_id=126;other.velocity_rad_s=-motor->velocity_rad_s;
 AppDisplay_Draw(&rtos,motor,&other);
 dump(title);
}

int main(void)
{
 MotorStatus motor;
 memset(&motor,0,sizeof(motor));
 motor.configured_id=127;motor.online=1U;motor.feedback_age_ms=4U;
 motor.velocity_rad_s=0.052f;motor.target_rad_s=0.0f;motor.state=3U;
 scenario("running, motor online", &motor, 754U, 18630U, 17552U, 2412U);
 motor.velocity_rad_s=-33.0f;motor.target_rad_s=-33.0f;motor.state=4U;
 scenario("negative extremes", &motor, 359999U, 24576U, 24575U, 99999U);
 motor.online=0U;motor.feedback_age_ms=0xFFFFFFFFU;motor.target_rad_s=33.0f;motor.state=5U;
 scenario("offline, fault, positive target", &motor, 3725U, 1024U, 0U, 0U);
 motor.state=0U;motor.target_rad_s=0.0f;
 scenario("disabled, empty heap", &motor, 0U, 0U, 0U, 1U);

 if(too_long||bad_glyph) {
  printf("FAILED: rows too long=%u, characters without a glyph=%u\n",too_long,bad_glyph);
  return 1;
 }
 if(strncmp(page[0],"UP 00:00:00 SW 00001",20)!=0) {puts("FAILED row 0");return 1;}
 if(strncmp(page[1],"HEAP 0.0K MIN 0.0K",18)!=0) {puts("FAILED row 1");return 1;}
 if(strncmp(page[2],"M1 7F OFFLINE DIS",16)!=0) {puts("FAILED motor 1 row");return 1;}
 if(strncmp(page[4],"M2 7E OFFLINE DIS",16)!=0) {puts("FAILED motor 2 row");return 1;}
 if(strncmp(page[3],"SET  +0.00 RAD/S",16)!=0 || strcmp(page[3],page[5])) {puts("FAILED targets");return 1;}
 if(strncmp(page[6],"MOTOR  BLK 0.8 S1.4K",19)!=0) {puts("FAILED task row");return 1;}
 if(strncmp(page[7],"OLED   RUN 1.2 S2.6K",19)!=0) {puts("FAILED task row");return 1;}
 scenario("rotating task rows", &motor, 1U, 0U, 0U, 1U);
 if(strncmp(page[6],"HEALTH BLK",10)!=0 || strncmp(page[7],"LED    RDY",10)!=0) {puts("FAILED rotating task rows");return 1;}
 puts("PASS: 8 rows, 21 columns, all glyphs present");
 return 0;
}
