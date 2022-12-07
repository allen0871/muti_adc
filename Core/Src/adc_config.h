#ifndef ADC_CONFIG
#define ADC_CONFIG

#define NPLC 10
#define NPLCCT (400*NPLC)
#define RUNDOWN 40

#define H25MHZ
//#define H26MHZ
#ifdef H25MHZ
#define SYSMHZ  25
#else
#define SYSMHZ  26
#endif

#define TIMCLKDIV (SYSMHZ*50)
#define TZBASE  (SYSMHZ*20)
#define TZCLK  (TZBASE-1)
#define TZS  10
#define TZW  TZBASE-TZS
#define TZCC TZBASE-190
#define ADH  2800
#define ADL  1200

#endif
