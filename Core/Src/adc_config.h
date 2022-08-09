#ifndef ADC_CONFIG
#define ADC_CONFIG

#define TIMCLKDIV 1250
#define NPLC 20
#define NPLCCT (400*NPLC)
#define RUNDOWN 40

#define TZBASE  500
#define TZCLK  (TZBASE-1)
#define TZS  10
#define TZW  TZBASE-TZS
#define TZCC TZBASE-180

#define ADH  2600
#define ADL  800

#endif
