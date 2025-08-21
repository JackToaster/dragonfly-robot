#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

#define CH32V20x            1
#define CH32V003FUN_BASE	1

// Though this should be on by default we can extra force it on.
#define FUNCONF_USE_DEBUGPRINTF 1
#define FUNCONF_DEBUGPRINTF_TIMEOUT (1<<31) // Wait for a very very long time.

#define FUNCONF_SYSTICK_USE_HCLK 1

#define FUNCONF_USE_HSE 1
#define FUNCONF_USE_PLL 1
#define FUNCONF_PLL_MULTIPLIER 18
#endif

