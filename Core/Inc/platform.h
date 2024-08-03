#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include "main.h"

#define DISABLE_IRQ __disable_irq()
#define ENABLE_IRQ __enable_irq()

#ifdef __cplusplus
extern "C" {
#endif

void platform_init();

#ifdef __cplusplus
}
#endif

#endif
