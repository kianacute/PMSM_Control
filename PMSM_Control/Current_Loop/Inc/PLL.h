#ifndef __PLL_H_
#define __PLL_H_

#include <stdint.h>
#include "Motor_parameter.h"
#include "Hal_Math.h"

struct PLL
{
    Hal_PI_t PLL_PI;
    float we;
    float theta;
};

#endif // __PLL_H__
