#ifndef _TIME_H__
#define _TIME_H__

#include "common/common.h"

void TIM6_Int_Init(u16 arr, u16 psc);
void perform_heartbeat(void);

#endif                          /* _TIME_H__ */
