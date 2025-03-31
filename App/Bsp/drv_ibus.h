//
// Created by DAMIAN_CHEN on 25-3-30.
//

#ifndef DRV_IBUS_H
#define DRV_IBUS_H


#include "cmsis_os.h"
#include "usart.h"


#define IBUS_start1			    0x20	    // 32 bytes
#define IBUS_start2			    0x40	    // Command to set servo or motor speed is always 0x40
#define IBUS_USER_CHANNELS		6			// Use 6 channels
#define IBUS_MAX_CHANNLES		14

void ibus_init(void);
static void ibus_unpack(void);

typedef struct {
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint16_t ch4;
    uint16_t ch5;
    uint16_t ch6;
}__attribute__((packed)) fs_ia6b_ibus_t;

#endif //DRV_IBUS_H
