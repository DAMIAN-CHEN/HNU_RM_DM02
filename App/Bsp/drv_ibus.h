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

#define IBUS_CH_MIN_VAL         1000
#define IBUS_CH_MID_VAL         1500
#define IBUS_CH_MAX_VAL         2000

#define IBUS_SW_UP              1000
#define IBUS_SW_MID             1500
#define IBUS_SW_DN              2000


void ibus_init(void);
void ibus_unpack(void);

typedef struct {
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint16_t ch4;
    uint16_t swB;
    uint16_t swC;
} fs_ia6b_ibus_t;

/*typedef struct {
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint16_t ch4;
    uint16_t swB;
    uint16_t swC;
}__attribute__((packed)) fs_ia6b_ibus_t;*/

extern fs_ia6b_ibus_t rc_data;

#endif //DRV_IBUS_H
