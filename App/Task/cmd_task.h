//
// Created by MR on 24-11-10.
//

#ifndef HNU_RM_DM02_CMD_TASK_H
#define HNU_RM_DM02_CMD_TASK_H


#include "cmsis_os.h"
#include "imu_task.h"
#include "pid.h"
#include "ibus_task.h"

typedef enum {
    UAV_LOCK=0,
    UAV_UNLOCK=1,
}uav_lock_status_e;

typedef struct {
    uav_lock_status_e lock_status;

}uav_status_t;


extern uav_status_t uav_now_status;
extern uav_status_t uav_last_status;


#endif //HNU_RM_DM02_CMD_TASK_H
