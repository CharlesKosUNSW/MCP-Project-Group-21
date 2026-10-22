#ifndef STORAGEFSM_H
#define STORAGEFSM_H
 
#include <stdint.h>
#include "kobuki.h"
#include "ultrasonic.h"
 
#define STORAGE_DRIVE_DISTANCE_MM   ((int16_t)1000)   // Distance to drive forward (mm)
 
typedef enum {
    STORAGE_RUNNING,
    STORAGE_DONE
} StorageFSM_Result;
 
// Function prototypes for StorageFSM
StorageFSM_Result StorageFSM_Run(Kobuki_Typedef *kobuki);
void StorageFSM_Reset(void);
 
#endif