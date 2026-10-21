#ifndef STORAGEFSM_H
#define STORAGEFSM_H

#include <stdint.h>
#include "kobuki.h"
#include "ultrasonic.h"

typedef enum {
    STORAGE_RUNNING,
    STORAGE_DONE
} StorageFSM_Result;

// Function prototypes for StorageFSM
StorageFSM_Result StorageFSM_Run(Kobuki_Typedef *kobuki, US_Typedef *object);

#endif