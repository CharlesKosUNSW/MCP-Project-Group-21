#ifndef STORAGEFSM_H
#define STORAGEFSM_H
#include <stdint.h>
#include "kobuki.h"

// Result returned to main FSM
typedef enum {
    STORAGE_RUNNING,
    STORAGE_DONE
} StorageFSM_Result;

// Reset storage FSM to initial state. Call when entering STORAGE
void StorageFSM_Reset(void);

// Run one step of storage FSM , call every loop while in STORAGE
StorageFSM_Result StorageFSM_Run(Kobuki_Typedef *kobuki);

#endif