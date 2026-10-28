#include <stdint.h>
#include "kobuki.h"
#include "StorageFSM.h"

// Storage parameters
#define STORAGE_DRIVE_DIST_MM   700    // mm. Distance to drive into storage. Changed from 1000

// FSM States
typedef enum {
    STORAGE_START,
    STORAGE_DRIVE,
    STORAGE_EXIT
} StorageFSM_State;

// Static variables
static StorageFSM_State storage_state;

// Reset storage FSM to initial state. Called when entering storage state from main FSM
void StorageFSM_Reset(void) {
    storage_state = STORAGE_START;
}

// Run one step of storage FSM , call every loop while in STORAGE in main FSM
StorageFSM_Result StorageFSM_Run(Kobuki_Typedef *kobuki) {
    switch(storage_state){
        case STORAGE_START:
            // Set distance to drive into storage
            Kobuki_Drive_Distance_Setpoint(kobuki->distance, STORAGE_DRIVE_DIST_MM);
            storage_state = STORAGE_DRIVE;
            break;

        case STORAGE_DRIVE:
            // Drive into storage. Once distance is reached , stop
            if(kobuki->distance_complete){
                Kobuki_Drive(STOP);
                storage_state = STORAGE_EXIT;
            } else {
                Kobuki_Drive_Distance(kobuki->distance);
            }
            break;

        case STORAGE_EXIT:
            // Exit storage FSM
            Kobuki_Drive(STOP);
            return STORAGE_DONE;
    }

    // If storage FSM is still running, return running state
    return STORAGE_RUNNING;
}