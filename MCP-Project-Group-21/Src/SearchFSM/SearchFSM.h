#ifndef SEARCHFSM_H
#define SEARCHFSM_H
#include <stdint.h>
#include "kobuki.h"
#include "ultrasonic.h"

// Result returned to main FSM
typedef enum {
    SEARCH_RUNNING,
    SEARCH_DONE
} SearchFSM_Result;

// Reset search FSM to initial state.
void SearchFSM_Reset(void); 

// Run one step of search FSM , call every loop while in SEARCH
SearchFSM_Result SearchFSM_Run ( Kobuki_Typedef *kobuki , US_Typedef *object );

#endif
