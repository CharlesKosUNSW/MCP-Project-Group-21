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

typedef enum {
    ROUTE_A,
    ROUTE_B
}   Course_Route;

// Reset search FSM to initial state.
void SearchFSM_Reset(void); 

// Getter function for course type
Course_Route SearchFSM_Get_Course(void);

// Run one step of search FSM , call every loop while in SEARCH
SearchFSM_Result SearchFSM_Run (Kobuki_Typedef *kobuki , US_Typedef *object );

#endif
