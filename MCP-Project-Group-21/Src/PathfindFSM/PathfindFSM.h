#ifndef PATHFINDFSM_H
#define PATHFINDFSM_H

#include <stdint.h>
#include "kobuki.h"
#include "ultrasonic.h"

// Result returned to main FSM
typedef enum {
    PATHFIND_RUNNING,
    PATHFIND_DONE
} PathfindFSM_Result;

// Reset pathfind FSM to initial state. Call when entering PATHFIND
void PathfindFSM_Reset(void);

// Run one step of pathfind FSM , call every loop while in PATHFIND
PathfindFSM_Result PathfindFSM_Run(Kobuki_Typedef *kobuki);

#endif