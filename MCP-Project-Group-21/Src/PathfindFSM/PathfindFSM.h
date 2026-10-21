#ifndef PATHFINDFSM_H
#define PATHFINDFSM_H

#include <stdint.h>
#include "kobuki.h"
#include "ultrasonic.h"

typedef enum {
    PATHFIND_RUNNING,
    PATHFIND_DONE
} PathfindFSM_Result;

PathfindFSM_Result PathfindFSM_Run(Kobuki_Typedef *kobuki, US_Typedef *object);

#endif