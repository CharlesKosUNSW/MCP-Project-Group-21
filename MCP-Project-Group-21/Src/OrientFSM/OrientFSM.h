#ifndef ORIENTFSM_H
#define ORIENTFSM_H

#include <stdint.h>
#include "kobuki.h"
#include "ultrasonic.h"

typedef enum {
    ORIENT_RUNNING,
    ORIENT_SUCCESS,
    ORIENT_FAIL
} OrientFSM_Result;

// Function prototypes for OrientFSM
OrientFSM_Result OrientFSM_Run(Kobuki_Typedef *kobuki, US_Typedef *object);

#endif // ORIENTFSM_H