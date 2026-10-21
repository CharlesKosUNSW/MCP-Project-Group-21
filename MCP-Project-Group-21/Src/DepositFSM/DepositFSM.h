#ifndef DEPOSITFSM_H
#define DEPOSITFSM_H 

#include <stdint.h>
#include "kobuki.h"
#include "ultrasonic.h"

typedef enum {
    DEPOSIT_RUNNING,
    DEPOSIT_DONE
} DepositFSM_Result;

// Function prototypes for DepositFSM
DepositFSM_Result DepositFSM_Run(Kobuki_Typedef *kobuki, US_Typedef *object);

#endif