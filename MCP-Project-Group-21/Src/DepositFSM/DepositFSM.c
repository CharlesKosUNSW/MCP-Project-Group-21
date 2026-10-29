#include "DepositFSM.h"
#include "kobuki.h"

// Deposit parameters
#define DETECT_RANGE_MM         500     // mm. Distance max threshold for object detection
#define SWEEP_HALF_DEG          90      // deg. Only sweep +/- this either side of the entry heading
#define SWEEP_HALF_TICK         (SWEEP_HALF_DEG * 100)
#define HALF_TURN_TICK          18000   // Angle wrap , angle is 0.01 deg per tick
#define FULL_TURN_TICK          36000   // Angle wrap , angle is 0.01 deg per tick
#define DEP_MP_OFFSET_DEG       -20     // deg. Midpoint correction for sensor lag. Retune , both edges now at SEARCH_SPEED
#define STOP_DIST_MM            300     // mm. Required gap between robot and deposit marker
#define DRIVE_DEFAULT           50      //mm

// FSM States
typedef enum {
    SWEEP_START,
    SWEEP_TO_EDGE,
    SCAN,
    OBJECT_DETECTED,
    OBJECT_ALIGN,
    DRIVE_TO_DEPOSIT,
    DEPOSIT_EXIT
} DepositFSM_State;

// Static variables
static DepositFSM_State deposit_state;
static int32_t entry_angle_ticks;       // heading on arriving at the entrance , centre of the sweep
static uint8_t entry_angle_set;         // 1 once entry heading recorded
static int32_t start_angle_ticks;       // angle when object first detected
static int32_t end_angle_ticks;         // angle when object leaves threshold
static int32_t sweep_angle_ticks;       // angle swept while object detected
static int32_t midpoint_angle_ticks;    // midpoint of sweep angle, used to rotate to object
static int32_t mp_rel_angle_deg;        // relative angle of the object in degrees
static uint16_t range_mm;               // distance to object when aligned

// Wrap angle difference to [-18000, 18000] ticks
static int32_t Wrap_Angle(int32_t angle_tick) {
    if(angle_tick > HALF_TURN_TICK){
        angle_tick -= FULL_TURN_TICK;
    }
    else if (angle_tick < -HALF_TURN_TICK){
        angle_tick += FULL_TURN_TICK;
    }
    return angle_tick;
}

// Reset deposit FSM to initial state. Called when entering deposit state from main FSM
void DepositFSM_Reset(void) {
    deposit_state = SWEEP_START;
    entry_angle_set = 0;
}

// Run one step of deposit FSM , call every loop while in DEPOSIT in main FSM
DepositFSM_Result DepositFSM_Run(Kobuki_Typedef *kobuki , US_Typedef *object ) {
    int32_t bearing_ticks;      // current heading relative to entry heading
    int16_t drive_mm;           // distance to drive towards deposit

    switch(deposit_state) {
        case SWEEP_START:
            // Record entry heading once , then turn to the CW edge of the sweep
            if(!entry_angle_set){
                entry_angle_ticks = kobuki->angle;
                entry_angle_set = 1;
            }
            //Set sweep to start 90 degrees CW from entry heading
            Kobuki_Rotate_Angle_Setpoint(kobuki->angle,Wrap_Angle(entry_angle_ticks - SWEEP_HALF_TICK - kobuki->angle) * DEG_PER_TICK);
            deposit_state = SWEEP_TO_EDGE;
            break;

        case SWEEP_TO_EDGE:
            // Rotate to the CW edge of the sweep
            Kobuki_Rotate_Angle(kobuki->angle);
            if(kobuki->rotation_complete){
                Kobuki_Rotate(STOP);
                deposit_state = SCAN;
            }
            break;

        case SCAN:
            // Sweep CCW across the window until an object is within threshold
            Kobuki_Rotate(SEARCH_SPEED);
            bearing_ticks = Wrap_Angle(kobuki->angle - entry_angle_ticks);
            if(object->detected && (object->range_mm < DETECT_RANGE_MM)){
                start_angle_ticks = kobuki->angle;
                deposit_state = OBJECT_DETECTED;
            } else if(bearing_ticks > SWEEP_HALF_TICK){
                // Reached the CCW edge with nothing found , go back and sweep again
                Kobuki_Rotate(STOP);
                deposit_state = SWEEP_START;
            }
            break;

        case OBJECT_DETECTED:
            //DEBUGGING
            Kobuki_Rotate(STOP);
            Kobuki_Rotate_Angle_Setpoint(kobuki->angle, 20);
            deposit_state = OBJECT_ALIGN;
            break;

            // Keep rotating at the same speed until the object leaves threshold
            Kobuki_Rotate(SEARCH_SPEED);
            bearing_ticks = Wrap_Angle(kobuki->angle - entry_angle_ticks);

            // Once object is no longer detected, calculate the midpoint of the object
            if(!(object->detected) || (object->range_mm >= DETECT_RANGE_MM) || (bearing_ticks > SWEEP_HALF_TICK)) {
                //Stop rotating
                Kobuki_Rotate(STOP);

                //Object has been passed, work out midpoint of sweep
                end_angle_ticks = kobuki->angle;
                sweep_angle_ticks = Wrap_Angle(end_angle_ticks - start_angle_ticks);
                midpoint_angle_ticks = Wrap_Angle(start_angle_ticks + (sweep_angle_ticks/2));
                mp_rel_angle_deg = (Wrap_Angle(midpoint_angle_ticks - end_angle_ticks)*DEG_PER_TICK)+DEP_MP_OFFSET_DEG;

                //Set rotate setpoint to midpoint of sweep, then rotate to that angle
                Kobuki_Rotate_Angle_Setpoint(kobuki->angle, mp_rel_angle_deg);
                deposit_state = OBJECT_ALIGN;
            }
            break;

        case OBJECT_ALIGN:
            //Rotate to the object midpoint
            Kobuki_Rotate_Angle(kobuki->angle);

            // Once rotation is complete, work out how far to drive
            if(kobuki->rotation_complete) {
                Kobuki_Rotate(STOP);

                // Drive the measured range minus the stopping gap
                range_mm = object->range_mm;
                drive_mm = (int16_t)range_mm - STOP_DIST_MM;
                if(drive_mm > 0) {
                    Kobuki_Drive_Distance_Setpoint(kobuki->distance, drive_mm);
                    deposit_state = DRIVE_TO_DEPOSIT;
                } else {
                    deposit_state = DEPOSIT_EXIT;    // already close enough
                }
            }
            break;

        case DRIVE_TO_DEPOSIT:
            // Stop on reaching the setpoint, or immediately if the bumper hits the marker
            if(kobuki->distance_complete || kobuki->bumper) {
                Kobuki_Drive(STOP);
                deposit_state = DEPOSIT_EXIT;
            } else {
                Kobuki_Drive_Distance(kobuki->distance);
            }
            break;

        case DEPOSIT_EXIT:
            // Exit deposit FSM
            Kobuki_Drive(STOP);
            return DEPOSIT_DONE;
    }

    // If deposit FSM is still running, return running state
    return DEPOSIT_RUNNING;
}
