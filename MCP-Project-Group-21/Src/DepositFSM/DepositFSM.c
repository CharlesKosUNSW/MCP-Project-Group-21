#include "DepositFSM.h"

// Search parameters
#define DETECT_RANGE_MM         700     // mm. Distance max threshold for object detection 
#define ROUTE_THRESHOLD_MM      475     // mm. Distance threshold for route planning
#define BOLLARD_TURN_DEG        90      // deg (+ ve is CCW ). 
#define HALF_TURN_TICK          18000   // Angle wrap , angle is 0.01 deg per tick
#define FULL_TURN_TICK          36000   // Angle wrap , angle is 0.01 deg per tick
#define MP_OFFSET_DEG           -20     // deg. Margin of error for midpoint calculation    -10 works
#define SEARCH_SPEED_FAST       200     // mm/s. Fast search speed                          100 works
#define ROUTE_A_OFFSET_DEG      -5      // Tuned angle offset for aligning with route A
#define ROUTE_B_OFFSET_DEG      -5      // Tuned angle offset for aligning with route B
#define STOP_DIST_MM            100     // mm. Required gap between robot and deposit marker
#define SENSOR_PROTRUSION_MM    0       // mm. How far the sensor sits in front of the robot's front edge
                                        // (negative if behind it). Measure this on your robot.

// FSM States
typedef enum {
    SCAN,
    OBJECT_DETECTED,
    OBJECT_ALIGN,
    DRIVE_TO_DEPOSIT,
    SEARCH_EXIT
} DepositFSM_State;

static DepositFSM_State deposit_state;
static int32_t start_angle_ticks;       // angle when object first detected
static int32_t end_angle_ticks;         // angle when object leaves threshold
static int32_t sweep_angle_ticks;       // angle swept while object detected
static int32_t midpoint_angle_ticks;    // midpoint of sweep angle, used to rotate to object
static int32_t mp_rel_angle_deg;        // relative angle of the object in degrees
static uint16_t range_mm;               // distance to object when detected

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

void DepositFSM_Reset(void) {
    deposit_state = SCAN;
}

DepositFSM_Result DepositFSM_Run(Kobuki_Typedef *kobuki , US_Typedef *object ) {

    switch(deposit_state) {
        case SCAN:
            // Rotate until object is within threshold
            Kobuki_Rotate(SEARCH_SPEED_FAST);
            if(object->detected && (object->range_mm < DETECT_RANGE_MM)){
                start_angle_ticks = kobuki->angle;
                deposit_state = OBJECT_DETECTED;
            }
            break;

        case OBJECT_DETECTED:
            // Rotate at a slower speed
            Kobuki_Rotate(SEARCH_SPEED);

            // Once object is no longer detected, calculate the midpoint of the object 
            if(!(object->detected) || (object->range_mm >= DETECT_RANGE_MM)) {
                //Stop rotating
                Kobuki_Rotate(STOP);

                //Object has been passed, work out midpoint of sweep
                end_angle_ticks = kobuki->angle;
                sweep_angle_ticks = Wrap_Angle(end_angle_ticks - start_angle_ticks);
                midpoint_angle_ticks = Wrap_Angle(start_angle_ticks + (sweep_angle_ticks/2));
                mp_rel_angle_deg =  (Wrap_Angle(midpoint_angle_ticks - end_angle_ticks)*DEG_PER_TICK)+MP_OFFSET_DEG;

                //Set rotate setpoint to midpoint of sweep, then rotate to that angle
                Kobuki_Rotate_Angle_Setpoint(kobuki->angle, mp_rel_angle_deg);

                // Transition to next state
                deposit_state = OBJECT_ALIGN;
            } 
            break;

        case OBJECT_ALIGN:
            //Rotate to the object midpoint
            Kobuki_Rotate_Angle(kobuki->angle);

            // Once rotation is complete, transition to next state  
            if(kobuki->rotation_complete) {
                Kobuki_Rotate(STOP);
                // Drive the measured range minus the 100 mm stopping gap
                range_mm = object->range_mm;
                int16_t drive_mm = (int16_t)range_mm + SENSOR_PROTRUSION_MM - STOP_DIST_MM;
                if(drive_mm > 0) {
                    Kobuki_Drive_Distance_Setpoint(kobuki->distance, drive_mm);
                    deposit_state = DRIVE_TO_DEPOSIT;
                } else {
                    deposit_state = SEARCH_EXIT;    // already close enough
                }
            }
            break;

        case DRIVE_TO_DEPOSIT:
            // If a wall is hit, return to scanning
            if(kobuki->bumper) {
                Kobuki_Drive(STOP);
                deposit_state = SCAN;
            }

            // Stop on reaching the setpoint, or immediately if a bumper is hit
            if(kobuki->distance_complete || kobuki->bumper) {
                Kobuki_Drive(STOP);
                deposit_state = SEARCH_EXIT;
            } else {
                Kobuki_Drive_Distance(kobuki->distance);
            }
            break;

        case SEARCH_EXIT:
            return DEPOSIT_DONE;
            break;
    }
    return DEPOSIT_RUNNING;
}