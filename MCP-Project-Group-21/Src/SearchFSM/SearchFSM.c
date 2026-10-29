#include <stdint.h>
#include "kobuki.h"
#include "ultrasonic.h"
#include "SearchFSM.h"

// Param tuning

// Search parameters
#define DETECT_RANGE_MM         700     // mm. Distance max threshold for object detection 
#define ROUTE_THRESHOLD_MM      500     // mm. Distance threshold for route planning
#define BOLLARD_TURN_DEG        90      // deg (+ ve is CCW ). 
#define HALF_TURN_TICK          18000   // Angle wrap , angle is 0.01 deg per tick
#define FULL_TURN_TICK          36000   // Angle wrap , angle is 0.01 deg per tick
#define MP_OFFSET_DEG           -20     // deg. Margin of error for midpoint calculation    -10 works
#define SEARCH_SPEED_FAST       200     // mm/s. Fast search speed                          100 works
//#define SEARCH_SPEED_FAST       360     // mm/s. Fast search speed                          100 works
#define ROUTE_A_OFFSET_DEG      -2       // Tuned angle offset for aligning with route A
#define ROUTE_B_OFFSET_DEG      -2       // Tuned angle offset for aligning with route B

// FSM States
typedef enum {
    SCAN,
    OBJECT_DETECTED,
    OBJECT_ALIGN,
    ROUTE_ALIGN,
    SEARCH_EXIT
} SearchFSM_State;

// Static variables 
static SearchFSM_State search_state;
static int32_t start_angle_ticks;       // angle when object first detected
static int32_t end_angle_ticks;         // angle when object leaves threshold
static int32_t sweep_angle_ticks;       // angle swept while object detected
static int32_t midpoint_angle_ticks;    // midpoint of sweep angle, used to rotate to object
static int32_t mp_rel_angle_deg;        // relative angle of the object in degrees
static uint16_t range_mm;               // distance to object when detected
static Course_Route Course;             // current course

// Getter function for course type
Course_Route SearchFSM_Get_Course(void) {
    return Course;
}

// Wrap angle difference to [ -18000 ,18000] ticks
static int32_t Wrap_Angle(int32_t angle_tick) {
    // If angle is greater than 180 deg , wrap to negative equivalent
    if(angle_tick > HALF_TURN_TICK){
        angle_tick -= FULL_TURN_TICK ;
    }
    // If angle is less than -180 deg , wrap to positive equivalent
    else if (angle_tick < -HALF_TURN_TICK){
        angle_tick += FULL_TURN_TICK ;
    }
    return angle_tick ;
}

// Reset search FSM to initial state. Called when entering search state from main FSM
void SearchFSM_Reset(void) {
    search_state = SCAN;
}

// Run one step of search FSM , call every loop while in SEARCH in main FSM
SearchFSM_Result SearchFSM_Run(Kobuki_Typedef *kobuki, US_Typedef *object) {
    switch(search_state){
        case SCAN:
            // Rotate until object is within threshold
            Kobuki_Rotate(SEARCH_SPEED_FAST);
            if(object->detected && (object->range_mm < DETECT_RANGE_MM)){
                start_angle_ticks = kobuki->angle;
                search_state = OBJECT_DETECTED;
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
                search_state = OBJECT_ALIGN;
            } 
            break;

        case OBJECT_ALIGN:
            //Rotate to the object midpoint
            Kobuki_Rotate_Angle(kobuki->angle);

            // Once rotation is complete, transition to next state  
            if(kobuki->rotation_complete) {
                Kobuki_Rotate(STOP);
                // Get the distance to the object
                range_mm = object->range_mm;

                if (range_mm > ROUTE_THRESHOLD_MM){
                    Course = ROUTE_A;
                    Kobuki_Rotate_Angle_Setpoint(kobuki->angle, -(BOLLARD_TURN_DEG+ROUTE_A_OFFSET_DEG));
                } else {
                    Course = ROUTE_B;
                    Kobuki_Rotate_Angle_Setpoint(kobuki->angle, (BOLLARD_TURN_DEG+ROUTE_B_OFFSET_DEG));
                }
                search_state = ROUTE_ALIGN;
            }
            break;

        case ROUTE_ALIGN:
            Kobuki_Rotate_Angle(kobuki->angle);
            if (kobuki->rotation_complete) {
                Kobuki_Rotate(STOP);
                search_state = SEARCH_EXIT;
            }
            break;

        case SEARCH_EXIT:
            // Exit search FSM
            Kobuki_Rotate(STOP);
            return SEARCH_DONE;
    }

    // If search FSM is still running, return running state
    return SEARCH_RUNNING;
}
