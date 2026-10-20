#include <stdint.h>
#include "kobuki.h"
#include "ultrasonic.h"
#include "SearchFSM.h"

// Search parameters
#define INVESTIGATE_DIST_MM     100     // mm. Distance to drive forward to investigate object
#define DETECT_RANGE_MM         400     // mm. Distance max threshold for object detection 
#define BOLLARD_TURN_DEG        90      // deg (+ ve is CCW ). 
#define EXTERNAL_MARGIN_DEG     20      // deg. Extra turn so object is not detected again
#define HALF_TURN_TICK          18000   // Angle wrap , angle is 0.01 deg per tick
#define FULL_TURN_TICK          36000   // Angle wrap , angle is 0.01 deg per tick

// FSM States
typedef enum {
    SCAN,
    OBJECT_DETECTED,
    OBJECT_ALIGN,
    OBJECT_INVESTIGATE,
    EXTERNAL_REVERSE,
    EXTERNAL_ROTATE,
    BOLLARD_REVERSE,
    BOLLARD_ROTATE,
    SEARCH_EXIT
} SearchFSM_State;

// Static variables 
static SearchFSM_State search_state;
static int32_t start_angle_ticks;       // angle when object first detected
static int32_t end_angle_ticks;         // angle when object leaves threshold
static int32_t sweep_angle_ticks;       // angle swept while object detected
static int32_t midpoint_angle_ticks;    // midpoint of sweep angle, used to rotate to object
static int32_t setpoint_angle_ticks;    // angle to rotate to after object detected
static uint16_t start_distance_ticks;   // distance when investigate started
static int16_t travelled_dist_mm;       // distance travelled before cliff

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

void SearchFSM_Reset(void) {
    search_state = SCAN;
}

SearchFSM_Result SearchFSM_Run (Kobuki_Typedef *kobuki, US_Typedef *object) {
    switch(search_state){
        case SCAN:
            // Rotate until object is within threshold
            Kobuki_Rotate(SEARCH_SPEED);
            if(object->detected && (object->range_mm < DETECT_RANGE_MM)){
                start_angle_ticks = kobuki->angle;
                search_state = OBJECT_DETECTED;
            }
            break;

        case OBJECT_DETECTED:
            // Keep rotating until object leaves threshold or is not detected at all
            if(!(object->detected) || (object->range_mm >= DETECT_RANGE_MM)) {
                //Stop rotating
                Kobuki_Rotate(STOP);

                //Object has been passed, work out midpoint of sweep
                end_angle_ticks = kobuki->angle;
                sweep_angle_ticks = Wrap_Angle(end_angle_ticks - start_angle_ticks);
                midpoint_angle_ticks = Wrap_Angle(start_angle_ticks + (sweep_angle_ticks/2));

                //Set rotate setpoint to midpoint of sweep, then rotate to that angle
                Kobuki_Rotate_Angle_Setpoint(kobuki->angle, Wrap_Angle(midpoint_angle_ticks - end_angle_ticks)*DEG_PER_TICK);

                // Transition to next state
                search_state = OBJECT_ALIGN;
            } else {
                Kobuki_Rotate(SEARCH_SPEED);
            }
            break;

        case OBJECT_ALIGN:
            // Rotate to the setpoint angle until complete, then transition to investigate state
            if(!kobuki->rotation_complete) {
                Kobuki_Rotate_Angle(kobuki->angle);
            } else {
                // Once aligned, transition to investigate state
                start_distance_ticks = kobuki->distance;
                Kobuki_Drive_Distance_Setpoint(kobuki->distance, INVESTIGATE_DIST_MM);
                search_state = OBJECT_INVESTIGATE;
            }
            break;

        case OBJECT_INVESTIGATE:
            // Move to object. If the robot runs into an object or off a cliff, assume that the object is external
            if(kobuki->cliff || kobuki->bumper) {
                // Cliff or wall detected, object is external. Reverse back to original position
                Kobuki_Drive(STOP);
                travelled_dist_mm = ( int16_t )(( double )(( uint16_t )( kobuki->distance - start_distance_ticks )) * MM_PER_TICK );
                Kobuki_Drive_Distance_Setpoint(kobuki->distance , -travelled_dist_mm);
                search_state = EXTERNAL_REVERSE ;
            } else if (kobuki->distance_complete == 1){
                // Object is bollard. Reverse back to original position
                Kobuki_Drive_Distance_Setpoint(kobuki->distance , -INVESTIGATE_DIST_MM);
                search_state = BOLLARD_REVERSE;
            } 
            // Keep driving to setpoint
            else {
                Kobuki_Drive_Distance(kobuki->distance);
            }
            break;

        case EXTERNAL_REVERSE:
            // Reverse back to point of rotation. Set to rotate away from the external object.
            if(!(kobuki->distance_complete)){
                //Keep reversing to setpoint
                Kobuki_Drive_Distance(kobuki->distance);    
            } else {
                // Reverse complete. Set rotation point to drive away from the bollard
                setpoint_angle_ticks = Wrap_Angle(midpoint_angle_ticks + (sweep_angle_ticks/2));
                Kobuki_Rotate_Angle_Setpoint(kobuki->angle, Wrap_Angle(setpoint_angle_ticks - kobuki->angle)*DEG_PER_TICK + EXTERNAL_MARGIN_DEG);
                search_state = EXTERNAL_ROTATE ;    
            }
            break;

        case EXTERNAL_ROTATE:
            // Rotate additional amount , then back to scan
            if(!(kobuki->rotation_complete)){
                Kobuki_Rotate_Angle(kobuki->angle);
            } else {
                search_state = SCAN ;
            }
            break;

        case BOLLARD_REVERSE:
            // Reverse back to point of rotation. Once complete, rotate to the side of the bollard.
            if(!(kobuki->distance_complete)){
                //Keep reversing to setpoint
                Kobuki_Drive_Distance(kobuki->distance);    
            } else {
                // Reverse complete. Set rotation point to drive away from the bollard
                Kobuki_Rotate_Angle_Setpoint(kobuki->angle, BOLLARD_TURN_DEG);
                search_state = BOLLARD_ROTATE ;    
            }
            break;

        case BOLLARD_ROTATE:
            // Rotate away from the bollard. Exit search FSM once complete.
            if(!(kobuki->rotation_complete)){
                Kobuki_Rotate_Angle(kobuki->angle);
            } else {
                search_state = SEARCH_EXIT;
            }
            break;

        case SEARCH_EXIT:
            // Exit search FSM
            Kobuki_Drive(STOP);
            return SEARCH_DONE;
    }

    // If search FSM is still running, return running state
    return SEARCH_RUNNING;
}
