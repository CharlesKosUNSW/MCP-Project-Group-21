#include <stdint.h>
#define STM32G474xx
#include "stm32g4xx.h"
#include "kobuki.h"
#include "ultrasonic.h"
#include "SearchFSM.h"
#include "PathfindFSM.h"

// Kobuki bumper / cliff bit masks
#define SIDE_RIGHT              0x01U   // bit 0
#define SIDE_CENTRE             0x02U   // bit 1
#define SIDE_LEFT               0x04U   // bit 2

// Pathfind parameters
#define CRUISE_SPEED            300     // mm/s. Drive speed between hazards ( capped at SPEED_LIMIT )
#define BACKUP_DIST_MM          -50     // mm. Reverse distance after hitting a wall or side cliff
#define NUDGE_TURN_DEG          5       // deg. Turn away from a wall or side cliff
#define CORNER_BACKUP_DIST_MM   -25     // mm. Reverse distance after reaching a corner
#define FIRST_CORNER_DEG        90      // deg. Turn at first corner
#define SECOND_CORNER_DEG       135     // deg. Turn at second corner , then facing storage entrance
#define LOOP_DELAY              5000U   // busy - wait iterations per loop

// FSM States
typedef enum {
    DRIVE,
    BACKUP,
    NUDGE_TURN,
    CORNER_BACKUP,
    CORNER_TURN
} PathfindFSM_State;

// Static variables
static PathfindFSM_State pathfind_state;
static int16_t turn_dir;                // +1 = CCW , -1 = CW. Direction of nudge turn
static int16_t corner_dir;              // +1 = CCW ( route A ) , -1 = CW ( route B )
static int16_t corner_turn_deg;         // deg. Size of current corner turn
static uint8_t corner_count;            // number of corners reached

// Reset pathfind FSM to initial state. Called when entering pathfind state from main FSM
void PathfindFSM_Reset(void) {
    pathfind_state = DRIVE;
    corner_count = 0;
}

// Run one step of pathfind FSM , call every loop while in PATHFIND in main FSM
PathfindFSM_Result PathfindFSM_Run(Kobuki_Typedef *kobuki) {
    uint8_t hazard;         // any bumper or cliff
    uint8_t front_cliff;    // cliff straight ahead , i.e. a corner
    uint8_t hit_side;       // which sides were hit

    // Corner = centre cliff , or both side cliffs , with no bumper hit
    hazard = (kobuki->bumper != 0) || (kobuki->cliff != 0);
    front_cliff = (kobuki->bumper == 0) &&
                  (((kobuki->cliff & SIDE_CENTRE) != 0) ||
                   (((kobuki->cliff & SIDE_LEFT) != 0) && ((kobuki->cliff & SIDE_RIGHT) != 0)));

    switch(pathfind_state){
        case DRIVE:
            if(front_cliff){
                // Reached a corner. Reverse slightly before turning
                corner_count++;
                Kobuki_Drive_Distance_Setpoint(kobuki->distance, CORNER_BACKUP_DIST_MM);
                Kobuki_Drive_Distance(kobuki->distance);
                pathfind_state = CORNER_BACKUP;
            } else if(hazard){
                // Hit a wall or side cliff. Turn away from the side that was hit
                hit_side = kobuki->bumper | kobuki->cliff;
                if((hit_side & SIDE_RIGHT) && !(hit_side & SIDE_LEFT)){
                    turn_dir = 1;       // right hit -> turn left ( CCW )
                } else {
                    turn_dir = -1;      // left or centre hit -> turn right ( CW )
                }
                Kobuki_Drive_Distance_Setpoint(kobuki->distance, BACKUP_DIST_MM);
                Kobuki_Drive_Distance(kobuki->distance);
                pathfind_state = BACKUP;
            } else {
                // Nothing in the way , keep driving
                Kobuki_Drive(CRUISE_SPEED);
            }
            break;

        case BACKUP:
            // Reverse , then nudge away from the hazard
            if(kobuki->distance_complete){
                Kobuki_Rotate_Angle_Setpoint(kobuki->angle, turn_dir * NUDGE_TURN_DEG);
                pathfind_state = NUDGE_TURN;
            } else {
                Kobuki_Drive_Distance(kobuki->distance);
            }
            break;

        case NUDGE_TURN:
            // Turn slightly , then carry on driving
            if(kobuki->rotation_complete){
                pathfind_state = DRIVE;
            } else {
                Kobuki_Rotate_Angle(kobuki->angle);
            }
            break;

        case CORNER_BACKUP:
            // Reverse , then turn the corner. Size of turn depends on which corner
            if(kobuki->distance_complete){
                if(corner_count == 2){
                    corner_turn_deg = SECOND_CORNER_DEG;
                } else {
                    corner_turn_deg = FIRST_CORNER_DEG;
                }

                // Direction of corner turns depends on route found in search
                if(SearchFSM_Get_Course() == ROUTE_A){
                    corner_dir = 1;     // route A -> CCW
                } else {
                    corner_dir = -1;    // route B -> CW
                }

                Kobuki_Rotate_Angle_Setpoint(kobuki->angle, corner_dir * corner_turn_deg);
                pathfind_state = CORNER_TURN;
            } else {
                Kobuki_Drive_Distance(kobuki->distance);
            }
            break;

        case CORNER_TURN:
            // Turn the corner. After the second corner , the robot faces the storage entrance
            if(kobuki->rotation_complete){
                if(corner_count == 1){
                    pathfind_state = DRIVE;
                } else {
                    return PATHFIND_DONE;
                }
            } else {
                Kobuki_Rotate_Angle(kobuki->angle);
            }
            break;
    }

    /*
    // Slow the loop down while pathfinding
    for(volatile uint32_t i = 0; i < LOOP_DELAY; i++){
        __NOP();
    }
    */

    // If pathfind FSM is still running, return running state
    return PATHFIND_RUNNING;
}