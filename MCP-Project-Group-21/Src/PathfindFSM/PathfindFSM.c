#include "PathfindFSM.h"
#include "SearchFSM.h"
#include <stdint.h>
#define STM32G474xx
#include "adc.h"
#include "kobuki.h"
#include "stm32g4xx.h"
#include "uart.h"
#include "ultrasonic.h"
#include "stdbool.h"
 
#define BUTTON_0_START 0x01U
#define BUTTON_1_STOP 0x02U
 
// Kobuki bumper/cliff bit masks: bit0=left, bit1=centre, bit2=right
#define SIDE_LEFT 0x01U
#define SIDE_CENTRE 0x02U
#define SIDE_RIGHT 0x04U
 
#define CRUISE_SPEED ((int16_t)300)
#define BACKUP_MM ((int16_t)-50)
#define TURN_DEG ((int16_t)5)
#define FIRST_TURN_DEG ((int16_t)90)
#define SECOND_TURN_DEG ((int16_t)135)
#define CORNER_BACKUP_MM ((int16_t)-25)
 
typedef enum { IDLE, DRIVE, BACKUP, TURN, CORNER_BACKUP, CORNER_TURN} RobotState;
 
int16_t turn_dir = 1; // +1 = CCW, -1 = CW
static int16_t corner_dir = 1; // corner turns: +1 = CCW (layout A), -1 = CW (layout B)
static uint8_t corner_count = 0U;
RobotState state = DRIVE;
bool isRouteA = true; // Assume Route A initially
int16_t deg = 0;
 
void PathfindFSM_Reset(int16_t corner_direction) {
  corner_dir = corner_direction;
  corner_count = 0U;
  state = DRIVE;
}
 
PathfindFSM_Result PathfindFSM_Run(Kobuki_Typedef *kobuki, US_Typedef *object) {
 
    uint8_t hazard = (kobuki->bumper != 0U) || (kobuki->cliff != 0U);
    uint8_t front_cliff = (kobuki->bumper == 0U) && (((kobuki->cliff & SIDE_CENTRE) != 0U) || (((kobuki->cliff & SIDE_LEFT) != 0U) && ((kobuki->cliff & SIDE_RIGHT) != 0U)));
 
    // Global stop: B1 or wheel drop, highest priority
    if ((kobuki->button & BUTTON_1_STOP) || kobuki->wheeldrop) {
      state = IDLE;
    } else {
      switch (state) {
      case IDLE:
        if (kobuki->button & BUTTON_0_START) {
          PathfindFSM_Reset(1); // Reset with default corner direction
        }
        break;
 
 
      case DRIVE:
        if (front_cliff) {
          corner_count++;
          Kobuki_Drive_Distance_Setpoint(kobuki->distance, CORNER_BACKUP_MM);
          kobuki->distance_complete = 0U; // clear stale flag from previous move
          state = CORNER_BACKUP;
         
        } else if (hazard) {
          uint8_t s = kobuki->bumper | kobuki->cliff;
          // right hit -> turn left (CCW); left or centre -> turn right (CW)
          //turn_dir = ((s & SIDE_RIGHT) && !(s & SIDE_LEFT)) ? 1 : -1;
          if ((s & SIDE_LEFT) && !(s & SIDE_RIGHT)) {
          turn_dir = 1;
        } else {
          turn_dir = -1;
        }
          kobuki->distance_complete = 0U;
          Kobuki_Drive_Distance_Setpoint(kobuki->distance, BACKUP_MM);
          state = BACKUP;
        }
        break;
 
      case BACKUP:
        if (kobuki->distance_complete) {
          Kobuki_Rotate_Angle_Setpoint(kobuki->angle, turn_dir * TURN_DEG);
          state = TURN;
        }
        break;
 
      case CORNER_BACKUP:
      if (kobuki->distance_complete) {
        if(corner_count == 1U) {
          deg = FIRST_TURN_DEG;
        } else if(corner_count == 2U) {
          deg = SECOND_TURN_DEG;
        } else {
          deg = FIRST_TURN_DEG; // default
        }
 
        isRouteA = (SearchFSM_Get_Course() == ROUTE_A); // Determine the route based on the search FSM
        if (!isRouteA) {
          corner_dir = -1; // For Route B, turn clockwise
        } else {
          corner_dir = 1; // For Route A, turn counter-clockwise
        }
        Kobuki_Rotate_Angle_Setpoint(kobuki->angle, corner_dir * deg);
        kobuki->rotation_complete = 0U;
        state = CORNER_TURN;
      }
      break;
 
      case TURN:
        if (kobuki->rotation_complete)
          state = DRIVE;
        break;
 
      case CORNER_TURN:
      if (kobuki->rotation_complete) {
        if (corner_count == 1U) {
          state = DRIVE; // carry on to the second corner
        } else {
          // Second corner done: now facing the storage entrance
          //Kobuki_Drive_Distance_Setpoint(kobuki->distance, STORAGE_DRIVE_MM);
          kobuki->distance_complete = 0U;
          return PATHFIND_DONE;
        }
      }
      break;
      }
    }
 
    // Outputs: resend every loop, Kobuki times commands out after ~3 s
    switch (state) {
    case IDLE:
      Kobuki_Drive(STOP);
      break;
    case DRIVE:
      Kobuki_Drive(CRUISE_SPEED);
      break;
    case BACKUP:
      Kobuki_Drive_Distance(kobuki->distance);
      break;
    case CORNER_BACKUP:
      Kobuki_Drive_Distance(kobuki->distance);
      break;
    case TURN:
      Kobuki_Rotate_Angle(kobuki->angle);
      break;
    case CORNER_TURN:
      Kobuki_Rotate_Angle(kobuki->angle);
      break;
    }
 
    for (volatile uint32_t i = 0; i < 5000U; ++i)
      __NOP();
 
    return PATHFIND_RUNNING;
}

/*
#include "PathfindFSM.h"

#include <stdint.h>
#define STM32G474xx
#include "adc.h"
#include "kobuki.h"
#include "stm32g4xx.h"
#include "uart.h"
#include "ultrasonic.h"

#define BUTTON_0_START 0x01U
#define BUTTON_1_STOP 0x02U

// Kobuki bumper/cliff bit masks: bit0=left, bit1=centre, bit2=right
#define SIDE_LEFT 0x01U
#define SIDE_CENTRE 0x02U
#define SIDE_RIGHT 0x04U

#define CRUISE_SPEED ((int16_t)200)     //100
#define BACKUP_MM ((int16_t)-80)       //-120
//#define TURN_DEG ((int16_t)90)
#define TURN_DEG ((int16_t)15)          //15
#define WALL_STOP_MM 10U //react to wall before contact

typedef enum { IDLE, DRIVE, BACKUP, TURN } RobotState;

int16_t turn_dir = 1; // +1 = CCW, -1 = CW
RobotState state = DRIVE;

PathfindFSM_Result PathfindFSM_Run(Kobuki_Typedef *kobuki, US_Typedef *object) {

    uint8_t hazard = (kobuki->bumper != 0U) || (kobuki->cliff != 0U);
    uint8_t wall = (object->range_mm < WALL_STOP_MM);

    // Global stop: B1 or wheel drop, highest priority
    if ((kobuki->button & BUTTON_1_STOP) || kobuki->wheeldrop) {
      state = IDLE;
    } else {
      switch (state) {
      case IDLE:
        
        if (kobuki->button & BUTTON_0_START)
          state = DRIVE;
        break;
        
      case DRIVE:
        if (hazard) {
          uint8_t s = kobuki->bumper | kobuki->cliff;
          // right hit -> turn left (CCW); left or centre -> turn right (CW)
          //turn_dir = ((s & SIDE_RIGHT) && !(s & SIDE_LEFT)) ? 1 : -1;
          turn_dir = ((s & SIDE_RIGHT) && !(s & SIDE_LEFT)) ? -1 : 1;
          Kobuki_Drive_Distance_Setpoint(kobuki->distance, BACKUP_MM);
          state = BACKUP;
        } else if (wall) {
          turn_dir = 1; // pick a default; could compare left/right scans
          Kobuki_Rotate_Angle_Setpoint(kobuki->angle, turn_dir * TURN_DEG);
          state = TURN;
        }
        break;

      case BACKUP:
        if (kobuki->distance_complete) {
          Kobuki_Rotate_Angle_Setpoint(kobuki->angle, turn_dir * TURN_DEG);
          state = TURN;
        }
        break;

      case TURN:
        if (kobuki->rotation_complete) {
          state = DRIVE;
        } 

        break;
      }
    }

    // Outputs: resend every loop, Kobuki times commands out after ~3 s
    switch (state) {
    case IDLE:
      Kobuki_Drive(STOP);
      break;
    case DRIVE:
      Kobuki_Drive(CRUISE_SPEED);
      break;
    case BACKUP:
      Kobuki_Drive_Distance(kobuki->distance);
      break;
    case TURN:
      Kobuki_Rotate_Angle(kobuki->angle);
      break;
    }

    for (volatile uint32_t i = 0; i < 5000U; ++i)
      __NOP();

    return PATHFIND_RUNNING;
}
*/