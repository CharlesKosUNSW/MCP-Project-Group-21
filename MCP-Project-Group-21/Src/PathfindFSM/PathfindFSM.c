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

#define CRUISE_SPEED ((int16_t)100)
#define BACKUP_MM ((int16_t)-120)
//#define TURN_DEG ((int16_t)90)
#define TURN_DEG ((int16_t)30)
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
        if (kobuki->rotation_complete)
          state = DRIVE;
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
