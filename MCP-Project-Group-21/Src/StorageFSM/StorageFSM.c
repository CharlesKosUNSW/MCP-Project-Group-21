#include "StorageFSM.h"
// Internal sub-states for the storage sub-FSM
typedef enum {
  STORAGE_SET_SETPOINT, // Record the target encoder position (runs once)
  STORAGE_DRIVING,      // Run the P controller until the target is reached
  STORAGE_FINISHED      // Motors stopped, task complete
} StorageSubState;
// Persists between calls so the sub-FSM remembers where it is
static StorageSubState storage_state = STORAGE_SET_SETPOINT;
 
void StorageFSM_Reset(void) { storage_state = STORAGE_SET_SETPOINT; }
 
StorageFSM_Result StorageFSM_Run(Kobuki_Typedef *kobuki) {
  //(void)object; // Ultrasonic not used yet. Silences unused-parameter warning
  switch (storage_state) {
  case STORAGE_SET_SETPOINT:
    // Target = current encoder position + 1000 mm (converted to ticks).
    // This also clears the internal distance_complete flag.
    Kobuki_Drive_Distance_Setpoint(kobuki->distance, STORAGE_DRIVE_DISTANCE_MM);
    storage_state = STORAGE_DRIVING;
    break;
  case STORAGE_DRIVING:
    if (kobuki->distance_complete) {
      // Target reached. Kobuki_Drive_Distance() doesn't send a stop
      // command itself, so stop explicitly.
      Kobuki_Drive(STOP);
      storage_state = STORAGE_FINISHED;
    } else {
      // P controller: speed is proportional to remaining error,
      // plus MIN_SPEED offset, saturated at SPEED_LIMIT in Kobuki_Drive()
      Kobuki_Drive_Distance(kobuki->distance);
    }
    break;
  case STORAGE_FINISHED:
    Kobuki_Drive(STOP); // Keep sending zero velocity
    return STORAGE_DONE;
  default:
    Kobuki_Drive(STOP); // Safe fallback
    storage_state = STORAGE_SET_SETPOINT;
    break;
  }
  return STORAGE_RUNNING;
}