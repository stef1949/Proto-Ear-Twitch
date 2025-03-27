#ifndef MAIN_H
#define MAIN_H

#include "ble.h"

#define BAUD_RATE       115200  // serial debug port baud rate

////////////////////// DEBUG MODE //////////////////////
#define DEBUG_MODE 1 // Set to true to enable debug outputs

#if DEBUG_MODE
#define DEBUG_BLE
#endif

// Function prototypes for mode-specific updates
//void updateAccelerometerMode();
//void updateRandomMode();
//void updateWagMode();

/*
/////////Options/////////
enum ModeOption {
    ACCELEROMETER_MODE,  // Use accelerometer-based control
    RANDOM_MODE,         // Use random servo positions
    WAG_MODE             // Use wagging mode
};
ModeOption wagMode = ACCELEROMETER_MODE; // Set the initial mode
*/

#endif /* MAIN_H */