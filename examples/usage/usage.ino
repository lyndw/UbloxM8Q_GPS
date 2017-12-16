// Example usage for UbloxM8Q_GPS library by Lynd Wieman.
// You should see AssetTracker2 for a thin interface to this driver

#include "UbloxM8Q_GPS.h"

// Initialize objects from the lib
UbloxM8Q_GPS ubloxM8Q_GPS;

void setup() {
    // Call functions on initialized library objects that require hardware
    ubloxM8Q_GPS.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    ubloxM8Q_GPS.process();
}
