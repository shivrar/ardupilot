/*
    Created by shivan on 7/15/20.
    Simple i2c ifc test for the pozyx tag.
    This script should reproduce the functionality of the ready_to_localise.ino file provided by the Pozyx Arduino
    library. However it will use the library ifc being built into the ardupilot instead.

*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Beacon/AP_Beacon_PozyxI2C.h>


const AP_HAL::HAL& hal = AP_HAL::get_HAL();
//We can hardcode the anchor positions here for the time being
//TODO: Look at putting the the anchors configurable via parameters. Possibly reuse the memory for the current ones.
void setup();
void loop();

void setup(void)
{
    // Setup the I2C bus and send the configuration data for the tag
    /* Required functionality to build into the library:
     * -Connect to bus and return checkable data if successful or not.
     * -Configure the pozyx tag with the relvenat acnhor, algorithm, dimension etc.
     * */
    hal.console->printf("hello world\n");
}

void loop(void)
{
    // Do positioning periodically and serial print it -> we can look at this using a serial moniter or mavproxy
    hal.scheduler->delay(1000);
    hal.console->printf("*\n");
}

AP_HAL_MAIN();
