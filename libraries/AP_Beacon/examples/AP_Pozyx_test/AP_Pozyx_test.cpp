/*
    Created by shivan on 7/15/20.
    Simple i2c ifc test for the pozyx tag.
    This script should reproduce the functionality of the ready_to_localise.ino file provided by the Pozyx Arduino
    library. However it will use the library ifc being built into the ardupilot instead.

*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Beacon/AP_Beacon.h>

#include <AP_Beacon/AP_Beacon_PozyxI2C.h>

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
//Simple macro for bit checking


const AP_HAL::HAL& hal = AP_HAL::get_HAL();
//We can hardcode the anchor positions here for the time being
//TODO: Look at putting the the anchors configurable via parameters. Possibly reuse the memory for the current ones.
void setup();
void loop();
static AP_SerialManager serial_manager;
AP_HAL::OwnPtr<AP_HAL::Device> dev;

bool toggle;


AP_Beacon beacon(serial_manager);
AP_Beacon_PozyxI2C pozyx(beacon);

void setup(void)
{
    // Setup the I2C bus and send the configuration data for the tag
    /* Required functionality to build into the library:
     * -Connect to bus and return checkable data if successful or not.
     * -Configure the pozyx tag with the relvenat acnhor, algorithm, dimension etc.
     * */
    pozyx.init(1);
    toggle = true;
//    dev = std::move(hal.i2c_mgr->get_device(1, POZYX_I2C_ADDRESS));

    hal.console->printf("hello world\n");
}

void loop(void)
{
    // Do positioning periodically and serial print it -> we can look at this using a serial moniter or mavproxy
//    if(!dev->read_registers(POZYX_ACCEL_X, (uint8_t*)&acceleration_raw, 3*sizeof(int16_t))){
//    dev->read_registers(POZYX_ACCEL_X, (uint8_t*)&acceleration_raw, 3*sizeof(int16_t));
//    uint32_t bus_id;
//    bus_id = dev->get_bus_id();
//    hal.console->printf("Bus id: %lu\n",bus_id);
//    uint8_t bus_address;
//    bus_address = dev->get_bus_address();
//    hal.console->printf("Bus address: %i\n",bus_address);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//    int16_t acceleration_raw[3];
//    if(!pozyx.reg_read(POZYX_ACCEL_X, (uint8_t*)&acceleration_raw, 3*sizeof(int16_t)))
//    {
//        hal.console->printf("Failed to read data\n");
//    }
//    else{
//        hal.console->printf("Accel x: %i\n",acceleration_raw[0]);
//        hal.console->printf("Accel y: %i\n",acceleration_raw[1]);
//        hal.console->printf("Accel z: %i\n",acceleration_raw[2]);
//    }

//    uint8_t cmd; // Turn on all the the leds
//
//    if(toggle){
//        cmd = 0b01100010;
//        toggle = false;
//    } else{
//        cmd = 0b01100100;
//        toggle = true;
//    }
//
//    bool status = pozyx.reg_function(POZYX_LED_CTRL, &cmd, 1, nullptr, 1);
//    if(status==POZYX_SUCCESS)
//    {
//        hal.console->printf("Successfully ran register function, result=%i\n", status);
//    } else{
//        hal.console->printf("Failed to run function. result=%i\n", status);
//    }


    uint16_t interval;
    bool  status = pozyx.reg_read(POZYX_POS_INTERVAL, (uint8_t*)&interval, 2);

    if(status==POZYX_SUCCESS)
    {
        hal.console->printf("Successfully got device interval=%i\n", interval);
    } else{
        hal.console->printf("Failed to get interval");
    }

    hal.scheduler->delay(1000);
    hal.console->printf("*\n");
}

AP_HAL_MAIN();
