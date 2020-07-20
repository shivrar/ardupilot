//
// Created by shivan on 7/14/20.
//

#include "AP_Beacon_PozyxI2C.h"

// This is a directive declaration so we'll grab hal from the relevant definition file
extern const AP_HAL::HAL& hal;


void AP_Beacon_PozyxI2C::init(int8_t bus)
{
    if (bus < 0) {
        // default to i2c external bus
        bus = 1;
    }
    dev = std::move(hal.i2c_mgr->get_device(bus, POZYX_I2C_ADDRESS));
    if (!dev) {
        return;
    }

    hal.console->printf("Starting POZYX device reading on I2C\n");
}

AP_Beacon_PozyxI2C::AP_Beacon_PozyxI2C(AP_Beacon &frontend):
    AP_Beacon_Backend(frontend)
{

}

bool AP_Beacon_PozyxI2C::healthy()
{
    return true;
}

// update
void AP_Beacon_PozyxI2C::update(void)
{
    return;
}


// Would need to add a anchor which would need the position (x,y,z) and the ID
void AP_Beacon_PozyxI2C::add_device(const Vector3f& position, uint16_t tag_id)
{
    return;
}