//
// Created by shivan on 7/14/20.
//
/*
    This work documented consists part of the MSc. Robotics Dissertation.
     Much of the ifc would be based on reading specific registers of the Pozyx tag since most of the processing is
    already done onboard.
 */
#pragma once

#include "AP_Beacon_Backend.h"
#include "AP_Pozyx_definitions.h"
#include <AP_HAL/I2CDevice.h>

class AP_Beacon_PozyxI2C : public AP_Beacon_Backend
{
public:
    // Constructor
    AP_Beacon_PozyxI2C(AP_Beacon &frontend);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() override;

    // update
    void update(void) override;

    // Setup the relevant stuffies for the Pozyx tag
    void init(int8_t bus);

    // Would need to add a anchor which would need the position (x,y,z) and the ID
    void add_device(const Vector3f& position, uint16_t tag_id);


private:
    struct PACKED Coordinate{
        // Coordinate data will be stored as mm
        int32_t x;
        int32_t y;
        int32_t z;
        uint32_t timestamp;
    };

    // Lets have a member variable to store the hal i2c ifc
    AP_HAL::OwnPtr<AP_HAL::Device> dev;
};

//