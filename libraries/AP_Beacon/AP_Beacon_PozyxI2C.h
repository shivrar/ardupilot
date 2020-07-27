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

    // This is essentially the main function call we'll use to get the relevant data from the pozyx
    //TODO: Check to see if this is already an implementation for this
    int reg_read(uint8_t reg_address, uint8_t *pData, uint32_t size);

    int reg_write(uint8_t reg_address, uint8_t *params, int param_size);

    int reg_function(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, uint32_t size);

    int write_read(uint8_t* write_data, int write_len, uint8_t* read_data, int read_len);

    bool write_bytes(uint8_t *write_buf_u8, uint32_t len_u8);

private:
    struct PACKED Coordinate{
        // Coordinate data will be stored as mm
        int32_t x;
        int32_t y;
        int32_t z;
        uint32_t timestamp;
    };

    uint16_t interval_ms = 1000;

    // Lets have a member variable to store the hal i2c ifc
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

};

//