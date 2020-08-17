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


#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
//Simple macro for bit checking

class AP_Beacon_PozyxI2C : public AP_Beacon_Backend
{
public:
    // Constructor
    AP_Beacon_PozyxI2C(AP_Beacon &frontend);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() override;

    // update
    void update(void) override;

    // This is essentially the main function call we'll use to get the relevant data from the pozyx
    //TODO: Check to see if this is already an implementation for this
    int reg_read(uint8_t reg_address, uint8_t *pData, uint32_t size);

    int reg_write(uint8_t reg_address, uint8_t *params, int param_size);

    int reg_function(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, uint32_t size);

    int write_read(uint8_t* write_data, int write_len, uint8_t* read_data, int read_len);

    bool write_bytes(uint8_t *write_buf_u8, uint32_t len_u8);

private:
//    uint16_t interval_ms = 100;

    // Lets have a member variable to store the hal i2c ifc
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    uint32_t last_update_ms;
    uint32_t last_beacon_update_ms;
    uint32_t last_tag_update_ms;

    Vector3f _curr_position;

    //TODO: Look at coding these in a parameter, for the time being to avoid hard coding let's do the setup via a seperate medium
//TODO: I have to leave this in for the time being for configuring the tag before use. Not sure why but I'll figure it out later.
    const uint16_t _anchors[4] = {0x6E2B, 0x676C, 0x670A, 0x6E22};     // the network id of the anchors: change these to the network ids of your anchors.
    const Vector3i _anchor_pos[4] = {Vector3i(0, 645, 1214), Vector3i(2737, -410, 1913), Vector3i(3651, 4120, 1853), Vector3i(1541, 4450, 1775)};

    // Setup the relevant stuffies for the Pozyx tag
    void _init(int8_t bus);

    void _update_beacons(void);

    void _update_tag(void);
};

//