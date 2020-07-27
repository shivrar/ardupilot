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
    _dev = std::move(hal.i2c_mgr->get_device(bus, POZYX_I2C_ADDRESS));
    if (!_dev) {
        return;
    }

    hal.console->printf("Starting POZYX device reading on I2C\n");
    // lets initialise the actual tag

    uint8_t whoami;
    uint8_t regs[3];
    regs[2] = 0x12;


    hal.scheduler->delay(250); // This might not be the cleanest but I'll look for something cleaner later
    if(this->reg_read(POZYX_WHO_AM_I, regs, 3) == POZYX_FAILURE){
        hal.console->printf("Failure to read info on shield\n");
        return;
    }
    whoami = regs[0];
    hal.console->printf("Who am I Pozyx:%x \n", whoami);

    this->reg_function(POZYX_DEVICES_CLEAR, nullptr, 0, nullptr, 1);


    //    TODO: Look at coding these in a parameter
    uint16_t anchors[4] = {0x6E2B, 0x676C, 0x670A, 0x6E22};     // the network id of the anchors: change these to the network ids of your anchors.
    Vector3i anchor_pos[4] = {Vector3i(0, 645, 1214), Vector3i(2737, -410, 1913), Vector3i(3651, 4120, 1853), Vector3i(1541, 4450, 1775)};

    //Iterate and set the positions in the tag. Look at possibly not hardcoding this
    for(int i = 0; i < 4; i++){
        uint8_t dev_params[15];

        Vector3i curr_anc_pos = anchor_pos[i];

        int32_t x = curr_anc_pos.x;
        int32_t y = curr_anc_pos.y;
        int32_t z = curr_anc_pos.z;


    //  Lord forgive me for I have memcpy'ed again
        memcpy(dev_params, anchors+i, 2); //First 2 bytes refer to the Network id
        dev_params[2] = 1; //Third byte to represent an anchor
        memcpy(dev_params+3, &x, 4); // Next 4 bytes is the x position
        memcpy(dev_params+7, &y, 4); // Next 4 bytes is the y position
        memcpy(dev_params+11, &z, 4); // Next 4 bytes is the z position
        bool status = this->reg_function(POZYX_DEVICE_ADD, dev_params, 15, nullptr, 1);
        if(!status){
            hal.console->printf("Error Setting Anchor Positions\n");
        }
    }

    if(this->reg_write(POZYX_POS_INTERVAL, (uint8_t*)&interval_ms, 2))
    {
        hal.console->printf("Error Setting Tag to continuous positioning\n");
    }

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

int AP_Beacon_PozyxI2C::reg_read(uint8_t reg_address, uint8_t *pData, uint32_t size)
{
//    This preliminary test is a good precursory check but there's some weird logic in the macro
//    if(!IS_REG_READABLE(reg_address))
//        return POZYX_FAILURE;
// THIS WORKS !!!!
    WITH_SEMAPHORE(_dev->get_semaphore());
    bool status = this->_dev->read_registers(reg_address, pData, size)? POZYX_SUCCESS : POZYX_FAILURE;
    return status;
}

int AP_Beacon_PozyxI2C::reg_write(uint8_t reg_address, uint8_t *params, int param_size)
{
    uint8_t status;

    uint8_t write_data[param_size+1];
    write_data[0] = reg_address;
    memcpy(write_data+1, params, param_size);

    WITH_SEMAPHORE(_dev->get_semaphore());
    status = this->write_read(write_data, param_size+1, nullptr, 0);
    return status? POZYX_SUCCESS:POZYX_FAILURE;
}

int AP_Beacon_PozyxI2C::reg_function(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, uint32_t size)
{
    // Call a register function using i2c with given parameters, the data from the function is stored in pData
    uint8_t status;

    // First let's write the relevant info
    uint8_t write_data[param_size+1];
    write_data[0] = reg_address;
    // As the pozyx folks said this feels clunky but it's probably better than for looping it for the time being
    memcpy(write_data+1, params, param_size);
    uint8_t read_data[size+1];

    WITH_SEMAPHORE(_dev->get_semaphore());
    status = this->write_read(write_data, param_size+1, read_data, size+1);
    if(status == POZYX_FAILURE) {
        return status;
    }
    memcpy(pData, read_data+1, size);
    // the first byte that a function returns is always it's success indicator, so we simply pass this through
    return read_data[0];
}

bool AP_Beacon_PozyxI2C::write_bytes(uint8_t *write_buf_u8, uint32_t len_u8)
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    return _dev->transfer(write_buf_u8, len_u8, NULL, 0);
}


int AP_Beacon_PozyxI2C::write_read(uint8_t* write_data, int write_len, uint8_t* read_data, int read_len)
{
//    Core function write and read functionality
    bool status;
    WITH_SEMAPHORE(_dev->get_semaphore());
    status = _dev->transfer(write_data, write_len, read_data, read_len)?  POZYX_SUCCESS : POZYX_FAILURE;
    return status;

}