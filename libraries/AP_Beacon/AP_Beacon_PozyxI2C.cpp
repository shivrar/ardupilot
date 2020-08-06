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

    this->reg_function(POZYX_DEVICES_CLEAR, nullptr, 0, nullptr, 1); // Leaving this in for now
//TODO: Look at coding these in a parameter, for the time being to avoid hard coding let's do the setup via a seperate medium
//TODO: I have to leave this in for the time being for configuring the tag before use. Not sure why but I'll figure it out later.
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
        set_beacon_position(i, anchor_pos[i]);
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
    // Should return true if we are constantly getting good data based on the POZYX_TIMEOUT
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// update
void AP_Beacon_PozyxI2C::update(void)
{

    //Should call the set_vehicle_position() from the parent class to pass it into whatever needs it -> look at pozyx_beacon
    //implementation with arduino

    uint8_t interrupt_status;
    uint8_t error_code;
    bool error = false;
    this->reg_read(POZYX_INT_STATUS, &interrupt_status, 1);

    if(CHECK_BIT(interrupt_status, 0)  == 1){
        this->reg_read(POZYX_ERRORCODE, &error_code, 1);
        hal.console->printf("Error Code =%x\n", error_code);
        error = true;
    }

    if(error){
        hal.console->printf("Error occurred, unable to position");
    }
    else {
        int32_t coordinates[3];
        if (this->reg_read(POZYX_POS_X, (uint8_t * ) & coordinates, 3 * sizeof(int32_t)) == POZYX_SUCCESS) {
            Vector3f veh_position(
                    Vector3f(coordinates[0] / 1000.0f, coordinates[1] / 1000.0f, coordinates[2] / 1000.0f));
            set_vehicle_position(veh_position, 0.2f); // I can get the xy covarince error buttttttt I 'm just gonna be conservative and use the 20cm accuracy instead
            this->last_update_ms = AP_HAL::millis();
        }
    }
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