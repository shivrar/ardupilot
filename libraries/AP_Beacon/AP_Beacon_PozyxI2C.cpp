//
// Created by shivan on 7/14/20.
//

#include "AP_Beacon_PozyxI2C.h"

// This is a directive declaration so we'll grab hal from the relevant definition file
extern const AP_HAL::HAL& hal;


void AP_Beacon_PozyxI2C::_init(int8_t bus)
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
    //Iterate and set the positions in the tag. Look at possibly not hardcoding this
    for(int i = 0; i < sizeof(this->_anchors)/ sizeof(_anchors[0]); i++){
        uint8_t dev_params[15];

        Vector3i curr_anc_pos = _anchor_pos[i];

        int32_t x = curr_anc_pos.x;
        int32_t y = curr_anc_pos.y;
        int32_t z = curr_anc_pos.z;


    //  Lord forgive me for I have memcpy'ed again
        memcpy(dev_params, _anchors+i, 2); //First 2 bytes refer to the Network id
        dev_params[2] = 1; //Third byte to represent an anchor
        memcpy(dev_params+3, &x, 4); // Next 4 bytes is the x position
        memcpy(dev_params+7, &y, 4); // Next 4 bytes is the y position
        memcpy(dev_params+11, &z, 4); // Next 4 bytes is the z position
        //TODO: can redo this with the DO_RANGING register to both add and get the distance -> could be used for prgramtic setting
        bool status = this->reg_function(POZYX_DEVICE_ADD, dev_params, 15, nullptr, 1);
        if(!status){
            hal.console->printf("Error Setting Anchor Positions\n");
        }
        set_beacon_position((uint8_t)i, Vector3f(x/1000.0f,y/1000.0f,z/1000.0f));
    }

//    uint8_t protocol = POZYX_RANGE_PROTOCOL_PRECISION;
//    this->reg_write(POZYX_RANGE_PROTOCOL, &protocol, 1);

    this->_update_beacons();

//    if(this->reg_write(POZYX_POS_INTERVAL, (uint8_t*)&interval_ms, 2))
//    {
//        hal.console->printf("Error Setting Tag to continuous positioning\n");
//    }
}

AP_Beacon_PozyxI2C::AP_Beacon_PozyxI2C(AP_Beacon &frontend):
    AP_Beacon_Backend(frontend)
{
    this->_init(1); //Setup the pozyx device
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
    // Update the beacon distance so they can be healthy
    this ->_update_tag();
    this->_update_beacons();
//    uint8_t interrupt_status;
//    uint8_t error_code;
//    bool error = false;
//    this->reg_read(POZYX_INT_STATUS, &interrupt_status, 1);
//
//    if(CHECK_BIT(interrupt_status, 0)  == 1){
//        this->reg_read(POZYX_ERRORCODE, &error_code, 1);
//        hal.console->printf("Error Code =%x\n", error_code);
//        error = true;
//    }
//
//    if(error){
//        hal.console->printf("Error occurred, unable to position\n");
//    }
//    else {
//        int32_t coordinates[3];
//        if (this->reg_read(POZYX_POS_X, (uint8_t * ) & coordinates, 3 * sizeof(int32_t)) == POZYX_SUCCESS) {
//            Vector3f veh_position(
//                    Vector3f(coordinates[0] / 1000.0f, coordinates[1] / 1000.0f, coordinates[2] / 1000.0f));
//            set_vehicle_position(veh_position, 0.2f); // I can get the xy covarince error buttttttt I 'm just gonna be conservative and use the 20cm accuracy instead
//            this->last_update_ms = AP_HAL::millis();
////            hal.console->printf("Beacon position set\n");
//        }
//    }
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

void AP_Beacon_PozyxI2C::_update_beacons(void)
{
    //Iterate and set the distance of the beacons if the tag is healthy
    if(this->healthy()) {
        for (int i = 0; i < sizeof(this->_anchors) / sizeof(_anchors[0]); i++) {
            //TODO: there is some weird things happening here for with the DORANGING nonsense

            float x = this->_anchor_pos[i].x / 1000.0f;
            float y = this->_anchor_pos[i].y / 1000.0f;
            float z = this->_anchor_pos[i].z / 1000.0f;

            float distance = safe_sqrt(Vector3f(x, y, z).distance_squared(this->_curr_position));
            set_beacon_distance((uint8_t) i, distance);
        }
    }
}


void AP_Beacon_PozyxI2C::_update_tag()
{
    if(!this->reg_function(POZYX_DO_POSITIONING, nullptr, 0, nullptr, 1)){
//        hal.console->printf("Error triggering positioning\n");
        return;
    }
    this->last_tag_update_ms = AP_HAL::millis();
    uint8_t interrupt_status;
    bool error = false;
    uint8_t error_code;
    this->reg_read(POZYX_INT_STATUS, &interrupt_status, 1);

//    while(CHECK_BIT(interrupt_status, 1) != 1)
//    {
//        this->reg_read(POZYX_INT_STATUS, &interrupt_status, 1);
//        if(AP_HAL::millis() - this->last_tag_update_ms > AP_BEACON_TIMEOUT_MS)
//        {
//            hal.console->printf("Positioning Has not completed\n");
//            break;
//        }
//    }

    if(CHECK_BIT(interrupt_status, 0)  == 1){
        this->reg_read(POZYX_ERRORCODE, &error_code, 1);
//        hal.console->printf("Error Code =%x\n", error_code);
        error = true;
    }

    if(error){
//        hal.console->printf("Error occurred, unable to position\n");
        return;
    }
    else {
        int32_t coordinates[3];
        if (this->reg_read(POZYX_POS_X, (uint8_t * ) & coordinates, 3 * sizeof(int32_t)) == POZYX_SUCCESS) {
            Vector3f veh_position(
                    Vector3f(coordinates[0] / 1000.0f, coordinates[1] / 1000.0f, coordinates[2] / 1000.0f));
            this->_curr_position = veh_position;
            set_vehicle_position(veh_position, 0.2f); // I can get the xy covarince error buttttttt I 'm just gonna be conservative and use the 20cm accuracy instead
            this->last_update_ms = AP_HAL::millis();
//            hal.console->printf("Beacon position set\n");
        }

    }


}

//TODO: recheck this later

//        if(this->reg_function(POZYX_DO_RANGING, (uint8_t*)_anchors+i, sizeof(uint16_t), nullptr, 1) == POZYX_FAILURE){
//            hal.console->printf("Error in triggering ranging\n");
//        }
//        this->last_beacon_update_ms = AP_HAL::millis();
//        bool error = false;
//        uint8_t error_code;
//        uint8_t interrupt_status;
//        this->reg_read(POZYX_INT_STATUS, &interrupt_status, 1);
//
//        if(CHECK_BIT(interrupt_status, 0)  == 1){
//            this->reg_read(POZYX_ERRORCODE, &error_code, 1);
//            hal.console->printf("Error Code =%x\n", error_code);
//            error = true;
//        }
//        if(error){
//            hal.console->printf("Error occurred, unable to range\n");
//        }
//        while(CHECK_BIT(interrupt_status, 4) == 0)
//        {
//            this->reg_read(POZYX_INT_STATUS, &interrupt_status, 1);
//
//            if((AP_HAL::millis() - this->last_beacon_update_ms > AP_BEACON_TIMEOUT_MS))
//            {
//                // If we are unable to get the data out for the range measurmensents don't try to set the beacon distances
//                hal.console->printf("Ranging has not finished before timeout\n");
//                return;
//            }
//        }
//        else {
//            uint8_t range_data[11];
//            if (this->reg_function(POZYX_DEVICE_GETRANGEINFO, (uint8_t*)_anchors+i, sizeof(uint16_t), range_data,
//                                   11 * sizeof(uint8_t)) == POZYX_SUCCESS) {
//                float distance;
//                memcpy(&distance, range_data + 5, sizeof(uint32_t));
//                set_beacon_distance((uint8_t) i, distance);
//            } else {
//
//                hal.console->printf("Error in retrieving and setting beacon distance\n");
//            }
