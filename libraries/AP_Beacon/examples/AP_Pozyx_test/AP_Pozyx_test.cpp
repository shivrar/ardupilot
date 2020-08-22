/*
  simple test of I2C interfaces
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Beacon/AP_Beacon_PozyxI2C.h>
#include <AP_Beacon/AP_Beacon.h>
#include <AP_Common/Location.h>
#include <stdio.h>

void setup();
void loop();
void set_object_value_and_report(const void *object_pointer,
                                 const struct AP_Param::GroupInfo *group_info,
                                 const char *name, float value);

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static AP_SerialManager serial_manager;
AP_Beacon beacon{serial_manager};

// try to set the object value but provide diagnostic if it failed
void set_object_value_and_report(const void *object_pointer,
                                 const struct AP_Param::GroupInfo *group_info,
                                 const char *name, float value)
{
    if (!AP_Param::set_object_value(object_pointer, group_info, name, value)) {
        printf("WARNING: AP_Param::set object value \"%s::%s\" Failed.\n",
               group_info->name, name);
    }
}

void setup(void)
{
    set_object_value_and_report(&beacon, beacon.var_info, "_TYPE", 3.0f);
    set_object_value_and_report(&beacon, beacon.var_info, "_LATITUDE", 0);
    set_object_value_and_report(&beacon, beacon.var_info, "_LONGITUDE", 0);
    set_object_value_and_report(&beacon, beacon.var_info, "_ALT", 10);

    set_object_value_and_report(&serial_manager, serial_manager.var_info, "0_PROTOCOL", 13.0f);
    serial_manager.init();
    beacon.init();
}

void loop(void)
{
    beacon.update();
    Vector3f pos;
    Location origin;
    float accuracy = 0.2f;
    beacon.get_vehicle_position_ned(pos, accuracy);
    printf("Position: %f %f %f\n", static_cast<double>(pos.x), static_cast<double>(pos.y), static_cast<double>(pos.z));
    if (beacon.get_origin(origin)) {
        printf("Origin: %il %il %il\n", static_cast<double>(origin.lat), static_cast<double>(origin.lng), static_cast<double>(origin.alt));
    }
    else{
        printf("This is where it's dying\n");
    }

    for(int i = 0; i< 4; i++){

        AP_Beacon::BeaconState state;
        beacon.get_beacon_data(i, state);
        printf("Beacon %i, is %i with distance: %f\n", i, state.healthy, state.distance);

    }
    hal.scheduler->delay(100);
}

AP_HAL_MAIN();
