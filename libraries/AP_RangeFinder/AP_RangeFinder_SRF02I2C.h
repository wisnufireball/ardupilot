#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

#define AP_RANGE_FINDER_SRF02I2C_DEFAULT_ADDR                   0x70
#define AP_RANGE_FINDER_SRF02I2C_REG_W_COMMAND                  0x00
#define AP_RANGE_FINDER_SRF02I2C_REG_R_CHECK_CODE               0x01
#define AP_RANGE_FINDER_SRF02I2C_REG_R_RANGE_HIGH_BYTE          0x02
#define AP_RANGE_FINDER_SRF02I2C_REG_R_RANGE_LOW_BYTE           0x03
#define AP_RANGE_FINDER_SRF02I2C_COMMAND_TAKE_RANGE_READING_CM  0x51


#define AP_RANGE_FINDER_SRF02I2C_SCALER        1.0
#define AP_RANGE_FINDER_SRF02I2C_MIN_DISTANCE  25
#define AP_RANGE_FINDER_SRF02I2C_MAX_DISTANCE  600


class AP_RangeFinder_SRF02I2C : public AP_RangeFinder_Backend
{
public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void);

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:
    // constructor
    AP_RangeFinder_SRF02I2C(RangeFinder::RangeFinder_State &_state,
                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool _init(void);
    void _timer(void);

    uint16_t _distance;
    bool _new_distance;
    
    // start a reading
    bool start_reading(void);
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
