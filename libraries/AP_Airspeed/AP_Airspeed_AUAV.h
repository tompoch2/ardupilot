/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_AUAV_ENABLED

/*
  backend driver for airspeed from I2C
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_AUAV : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_AUAV(AP_Airspeed &frontend, uint8_t _instance);
    ~AP_Airspeed_AUAV(void) {}
    
    // probe and initialise the sensor
    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override;

private:
    void _measure(uint8_t sensor_number);
    void _collect(uint8_t sensor_number);
    void _timer();
    bool _read_coefficients(uint8_t sensor_number);
    uint32_t _read_register(uint8_t cmd);
    void _set_multiplexer(uint8_t channel);
    float get_differential_pressure_i(uint8_t index);
    
    uint32_t _last_sample_time_ms[NUM_PARALLEL_AIRSPEED_SENSORS];
    uint32_t _measurement_started_ms[NUM_PARALLEL_AIRSPEED_SENSORS];
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_multiplexer;

    float DLIN_A[NUM_PARALLEL_AIRSPEED_SENSORS];
    float DLIN_B[NUM_PARALLEL_AIRSPEED_SENSORS];
    float DLIN_C[NUM_PARALLEL_AIRSPEED_SENSORS];
    float DLIN_D[NUM_PARALLEL_AIRSPEED_SENSORS];
    float D_Es[NUM_PARALLEL_AIRSPEED_SENSORS];
    float D_TC50H[NUM_PARALLEL_AIRSPEED_SENSORS];
    float D_TC50L[NUM_PARALLEL_AIRSPEED_SENSORS]; // Diff coeffs
    float pressure_digital[NUM_PARALLEL_AIRSPEED_SENSORS];
    float temp[NUM_PARALLEL_AIRSPEED_SENSORS];
    bool probe(uint8_t bus, uint8_t address);
};

#endif  // AP_Airspeed_AUAV_ENABLED
