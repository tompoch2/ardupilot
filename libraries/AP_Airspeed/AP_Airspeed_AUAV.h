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
    void _measure();
    void _collect();
    void _timer();
    bool _read_coefficients();
    uint32_t _read_register(uint8_t cmd);
    
    uint32_t _last_sample_time_ms;
    uint32_t _measurement_started_ms;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    float range_inH2O;

    float DLIN_A , DLIN_B, DLIN_C, DLIN_D, D_Es, D_TC50H, D_TC50L; // Diff coeffs
    float pressure;
    float temp;
    bool probe(uint8_t bus, uint8_t address);
};

#endif  // AP_Airspeed_AUAV_ENABLED
