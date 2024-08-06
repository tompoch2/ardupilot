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

/*
  backend driver for airspeed from a I2C MS4525D0 sensor
 */
#include "AP_Airspeed_AUAV.h"

#if AP_Airspeed_AUAV_ENABLED

#define AUAV_AIRSPEED_I2C_ADDR 0x26

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <utility>

extern const AP_HAL::HAL &hal;


AP_Airspeed_AUAV::AP_Airspeed_AUAV(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
}

// probe for a sensor
bool AP_Airspeed_AUAV::probe(uint8_t bus, uint8_t address)
{
    _dev = hal.i2c_mgr->get_device(bus, address);
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    // lots of retries during probe
    _dev->set_retries(10);
    
    _measure();
    hal.scheduler->delay(10);
    _collect();

    return _last_sample_time_ms != 0;
}

// probe and initialise the sensor
bool AP_Airspeed_AUAV::init()
{
    if (bus_is_configured()) {
        // the user has configured a specific bus
        if (probe(get_bus(), AUAV_AIRSPEED_I2C_ADDR)) {
            goto found_sensor;
        }
    } else {
        // if bus is not configured then fall back to the old
        // behaviour of probing all buses, external first
        FOREACH_I2C_EXTERNAL(bus) {
            if (probe(bus, AUAV_AIRSPEED_I2C_ADDR)) {
                goto found_sensor;
            }
        }
        FOREACH_I2C_INTERNAL(bus) {
            if (probe(bus, AUAV_AIRSPEED_I2C_ADDR)) {
                goto found_sensor;
            }
        }
    }

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AUAV AIRSPEED[%u]: no sensor found", get_instance());
    return false;

found_sensor:
    _dev->set_device_type(uint8_t(DevType::MS4525));
    set_bus_id(_dev->get_bus_id());

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AUAV AIRSPEED[%u]: Found bus %u addr 0x%02x", get_instance(), _dev->bus_num(), _dev->get_bus_address());

    // drop to 2 retries for runtime
    _dev->set_retries(2);
    
    _dev->register_periodic_callback(20000,
                                     FUNCTOR_BIND_MEMBER(&AP_Airspeed_AUAV::_timer, void));
    return true;
}

// start a measurement
void AP_Airspeed_AUAV::_measure()
{
    _measurement_started_ms = 0;
    uint8_t cmd = 0xAA;
    if (_dev->transfer(&cmd, 1, nullptr, 0)) {
        _measurement_started_ms = AP_HAL::millis();
    }
}

/*
  this equation is an inversion of the equation in the
  pressure transfer function figure on page 4 of the datasheet
  
  We negate the result so that positive differential pressures
  are generated when the bottom port is used as the static
  port on the pitot and top port is used as the dynamic port
*/
float AP_Airspeed_AUAV::_get_pressure(int16_t dp_raw) const
{
    const float P_max = get_psi_range();
    
    return press;
}

/*
  convert raw temperature to temperature in degrees C
 */
float AP_Airspeed_AUAV::_get_temperature(int16_t dT_raw) const
{
    float temp  = ((200.0f * dT_raw) / 2047) - 50;
    return temp;
}

// read the values from the sensor
void AP_Airspeed_AUAV::_collect()
{
    uint8_t data[4];

    _measurement_started_ms = 0;

    _last_sample_time_ms = AP_HAL::millis();
}

/**
   correct for 5V rail voltage if the system_power ORB topic is
   available

   See http://uav.tridgell.net/MS4525/MS4525-offset.png for a graph of
   offset versus voltage for 3 sensors
 */
void AP_Airspeed_AUAV::_voltage_correction(float &diff_press_pa, float &temperature)
{
	const float slope = 65.0f;
	const float temp_slope = 0.887f;

	/*
	  apply a piecewise linear correction within range given by above graph
	 */
	float voltage_diff = hal.analogin->board_voltage() - 5.0f;

    voltage_diff = constrain_float(voltage_diff, -0.7f, 0.5f);

	diff_press_pa -= voltage_diff * slope;
	temperature -= voltage_diff * temp_slope;
}

// 50Hz timer
void AP_Airspeed_AUAV::_timer()
{
    if (_measurement_started_ms == 0) {
        _measure();
        return;
    }
    if ((AP_HAL::millis() - _measurement_started_ms) > 10) {
        _collect();
        // start a new measurement
        _measure();
    }
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_AUAV::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    if (_press_count > 0) {
        _pressure = _press_sum / _press_count;
        _press_count = 0;
        _press_sum = 0;
    }

    pressure = _pressure;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_AUAV::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    if (_temp_count > 0) {
        _temperature = _temp_sum / _temp_count;
        _temp_count = 0;
        _temp_sum = 0;
    }

    temperature = _temperature;
    return true;
}

#endif  // AP_Airspeed_AUAV_ENABLED
