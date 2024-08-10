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
  backend driver for airspeed from a I2C AUAV sensor
 */
#include "AP_Airspeed_AUAV.h"

#if AP_AIRSPEED_AUAV_ENABLED

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

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AUAV PROBE");
    return _last_sample_time_ms != 0;

}

// probe and initialise the sensor
bool AP_Airspeed_AUAV::init()
{
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AUAV INIT");

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
    _dev->set_device_type(uint8_t(DevType::AUAV));
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
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AUAV MEASURE");
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
    return 0.0f;
}

/*
  convert raw temperature to temperature in degrees C
 */
float AP_Airspeed_AUAV::_get_temperature(int16_t dT_raw) const
{
    return 0.0f;
}

// read the values from the sensor
void AP_Airspeed_AUAV::_collect()
{
    _measurement_started_ms = 0; // It should always get reset by _measure. This is a safety to handle failures of i2c bus
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AUAV COLLECT");
    uint8_t inbuf[7];
    if (!_dev->read((uint8_t *)&inbuf, sizeof(inbuf))) {
        return;
    }
    const int32_t Tref_Counts = 7576807; // temperature counts at 25C
    const float TC50Scale = 100.0 * 100.0 * 167772.2; // scale TC50 to 1.0% FS0
    float AP3, BP2, CP, Corr, Pcorr, Pdiff, TC50, Pnfso, Tcorr, PCorrt;
    int32_t iPraw, Tdiff, iTemp;
    // uint32_t PComp;

    // Convert unsigned 24-bit pressure value to signed +/- 23-bit:
    iPraw = (inbuf[1]<<16) + (inbuf[2]<<8) + inbuf[3] - 0x800000;
    // Convert signed 23-bit value to float, normalized to +/- 1.0:
    float Pnorm = (float)iPraw; // cast to float
    Pnorm /= (float) 0x7FFFFF;
    AP3 = DLIN_A * Pnorm * Pnorm * Pnorm; // A*Pout^3
    BP2 = DLIN_B * Pnorm * Pnorm; // B*Pout^2
    CP = DLIN_C * Pnorm; // C*POut
    Corr = AP3 + BP2 + CP + DLIN_D; // Linearity correction term
    Pcorr = Pnorm + Corr; // Corrected P, range +/-1.0.

    // Compute difference from reference temperature, in sensor counts:
    iTemp = (inbuf[4]<<16) + (inbuf[5]<<8) + inbuf[6]; // 24-bit temperature
    Tdiff = iTemp - Tref_Counts; // see constant defined above.
    Pnfso = (Pcorr + 1.0)/2.0;
    //TC50: Select High/Low, based on current temp above/below 25C:
    if (Tdiff > 0)
        TC50 = D_TC50H;
    else
        TC50 = D_TC50L;
    // Find absolute difference between midrange and reading (abs(Pnfso-0.5)):
    if (Pnfso > 0.5)
        Pdiff = Pnfso - 0.5;
    else
        Pdiff = 0.5 - Pnfso;
    Tcorr = (1.0 - (D_Es * 2.5 * Pdiff)) * Tdiff * TC50 / TC50Scale;
    PCorrt = Pnfso - Tcorr; // corrected P: float, [0 to +1.0)
    pressure = PCorrt;
    // PComp = (uint32_t) (PCorrt * (float)0xFFFFFF);
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
}

bool AP_Airspeed_AUAV::_read_coefficients()
{
    // uint8_t raw_bytes[3];
    // uint8_t cmd1 = 0x31;
    // if (!_dev->transfer(&cmd1,1,(uint8_t *)&raw_bytes, sizeof(raw_bytes))) {
    //     return false;
    // }
    // uint8_t cmd2 = 0x31;
    // if (!_dev->transfer(&cmd2,1,(uint8_t *)&raw_bytes, sizeof(raw_bytes))) {
    //     return false;
    // }
    // const int32_t Tref_Counts = 7576807; // temperature counts at 25C
    // const float TC50Scale = 100.0 * 100.0 * 167772.2; // scale TC50 to 1.0% FS0
    // float AP3, BP2, CP, Corr, Pcorr, Pdiff, TCadj, TC50, Pnfso, Tcorr, Pcorrt;
    // int32_t iPraw, Tdiff, iTemp, iPCorrected;
    // uint32_t PComp;

    // // Convert unsigned 24-bit pressure value to signed +/- 23-bit:
    // iPraw = (inbuf[1]<<16) + (inbuf[2]<<8) + inbuf[3] - 0x800000;
    // // Convert signed 23-bit value to float, normalized to +/- 1.0:
    // Pnorm = (float)iPraw; // cast to float
    // Pnorm /= (float) 0x7FFFFF;
    // AP3 = DLIN_A * Pnorm * Pnorm * Pnorm; // A*Pout^3
    // BP2 = DLIN_B * Pnorm * Pnorm; // B*Pout^2
    // CP = DLIN_C * Pnorm; // C*POut
    // Corr = AP3 + BP2 + CP + DLIN_D; // Linearity correction term
    // Pcorr = Pnorm + Corr; // Corrected P, range +/-1.0.

    // // Compute difference from reference temperature, in sensor counts:
    // iTemp = (inbuf[4]<<16) + (inbuf[5]<<8) + inbuf[6]; // 24-bit temperature
    // Tdiff = iTemp – Tref_Counts; // see constant defined above.
    // Pnfso = (Pcorr + 1.0)/2.0;
    // //TC50: Select High/Low, based on current temp above/below 25C:
    // if (Tdiff > 0)
    //     TC50 = D_TC50H;
    // else
    //     TC50 = D_TC50L;
    // // Find absolute difference between midrange and reading (abs(Pnfso-0.5)):
    // if (Pnfso > 0.5)
    //     Pdiff = Pnfso - 0.5;
    // else
    //     Pdiff = 0.5 - Pnfso;
    return false;
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
bool AP_Airspeed_AUAV::get_differential_pressure(float &_pressure)
{
    WITH_SEMAPHORE(sem);
    
    _pressure = 250+1.25*(pressure-(0.1f*pow(2,23))/pow(2,24))*1000;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_AUAV::get_temperature(float &_temperature)
{
    WITH_SEMAPHORE(sem);
    _temperature = 240;
    return true;
}

#endif  // AP_Airspeed_AUAV_ENABLED
