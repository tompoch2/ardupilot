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
#include "AP_Logger/AP_Logger.h"
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
    _dev_abs = hal.i2c_mgr->get_device(bus, 0x27);
    if (!_dev) {
        return false;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->set_retries(10);
    _measure();
    hal.scheduler->delay(10);
    _collect();

    WITH_SEMAPHORE(_dev_abs->get_semaphore());
    _dev_abs->set_retries(10);
    _measure_abs();
    hal.scheduler->delay(10);
    _collect_abs();

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
    _dev->set_device_type(uint8_t(DevType::AUAV));
    set_bus_id(_dev->get_bus_id());

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AUAV AIRSPEED[%u]: Found bus %u addr 0x%02x", get_instance(), _dev->bus_num(), _dev->get_bus_address());

    // drop to 2 retries for runtime
    _read_coefficients();
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

// read the values from the sensor
void AP_Airspeed_AUAV::_collect()
{
    _measurement_started_ms = 0; // It should always get reset by _measure. This is a safety to handle failures of i2c bus
    uint8_t inbuf[7];
    if (!_dev->read((uint8_t *)&inbuf, sizeof(inbuf))) {
        return;
    }
    const int32_t Tref_Counts = 7576807; // temperature counts at 25C
    const float TC50Scale = 100.0f * 100.0f * 167772.2f; // scale TC50 to 1.0% FS0
    float AP3, BP2, CP, Corr, Pcorr, Pdiff, TC50, Pnfso, Tcorr, PCorrt;
    int32_t iPraw, Tdiff, iTemp;
    uint32_t PComp;

    // Convert unsigned 24-bit pressure value to signed +/- 23-bit:
    iPraw = (inbuf[1]<<16) + (inbuf[2]<<8) + inbuf[3] - 0x800000;
    // Convert signed 23-bit valu11e to float, normalized to +/- 1.0:
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
    Pnfso = (Pcorr + 1.0f)/2.0f;
    //TC50: Select High/Low, based on current temp above/below 25C:
    if (Tdiff > 0)
        TC50 = D_TC50H;
    else
        TC50= D_TC50L;
    // Find absolute difference between midrange and reading (abs(Pnfso-0.5)):
    if (Pnfso > 0.5)
        Pdiff = Pnfso - 0.5;
    else
        Pdiff = 0.5f - Pnfso;
    Tcorr = (1.0f - (D_Es * 2.5f * Pdiff)) * Tdiff * TC50 / TC50Scale;
    PCorrt = Pnfso - Tcorr; // corrected P: float, [0 to +1.0)
    PComp = (uint32_t) (PCorrt * (float)0xFFFFFF);
    pressure_digital = PComp;
    _last_sample_time_ms = AP_HAL::millis();
}

void AP_Airspeed_AUAV::_measure_abs()
{
    _measurement_started_ms = 0;
    uint8_t cmd = 0xAA;
    if (_dev_abs->transfer(&cmd, 1, nullptr, 0)) {
        _measurement_started_ms = AP_HAL::millis();
    }
}

void AP_Airspeed_AUAV::_collect_abs()
{
    _measurement_started_ms = 0; // It should always get reset by _measure. This is a safety to handle failures of i2c bus
    uint8_t inbuf[7];
    if (!_dev_abs->read((uint8_t *)&inbuf, sizeof(inbuf))) {
        return;
    }
    const int32_t Tref_Counts = 7576807; // temperature counts at 25C
    const float TC50Scale = 100.0f * 100.0f * 167772.2f; // scale TC50 to 1.0% FS0
    float AP3, BP2, CP, Corr, Pcorr, Pdiff, TC50, Pnfso, Tcorr, PCorrt;
    int32_t iPraw, Tdiff, iTemp;
    uint32_t PComp;

    // Convert unsigned 24-bit pressure value to signed +/- 23-bit:
    iPraw = (inbuf[1]<<16) + (inbuf[2]<<8) + inbuf[3] - 0x800000;
    // Convert signed 23-bit valu11e to float, normalized to +/- 1.0:
    float Pnorm = (float)iPraw; // cast to float
    Pnorm /= (float) 0x7FFFFF;
    AP3 = ALIN_A * Pnorm * Pnorm * Pnorm; // A*Pout^3
    BP2 = ALIN_B * Pnorm * Pnorm; // B*Pout^2
    CP = ALIN_C * Pnorm; // C*POut
    Corr = AP3 + BP2 + CP + ALIN_D; // Linearity correction term
    Pcorr = Pnorm + Corr; // Corrected P, range +/-1.0.

    // Compute difference from reference temperature, in sensor counts:
    iTemp = (inbuf[4]<<16) + (inbuf[5]<<8) + inbuf[6]; // 24-bit temperature
    Tdiff = iTemp - Tref_Counts; // see constant defined above.
    Pnfso = (Pcorr + 1.0f)/2.0f;
    //TC50: Select High/Low, based on current temp above/below 25C:
    if (Tdiff > 0)
        TC50 = A_TC50H;
    else
        TC50= A_TC50L;
    // Find absolute difference between midrange and reading (abs(Pnfso-0.5)):
    if (Pnfso > 0.5)
        Pdiff = Pnfso - 0.5;
    else
        Pdiff = 0.5f - Pnfso;
    Tcorr = (1.0f - (A_Es * 2.5f * Pdiff)) * Tdiff * TC50 / TC50Scale;
    PCorrt = Pnfso - Tcorr; // corrected P: float, [0 to +1.0)
    PComp = (uint32_t) (PCorrt * (float)0xFFFFFF);
    pressure_abs_L = PComp;
    _last_sample_time_ms = AP_HAL::millis();
}

uint32_t AP_Airspeed_AUAV::_read_register(uint8_t cmd)
{
    uint8_t raw_bytes1[3];
    if (!_dev->transfer(&cmd,1,(uint8_t *)&raw_bytes1, sizeof(raw_bytes1))) {
        return 0;
    }
    uint8_t raw_bytes2[3];
    uint8_t cmd2 = cmd + 1;
    if (!_dev->transfer(&cmd2,1,(uint8_t *)&raw_bytes2, sizeof(raw_bytes2))) {
        return 0;
    }
    uint32_t result = ((uint32_t)raw_bytes1[1] << 24) | ((uint32_t)raw_bytes1[2] << 16) | ((uint32_t)raw_bytes2[1] << 8) | (uint32_t)raw_bytes2[2];
    return result;
}

uint32_t AP_Airspeed_AUAV::_read_register_abs(uint8_t cmd)
{
    uint8_t raw_bytes1[3];
    if (!_dev_abs->transfer(&cmd,1,(uint8_t *)&raw_bytes1, sizeof(raw_bytes1))) {
        return 0;
    }
    uint8_t raw_bytes2[3];
    uint8_t cmd2 = cmd + 1;
    if (!_dev_abs->transfer(&cmd2,1,(uint8_t *)&raw_bytes2, sizeof(raw_bytes2))) {
        return 0;
    }
    uint32_t result = ((uint32_t)raw_bytes1[1] << 24) | ((uint32_t)raw_bytes1[2] << 16) | ((uint32_t)raw_bytes2[1] << 8) | (uint32_t)raw_bytes2[2];
    return result;
}

bool AP_Airspeed_AUAV::_read_coefficients()
{
    // Differential Coefficients
    int32_t i32A = 0, i32B =0, i32C =0, i32D=0, i32TC50HLE=0;
    int8_t i8TC50H = 0, i8TC50L = 0, i8Es = 0;
    i32A = _read_register(0x2B);
    DLIN_A = ((float)(i32A))/((float)(0x7FFFFFFF));

    i32B = _read_register(0x2D);
    DLIN_B = (float)(i32B)/(float)(0x7FFFFFFF);

    i32C = _read_register(0x2F);
    DLIN_C = (float)(i32C)/(float)(0x7FFFFFFF);

    i32D = _read_register(0x31);
    DLIN_D = (float)(i32D)/(float)(0x7FFFFFFF);

    i32TC50HLE = _read_register(0x33);
    i8TC50H = (i32TC50HLE >> 24) & 0xFF; // 55 H
    i8TC50L = (i32TC50HLE >> 16) & 0xFF; // 55 L
    i8Es = (i32TC50HLE ) & 0xFF; // 56 L
    D_Es = (float)(i8Es)/(float)(0x7F);
    D_TC50H = (float)(i8TC50H)/(float)(0x7F);
    D_TC50L = (float)(i8TC50L)/(float)(0x7F);

    // Absolute Coefficients
    int32_t ai32A = 0, ai32B =0, ai32C =0, ai32D=0, ai32TC50HLE=0;
    int8_t ai8TC50H = 0, ai8TC50L = 0, ai8Es = 0;
    // These i32 Reads return 2 register values merged as int32
    // i32 then normalized to +/- 1.0
    ai32A = _read_register_abs(0x2F); // returns 47 | 48 as int32
    ALIN_A = ((float)(ai32A))/((float)(0x7FFFFFFF));
    ai32B = _read_register_abs(0x31); // returns 49 | 50 as int32
    ALIN_B = (float)(ai32B)/(float)(0x7FFFFFFF);
    ai32C = _read_register_abs(0x33); // returns 51 | 52 as int32
    ALIN_C = (float)(ai32C)/(float)(0x7FFFFFFF);
    ai32D = _read_register_abs(0x35); // returns 53 | 54 as int32
    ALIN_D = (float)(ai32D)/(float)(0x7FFFFFFF);
    ai32TC50HLE = _read_register_abs(0x37); // 55: TC50H | TC50L
    ai8TC50H = (ai32TC50HLE >> 24) & 0xFF; // 55 H
    ai8TC50L = (ai32TC50HLE >> 16) & 0xFF; // 55 L
    ai8Es = (ai32TC50HLE ) & 0xFF; // 56 L
    A_Es = (float)(ai8Es)/(float)(0x7F);
    A_TC50H = (float)(ai8TC50H)/(float)(0x7F);
    A_TC50L = (float)(ai8TC50L)/(float)(0x7F);

    return true; //Need to actually check
}

// 50Hz timer
void AP_Airspeed_AUAV::_timer()
{

    if (_measurement_started_ms == 0) {
        _measure_abs();
        _measure();
    }

    if ((AP_HAL::millis() - _measurement_started_ms) > 10) {
        _collect();
        _collect_abs();
        // start a new measurement
        _measure();
        _measure_abs(); // Start abs pressure measurement
    }
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_AUAV::get_differential_pressure(float &_pressure)
{
    WITH_SEMAPHORE(sem);
    _pressure = 248.8f*1.25f*((pressure_digital-8388608)/16777216.0f)*20;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_AUAV::get_temperature(float &_temperature)
{
    WITH_SEMAPHORE(sem);
    _temperature = 0;
    return true;
}

#endif  // AP_Airspeed_AUAV_ENABLED
