// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"
#include <RC_Channel/RC_Channel.h>     // RC Channel Library

void Plane::init_barometer(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));    
    barometer.calibrate();

    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

void Plane::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.init();
#endif
}

/*
  read the rangefinder and update height estimate
 */
void Plane::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    if (should_log(MASK_LOG_SONAR))
        Log_Write_Sonar();

    rangefinder_height_update();
#endif
}

/*
  ask airspeed sensor for a new value
 */
void Plane::read_airspeed(void)
{
    if (airspeed.enabled()) {
        airspeed.read();
        if (should_log(MASK_LOG_IMU)) {
            Log_Write_Airspeed();
        }
        calc_airspeed_errors();

        // supply a new temperature to the barometer from the digital
        // airspeed sensor if we can
        float temperature;
        if (airspeed.get_temperature(temperature)) {
            barometer.set_external_temperature(temperature);
        }
    }

    // update smoothed airspeed estimate
    float aspeed;
    if (ahrs.airspeed_estimate(&aspeed)) {
        smoothed_airspeed = smoothed_airspeed * 0.8f + aspeed * 0.2f;
    }
}

void Plane::zero_airspeed(bool in_startup)
{
    airspeed.calibrate(in_startup);
    read_airspeed();
    // update barometric calibration with new airspeed supplied temperature
    barometer.update_calibration();
    gcs_send_text_P(SEVERITY_LOW,PSTR("zero airspeed calibrated"));
}

// read_battery - reads battery voltage and current and invokes failsafe
// should be called at 10hz
void Plane::read_battery(void)
{
    battery.read();
    compass.set_current(battery.current_amps());

    if (!usb_connected && 
        hal.util->get_soft_armed() &&
        battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        low_battery_event();
    }
}

// read the receiver rssi as an 8 bit number for mavlink
// rc_channels_scaled message
void Plane::read_receiver_rssi(void)
{
    receiver_rssi = RC_Channel::read_receiver_rssi(g.rssi_pin, g.rssi_range, rssi_analog_source, g.rssi_channel, g.rssi_channel_low_pwm_value, g.rssi_channel_high_pwm_value);
}

