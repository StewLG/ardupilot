// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 
#ifndef RSSI_H
#define RSSI_H

// Clean these up? Are all needed?
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class Rssi 
{
public:
    enum RssiType {
        RSSI_DISABLED           = 0x0000,
        RSSI_ANALOG_PIN         = 0x0001,
        RSSI_RC_CHANNEL_VALUE   = 0x0002
    };	
	
    // constructor
    Rssi();

    // destructor
    ~Rssi(void);
    
    static const struct AP_Param::GroupInfo var_info[];		
	
    // RSSI parameters
    AP_Int8     	rssi_pin;    							// Analog pin RSSI value found on
	// TODO: rssi_pin_min_range and rssi_pin_max_range
    AP_Float        rssi_range;                             // allows to set max voltage for rssi pin such as 5.0, 3.3 etc. 
    AP_Int8         rssi_channel;                           // allows rssi to be read from given channel as PWM value
    AP_Int16        rssi_channel_low_pwm_value;             // PWM value for weakest rssi signal
    AP_Int16        rssi_channel_high_pwm_value;      	    // PWM value for strongest rssi signal 
		
	// read the receiver RSSI value as an 8 bit number
	uint8_t read_receiver_rssi(RssiType rssiType);
	
private:

    // Analog Inputs
    // a pin for reading the receiver RSSI voltage. 
	AP_HAL::AnalogSource *rssi_analog_source;

    // read the RSSI value from an analog pin								
	uint8_t read_pin_rssi();

	// read the RSSI value from a PWM value on a RC channel
	uint8_t read_channel_rssi();
};

#endif // RSSI_H
	
	
