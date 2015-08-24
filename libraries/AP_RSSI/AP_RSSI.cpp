/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include <AP_RSSI/AP_RSSI.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RSSI::var_info[] PROGMEM = {
				
    // @Param: RSSI_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: This selects an analog pin for the receiver RSSI voltage. It assumes the voltage is 5V for max rssi, 0V for minimum
    // @Values: -1:Disabled, 0:APM2 A0, 1:APM2 A1, 13:APM2 A13, 103:Pixhawk SBUS
    // @User: Standard
    //GSCALAR(rssi_pin,            "RSSI_PIN",         -1),
    AP_GROUPINFO("RSSI_PIN", 0, AP_RSSI, rssi_pin,  -1),	

    // @Param: RSSI_RANGE
    // @DisplayName: Receiver RSSI voltage range
    // @Description: Receiver RSSI voltage range
    // @Units: Volt
    // @Values: 3.3:3.3V, 5.0:5V
    // @User: Standard
    //GSCALAR(rssi_range,          "RSSI_RANGE",         5.0),
    AP_GROUPINFO("RSSI_RANGE", 1, AP_RSSI, rssi_range,  5.0),	
    
    // @Param: RSSI_CHANNEL
    // @DisplayName: Receiver RSSI channel number
    // @Description: The channel number where RSSI will be output by the radio receiver.
    // @Units: 
    // @Values: 0:Disabled,1:Channel1,2:Channel2,3:Channel3,4:Channel4,5:Channel5,6:Channel6,7:Channel7,8:Channel8
    // @User: Standard
    //GSCALAR(rssi_channel,          "RSSI_CHANNEL",         0),
    AP_GROUPINFO("RSSI_CHANNEL", 2, AP_RSSI, rssi_channel,  0),		
    
    // @Param: RSSI_CHAN_LOW
    // @DisplayName: Receiver RSSI PWM low value
    // @Description: This is the PWM value that the radio receiver will put on the RSSI_CHANNEL when the signal strength is the weakest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a lower value than RSSI_CHAN_HIGH. 
    // @Units: Microseconds
    // @Range: 0 2000
    // @User: Standard
    //GSCALAR(rssi_channel_low_pwm_value, "RSSI_CHAN_LOW", 1000), 
    AP_GROUPINFO("RSSI_CHAN_LOW", 3, AP_RSSI, rssi_channel_low_pwm_value,  1000),		
    
    // @Param: RSSI_CHAN_HIGH
    // @DisplayName: Receiver RSSI PWM high value
    // @Description: This is the PWM value that the radio receiver will put on the RSSI_CHANNEL when the signal strength is the strongest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a higher value than RSSI_CHAN_LOW. 
    // @Units: Microseconds
    // @Range: 0 2000
    // @User: Standard
    //GSCALAR(rssi_channel_high_pwm_value, "RSSI_CHAN_HIGH", 2000),  
    AP_GROUPINFO("RSSI_CHAN_HIGH", 4, AP_RSSI, rssi_channel_high_pwm_value,  2000),		
    
    AP_GROUPEND
};

// Public
// ------

// constructor
AP_RSSI::AP_RSSI()
{		
    AP_Param::setup_object_defaults(this, var_info);	
	rssi_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);	
}

// destructor
AP_RSSI::~AP_RSSI(void)
{		
}

// read the receiver RSSI value as an 8 bit number
uint8_t AP_RSSI::read_receiver_rssi(RssiType rssiType)
{
	// Default to 0 RSSI
    uint8_t receiver_rssi = 0;	
		
	switch (rssiType) {
		case RSSI_DISABLED :
		   receiver_rssi = 0;
		   break;
		case RSSI_ANALOG_PIN :
		   receiver_rssi = read_pin_rssi();
		   break;
		case RSSI_RC_CHANNEL_VALUE :
		   receiver_rssi = read_channel_rssi();
		   break;
		default : 	
		   receiver_rssi = 0;		
	}
	
	return receiver_rssi;
}

// Private
// -------
		
// read the RSSI value from an analog pin		
uint8_t AP_RSSI::read_pin_rssi()
{
	uint8_t pin_rssi = 0;	
	
	// avoid divide by zero
	if (rssi_range > 0) {            
		rssi_analog_source->set_pin(rssi_pin);
		float ret = rssi_analog_source->voltage_average() * 255 / rssi_range;
		pin_rssi = constrain_int16(ret, 0, 255);
	}
	return pin_rssi;
}

// read the RSSI value from a PWM value on a RC channel
uint8_t AP_RSSI::read_channel_rssi()
{
	uint8_t channel_rssi = 0;		
	
	int rssi_channel_value = hal.rcin->read(rssi_channel-1);
	// Note that user-supplied PWM range may be inverted and we accommodate that here. 
	//(Some radio receivers put out inverted PWM ranges for RSSI-type values).
	bool pwm_range_is_inverted = (rssi_channel_high_pwm_value < rssi_channel_low_pwm_value);
	// First, constrain to the possible PWM range - values outside are clipped to ends 
	int rssi_channel_value_clipped = constrain_int16(rssi_channel_value, 
													 pwm_range_is_inverted ? rssi_channel_high_pwm_value : rssi_channel_low_pwm_value, 
													 pwm_range_is_inverted ? rssi_channel_low_pwm_value : rssi_channel_high_pwm_value);
	// Then scale to 0-255 RSSI normally is presented in.
	int rssi_channel_range = abs(rssi_channel_high_pwm_value - rssi_channel_low_pwm_value);
	if (rssi_channel_range == 0) {
		// User range isn't meaningful, return 0 for RSSI (and avoid divide by zero)
		channel_rssi = 0;
	} else {
		float conversion_ratio = (float)255 / (float)rssi_channel_range;
		int rssi_channel_offset = abs(rssi_channel_value_clipped - rssi_channel_low_pwm_value);
		channel_rssi = (int)(rssi_channel_offset * conversion_ratio);
		channel_rssi = constrain_int16(channel_rssi, 0, 255);
	}   
	return channel_rssi;
}

