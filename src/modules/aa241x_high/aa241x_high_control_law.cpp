/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file aa241x_fw_control.cpp
 *
 * Secondary file to the fixedwing controller containing the
 * control law for the AA241x class.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */

#include <uORB/uORB.h>

// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"

// needed for variable names
using namespace aa241x_high;

// define global variables (can be seen by all files in aa241x_high directory unless static keyword used)

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */

void flight_control() {
    
    float my_float_variable = 0.0f;		/**< example float variable */
    // float my_low_data = low_data.field1;      // getting low data value example
    // high_data.field1 = my_float_variable;     // setting high data value example
    
    float max_aileron_deflection = 30.0f; // in degrees, used to scale input from [-1; 1] to [-30 deg; 30 deg]
    float max_elevator_deflection = 30.0f; // in degrees
    float max_rudder_deflection = 30.0f; // in degrees
    // Not sure of throttle units, right now just a range from 0 to 1 so no scaling factor to servos
    
    float deg2rad = 0.01745329f;
    float rad2deg = 57.2957795f;
    
    
    // An example of how to run a one time 'setup' for example to lock one's altitude and heading...
    if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,
        //	should only occur on first engagement since this is 59Hz loop
        yaw_desired = yaw; 							// yaw_desired already defined in aa241x_high_aux.h
        altitude_desired = position_D_baro; 		// altitude_desired needs to be declared outside flight_control() function
    }
    
    // TODO: write all of your flight control here...
    // Make sure servo out values are defined in case of a breakdown in control logic in this code
    throttle_servo_out = man_throttle_in + aah_parameters.trim_throttle; //throttle has no scaling factor
    pitch_servo_out = man_pitch_in + aah_parameters.trim_elevator / max_elevator_deflection;
    yaw_servo_out = man_yaw_in + aah_parameters.trim_rudder / max_rudder_deflection;
    roll_servo_out = man_roll_in + aah_parameters.trim_aileron / max_aileron_deflection;
    
    // Begin control law logic, Default case is 0, should not change manual input plus trim
    if (aah_parameters.ctrl_case == 1) {        // Case 1: Simple roll stabilization
        roll_command = aah_parameters.cmd_phi; //in degrees
        
        //Limit desired roll so bank angle does not exceed max
        float maxBankAngle = 45.0f;
        if (roll_command > maxBankAngle) {
            roll_command = maxBankAngle;
        }
        else if(roll_command < -maxBankAngle) {
            roll_command = -maxBankAngle;
        }
        
        // Execute proportional control
        float delta_aileron = aah_parameters.proportional_roll_gain * (roll_command - roll); // Aileron deflection (in deg? probably)
        
        // Update servo commands (remember our control law is deviation from trim so we sum our perturbational change delta)
        roll_servo_out = roll_servo_out + delta_aileron / max_aileron_deflection;
    }
    // #############################################################################################################################
    else if (aah_parameters.ctrl_case == 2){     // Case 2: Longitudinal only, steady and level
        u_command = aah_parameters.cmd_u
        alt_command = aah_parameters.cmd_alt
        
        //Check bounds on command inputs
        float maxVelocity = 30.0f; // m/s
        float minVelocity = 11.0f; // m/s
        float maxAltitude = 100.0f; // meters, stay under 400ft
        float minAltitude = 20.0f; // meters
        if (u_command > maxVelocity) {
            u_command = maxVelocity;
        }
        else if(u_command < minVelocity) {
            u_command = minVelocity;
        }
        if (alt_command > maxAltitude) {
            alt_command = maxAltitude;
        }
        else if(alt_command < minAltitude) {
            alt_command = minAltitude;
        }
        
        // Execute proportional control
        float delta_throttle = aah_parameters.gain_throttle*(u_command - speed_body_u);
        float alt_measured = -position_D_baro;
        float pitch_command = aah_parameters.gain_altitude*(alt_command - alt_measured); //Should be degrees
        float delta_elevator = aah_parameters.gain_pitch*(pitch_command - pitch); //Should be degrees
        
        // Update servo outputs (remember our control law is deviation from trim so we sum our perturbational change delta)
        throttle_servo_out = throttle_servo_out + delta_throttle;
        pitch_servo_out = pitch_servo_out + delta_elevator / max_elevator_deflection;
    }

// ################################################################################################################################
    else if (aah_parameters.ctrl_case == 3){      // Case 3: Full control with DR damping
        float proportionalThrottleCorrection = aah_parameters.gain_throttle*(aah_parameters.cmd_u - speed_body_u);
    
        float proportionalPitchCorrection = aah_parameters.gain_altitude*(aah_parameters.cmd_alt - position_D_baro);
    
        float proportionalElevatorCorrection = aah_parameters.gain_pitch*(proportionalPitchCorrection - pitch);
    
        float elevatorOutput = -(pitch_trim + proportionalElevatorCorrection);	//negative to invert servo output
        float throttleOutput = man_throttle_in + proportionalThrottleCorrection;
    }
    // ###################################################################################################################
    
    // Bounds check the servo outputs to keep within limits of [-1; 1] or [0;1] for throttle
    if (throttle_servo_out > 1.0f) {
        throttle_servo_out = 1.0f;
    } else if (throttle_servo_out < 0.0f ) {
        throttle_servo_out = 0.0f;
    }
    if (pitch_servo_out > 1.0f) {
        pitch_servo_out = 1.0f;
    } else if (pitch_servo_out < -1.0f ) {
        pitch_servo_out = -1.0f;
    }
    if (roll_servo_out > 1.0f) {
        roll_servo_out = 1.0f;
    } else if (roll_servo_out < -1.0f ) {
        roll_servo_out = -1.0f;
    }
    if (yaw_servo_out > 1.0f) {
        yaw_servo_out = 1.0f;
    } else if (yaw_servo_out < -1.0f ) {
        yaw_servo_out = -1.0f;
    }
    
    
}
