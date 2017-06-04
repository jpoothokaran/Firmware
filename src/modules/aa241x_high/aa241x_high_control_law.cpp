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
#include <math.h>
// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"


// needed for variable names
using namespace aa241x_high;

// define global variables (can be seen by all files in aa241x_high directory unless static keyword used)
float u_initial;
float alt_initial;
float x0;
float x1;
float y00;
float y11;
float a;
float b;
float c;
float sign_convention;
float roll_command;
float u_command;
float alt_command;
float beta_command;
float phi_command;
float psi_command;
float heading_command;
float servo_throttle_initial,
float servo_elevator_initial,
float servo_rudder_initial,
float servo_aileron_initial,
float pitch_command,
float yaw_temp;
		

const float deg2rad = 0.01745329f;
//float rad2deg = 57.2957795f;
const float PI = 3.1415927;
	

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */

void flight_control() {
    
    //float my_float_variable = 0.0f;		/**< example float variable */
    // float my_low_data = low_data.field1;      // getting low data value example
    // high_data.field1 = my_float_variable;     // setting high data value example
    
    float max_aileron_deflection = 30.0f * deg2rad; // in rads, used to scale input from [-1; 1] to [-30 deg; 30 deg]
    float max_elevator_deflection = 25.0f * deg2rad; // in rads
    float max_rudder_deflection = 25.0f * deg2rad; // in rads
    // Not sure of throttle units, right now just a range from 0 to 1 so no scaling factor to servos
	//float alt_initial;
    
    
    // An example of how to run a one time 'setup' for example to lock one's altitude and heading...
    if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,
        //these four variables are logged so might as well set them, more data is better
        roll_desired=roll;    // yaw_desired already defined in aa241x_high_aux.h
        pitch_desired=pitch;
        yaw_desired=yaw;
        throttle_desired=throttle_servo_out;
        //	should only occur on first engagement since this is 59Hz loop
        //float psi_initial = yaw;
        alt_initial = -position_D_gps; 		
        u_initial = speed_body_u;

        //initial servo commands at time of switch
        servo_throttle_initial = man_throttle_in;
        servo_elevator_initial = man_pitch_in;
        servo_rudder_initial = man_yaw_in;
        servo_aileron_initial = man_roll_in;
		
        //line tracking initialization using position north and east, and the slope of the inital heading
        //y is negative east axis and x is north axis
        //use initial position as one point
		//second point is propogated out a length a in front of current heading.
        //line goes through first and second point
        //Then distance off path is altitude of triangle connecting current point, first, and second point
        //convention for positive distance is to the right of the line
		//positive distance determined by dot product of vectors A (desired vector) and B(current position)
		x0 = position_N;
        y00 = -position_E;
        a = 20.0f; // in meters, try to be large enough to avoid numerical errors but not so small we cause numeric errors
        x1 = x0 + a * cosf(-yaw);
		y11 = y00 + a * sinf(-yaw);
    }
    
    // TODO: write all of your flight control here...
    // Make sure servo out values are defined in case of a breakdown in control logic in this code
    throttle_servo_out = man_throttle_in; // + aah_parameters.trim_throttle; //throttle has no scaling factor
    pitch_servo_out = man_pitch_in; // + aah_parameters.trim_elevator / max_elevator_deflection;
    yaw_servo_out = man_yaw_in; // + aah_parameters.trim_rudder / max_rudder_deflection;
    roll_servo_out = man_roll_in; // + aah_parameters.trim_aileron / max_aileron_deflection;
    
    // Begin control law logic, Default case is 0, does not take commanded parameters, holds current heading altitude and speed as well as try to follow line
    if (aah_parameters.ctrl_case == 0) {
		//get geometry necessary for following line
        float x = position_N;
        float y = -position_E;
		b = sqrtf(powf(x1-x,2.0f)+powf(y11-y,2.0f)); //triangle side length
		c = sqrtf(powf(x-x0,2.0f)+powf(y-y00,2.0f)); //triangle side length
		float s = (a+b+c)/2.0f; //half perimeter used for herons formula for area
        float d = 2*sqrtf(s*(s-a)*(s-b)*(s-c))/a; //perpendicular distance off line (y in matlab code)
		//determine whether currently to the right or left of line, right is positive
		//Set up desired line vector rel to x0,y00
		float A_x = x1-x0;
		float A_y = y11-y00;
		//Set up current position rel to x0,y00 vector
		float B_x = x-x0;
		float B_y = y-y00;
		float d_sign = A_x * (-B_y) + A_y * B_x; //if d_sign>0 then B points to right of A, else if d_sign<0 B is left of A
		if (d_sign < 0){
			d = -d;
		} //else d, which always starts positive will just be positive or zero, no need to switch sign by muliplying
        
        //commands
        float d_command = 0.0f;
        beta_command = 0.0f;
        u_command = u_initial;
        alt_command = alt_initial;
        
        //Execute proportional control
        float delta_throttle = aah_parameters.gain_throttle*(u_command - speed_body_u);
        float alt_measured = -position_D_gps;
        pitch_command = aah_parameters.gain_altitude*(alt_command - alt_measured); //Should be a radian output
        float delta_elevator = aah_parameters.gain_pitch*(pitch_command + pitch_desired - pitch); //Should be in rads
        // pitch_desired is the initial (aka trim) pitch to fly at so we want to address deviations from that condition
        
        psi_command = aah_parameters.gain_tracking*(d_command - d); // tracking gain (y in matlab)
		//wrap to pi on heading going in in case commanded heading is -179 deg and measured is 179, difference is -358, which wraps to 2
		yaw_temp = psi_command + yaw_desired - yaw; //if on line psi_command is 0, so want to match initial yaw
		if (yaw_temp > PI) {
			yaw_temp = yaw_temp - 2.0f * PI;
		}
		else if (yaw_temp < -PI) {
			yaw_temp = yaw_temp + 2.0f * PI;
		}
        phi_command = aah_parameters.gain_psi*(yaw_temp);
		//not wrapping on roll because never expect to get close to pi or -pi
        float delta_aileron = aah_parameters.gain_phi*(phi_command + aah_parameters.cmd_phi - roll); //Should be in rads
        float delta_rudder = aah_parameters.gain_beta*(beta_command - speed_body_v/speed_body_u); //Should be in rads, small angle approx of beta
        
        // Update servo outputs (remember our control law is deviation from trim so we sum our perturbational change delta)
        throttle_servo_out = throttle_servo_out + delta_throttle;
        pitch_servo_out = pitch_servo_out + delta_elevator / max_elevator_deflection;
        roll_servo_out = roll_servo_out + delta_aileron / max_aileron_deflection;
        yaw_servo_out = yaw_servo_out + delta_rudder / max_rudder_deflection;
        
    }
    // #####################################################################################################################
    if (aah_parameters.ctrl_case == 1) {        // Case 1: Same as case 0 but now no manual input
        //get geometry necessary for following line
        float x = position_N;
        float y = -position_E;
		b = sqrtf(powf(x1-x,2.0f)+powf(y11-y,2.0f)); //triangle side length
		c = sqrtf(powf(x-x0,2.0f)+powf(y-y00,2.0f)); //triangle side length
		float s = (a+b+c)/2.0f; //half perimeter used for herons formula for area
        float d = 2*sqrtf(s*(s-a)*(s-b)*(s-c))/a; //perpendicular distance off line (y in matlab code)
		//determine whether currently to the right or left of line, right is positive
		//Set up desired line vector rel to x0,y00
		float A_x = x1-x0;
		float A_y = y11-y00;
		//Set up current position rel to x0,y00 vector
		float B_x = x-x0;
		float B_y = y-y00;
		float d_sign = A_x * (-B_y) + A_y * B_x; //if d_sign>0 then B points to right of A, else if d_sign<0 B is left of A
		if (d_sign < 0){
			d = -d;
		} //else d, which always starts positive will just be positive or zero, no need to switch sign by muliplying
        
        //commands
        float d_command = 0.0f;
        beta_command = 0.0f;
        u_command = u_initial;
        alt_command = alt_initial;
        
        //Execute proportional control
        float delta_throttle = aah_parameters.gain_throttle*(u_command - speed_body_u);
        float alt_measured = -position_D_gps;
        pitch_command = aah_parameters.gain_altitude*(alt_command - alt_measured); //Should be a radian output
        float delta_elevator = aah_parameters.gain_pitch*(pitch_command + pitch_desired - pitch); //Should be in rads
        // pitch_desired is the initial (aka trim) pitch to fly at so we want to address deviations from that condition
        
        psi_command = aah_parameters.gain_tracking*(d_command - d); // tracking gain (y in matlab)
		//wrap to pi on heading going in in case commanded heading is -179 deg and measured is 179, difference is -358, which wraps to 2
		yaw_temp = psi_command + yaw_desired - yaw; //if on line psi_command is 0, so want to match initial yaw
		if (yaw_temp > PI) {
			yaw_temp = yaw_temp - 2.0f * PI;
		}
		else if (yaw_temp < -PI) {
			yaw_temp = yaw_temp + 2.0f * PI;
		}
        phi_command = aah_parameters.gain_psi*(yaw_temp);
		//not wrapping on roll because never expect to get close to pi or -pi
        float delta_aileron = aah_parameters.gain_phi*(phi_command + aah_parameters.cmd_phi - roll); //Should be in rads
        float delta_rudder = aah_parameters.gain_beta*(beta_command - speed_body_v/speed_body_u); //Should be in rads, small angle approx of beta
        
        // Update servo outputs (remember our control law is deviation from trim so we sum our perturbational change delta)
        //DIFFERENCE FROM CASE 0, NO MANUAL INPUT, sets trim to values at time of switch
        throttle_servo_out = servo_throttle_initial + delta_throttle;
        pitch_servo_out = servo_elevator_initial + delta_elevator / max_elevator_deflection;
        roll_servo_out = servo_rudder_initial + delta_aileron / max_aileron_deflection;
        yaw_servo_out = servo_aileron_initial + delta_rudder / max_rudder_deflection;
    }
    // #############################################################################################################################
    else if (aah_parameters.ctrl_case == 2){     // Case 2: builds on Case 1,
        //Takes values from QGroundControl (heading, altitude, speed) and follows a line based on position at time of switch
        //in direction of specified heading
        //also NO MANUAL INPUT, uses trim servo input from time of control switch
        //NEED TO UPDATE TRIMS interp trim pitch (pitch_desired) given current state of speed desired
        u_command = aah_parameters.cmd_u;
        alt_command = aah_parameters.cmd_alt;
        heading_command = aah_parameters.cmd_psi; //expect degrees from QgroundControl
        heading_command = heading_command * deg2rad;
        
        //commands we don't want changing
        float d_command = 0.0f;
        beta_command = 0.0f;        
        
        //Check bounds on command inputs put to Qground control
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
        if (heading_command > PI) {
            heading_command = heading_command - 2.0f * PI;
        }
        else if (heading_command < -PI) {
            heading_command = heading_command + 2.0f * PI;
        }
        
        //new in this case, calc x1,y11
        x1 = x0 + a * cosf(-heading_command);
        y11 = y00 + a * sinf(-heading_command);
        //get geometry necessary for following line
        float x = position_N;
        float y = -position_E;
		b = sqrtf(powf(x1-x,2.0f)+powf(y11-y,2.0f)); //triangle side length
		c = sqrtf(powf(x-x0,2.0f)+powf(y-y00,2.0f)); //triangle side length
		float s = (a+b+c)/2.0f; //half perimeter used for herons formula for area
        float d = 2*sqrtf(s*(s-a)*(s-b)*(s-c))/a; //perpendicular distance off line (y in matlab code)
		//determine whether currently to the right or left of line, right is positive
		//Set up desired line vector rel to x0,y00
		float A_x = x1-x0;
		float A_y = y11-y00;
		//Set up current position rel to x0,y00 vector
		float B_x = x-x0;
		float B_y = y-y00;
		float d_sign = A_x * (-B_y) + A_y * B_x; //if d_sign>0 then B points to right of A, else if d_sign<0 B is left of A
		if (d_sign < 0){
			d = -d;
		} //else d, which always starts positive will just be positive or zero, no need to switch sign by muliplying
        
        //Execute proportional control
        float delta_throttle = aah_parameters.gain_throttle*(u_command - speed_body_u);
        float alt_measured = -position_D_gps;
        pitch_command = aah_parameters.gain_altitude*(alt_command - alt_measured); //Should be a radian output
        float delta_elevator = aah_parameters.gain_pitch*(pitch_command + pitch_desired - pitch); //Should be in rads
        // pitch_desired is the initial (aka trim) pitch to fly at so we want to address deviations from that condition
        
        psi_command = aah_parameters.gain_tracking*(d_command - d); // tracking gain (y in matlab)
		//wrap to pi on heading going in in case commanded heading is -179 deg and measured is 179, difference is -358, which wraps to 2
		yaw_temp = psi_command + heading_command - yaw; //if on line psi_command is 0, so want to match commanded heading
		if (yaw_temp > PI) {
			yaw_temp = yaw_temp - 2.0f * PI;
		}
		else if (yaw_temp < -PI) {
			yaw_temp = yaw_temp + 2.0f * PI;
		}
        phi_command = aah_parameters.gain_psi*(yaw_temp);
		//not wrapping on roll because never expect to get close to pi or -pi
        float delta_aileron = aah_parameters.gain_phi*(phi_command + aah_parameters.cmd_phi - roll); //Should be in rads
        float delta_rudder = aah_parameters.gain_beta*(beta_command - speed_body_v/speed_body_u); //Should be in rads, small angle approx of beta
        
        // Update servo outputs (remember our control law is deviation from trim so we sum our perturbational change delta)
        //DIFFERENCE FROM CASE 0, NO MANUAL INPUT, sets trim to values at time of switch
        //Want to make it so servo_xxxxx_initial becomes a trim from some lookup table/function at trim case
        throttle_servo_out = servo_throttle_initial + delta_throttle;
        pitch_servo_out = servo_elevator_initial + delta_elevator / max_elevator_deflection;
        roll_servo_out = servo_rudder_initial + delta_aileron / max_aileron_deflection;
        yaw_servo_out = servo_aileron_initial + delta_rudder / max_rudder_deflection;
    }

        // #############################################################################################################################
    else if (aah_parameters.ctrl_case == 3){     // Case 3: Longitudinal steady level, with basic roll stabilization
        u_command = aah_parameters.cmd_u;
        alt_command = aah_parameters.cmd_alt;
        roll_command = 0.0f; //in radians
        
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
        float alt_measured = -position_D_gps;
        pitch_command = aah_parameters.gain_altitude*(alt_command - alt_measured); //Should be rads
        float delta_elevator = aah_parameters.gain_pitch*(pitch_command - pitch); //Should be rads
        float delta_aileron = aah_parameters.proportional_roll_gain * (roll_command - roll); // Aileron deflection (in rad)
        
        // Update servo outputs (remember our control law is deviation from trim so we sum our perturbational change delta)
        throttle_servo_out = throttle_servo_out + delta_throttle;
        pitch_servo_out = pitch_servo_out + delta_elevator / max_elevator_deflection;
        roll_servo_out = roll_servo_out + delta_aileron / max_aileron_deflection;

    }
	
// ################################################################################################################################
    else if (aah_parameters.ctrl_case == 4){      // Case 4: same as casse zero but now takes target point from low control law
        float x = position_N;
        float y = -position_E;
        x1=low_data.field1;
        y11=low_data.field2;
        x0=low_data.field3;
        y00=low_data.field4;
        psi0=low_data.field5; //yaw (heading) associated with line from 0 to 1
        b = sqrtf(powf(x1-x,2.0f)+powf(y11-y,2.0f)); //triangle side length
	c = sqrtf(powf(x-x0,2.0f)+powf(y-y00,2.0f)); //triangle side length
	float s = (a+b+c)/2.0f; //half perimeter used for herons formula for area
        float d = 2*sqrtf(s*(s-a)*(s-b)*(s-c))/a; //perpendicular distance off line (y in matlab code)
	//determine whether currently to the right or left of line, right is positive
	//Set up desired line vector rel to x0,y00
	float A_x = x1-x0;
	float A_y = y11-y00;
	//Set up current position rel to x0,y00 vector
	float B_x = x-x0;
	float B_y = y-y00;
	float d_sign = A_x * (-B_y) + A_y * B_x; //if d_sign>0 then B points to right of A, else if d_sign<0 B is left of A
	if (d_sign < 0){
		d = -d;
	} //else d, which always starts positive will just be positive or zero, no need to switch sign by muliplying
        
        //commands
        float d_command = 0.0f;
        beta_command = 0.0f;
        u_command = u_initial; // commanded velocity to fly at
        alt_command = alt_initial; // commanded altitude to fly at
        
        //Execute proportional control
        float delta_throttle = aah_parameters.gain_throttle*(u_command - speed_body_u);
        float alt_measured = -position_D_gps;
        pitch_command = aah_parameters.gain_altitude*(alt_command - alt_measured); //Should be a radian output
        float delta_elevator = aah_parameters.gain_pitch*(pitch_command + pitch_desired - pitch); //Should be in rads
        // pitch_desired is the initial (aka trim) pitch to fly at so we want to address deviations from that condition
        
        psi_command = aah_parameters.gain_tracking*(d_command - d); // tracking gain (y in matlab)
		//wrap to pi on heading going in in case commanded heading is -179 deg and measured is 179, difference is -358, which wraps to 2
		yaw_temp = psi_command + yaw_desired - yaw; //if on line psi_command is 0, so want to match initial yaw
		if (yaw_temp > PI) {
			yaw_temp = yaw_temp - 2.0f * PI;
		}
		else if (yaw_temp < -PI) {
			yaw_temp = yaw_temp + 2.0f * PI;
		}
        phi_command = aah_parameters.gain_psi*(yaw_temp);
		//not wrapping on roll because never expect to get close to pi or -pi
        float delta_aileron = aah_parameters.gain_phi*(phi_command + aah_parameters.cmd_phi - roll); //Should be in rads
        float delta_rudder = aah_parameters.gain_beta*(beta_command - speed_body_v/speed_body_u); //Should be in rads, small angle approx of beta
        
        // Update servo outputs (remember our control law is deviation from trim so we sum our perturbational change delta)
        throttle_servo_out = servo_throttle_initial + delta_throttle;
        pitch_servo_out = servo_elevator_initial + delta_elevator / max_elevator_deflection;
        roll_servo_out = servo_rudder_initial + delta_aileron / max_aileron_deflection;
        yaw_servo_out = servo_aileron_initial + delta_rudder / max_rudder_deflection;
    }
    
    // ###################################################################################################################
    else if (aah_parameters.ctrl_case == 5){      // Case 5: Full control without DR damping, has heading, no tracking
        u_command = aah_parameters.cmd_u;
        alt_command = aah_parameters.cmd_alt;
        beta_command = aah_parameters.cmd_beta;
        phi_command = aah_parameters.cmd_phi;
        psi_command = aah_parameters.cmd_psi;
            
        //Check bounds on command inputs
        float maxVelocity = 30.0f; // m/s
        float minVelocity = 11.0f; // m/s
        float maxAltitude = 100.0f; // meters, stay under 400ft
        float minAltitude = 20.0f; // meters
        float maxBankAngle = 45.0f * deg2rad; // radians
        float maxSideslipAngle = 10.0f *deg2rad; // radians
        float minHeading = -180.0f * deg2rad; // radians
        float maxHeading = 180.0f * deg2rad; // radians
        
        if (psi_command > maxHeading){
            psi_command = psi_command - PI;
        }
        else if (psi_command < minHeading){
            psi_command = psi_command + PI;
        }
        if (phi_command > maxBankAngle) {
            phi_command = maxBankAngle;
        }
        else if(phi_command < -maxBankAngle) {
            phi_command = -maxBankAngle;
        }
        if (beta_command > maxSideslipAngle){
            beta_command = maxSideslipAngle;
        }
        else if (beta_command < -maxSideslipAngle){
            beta_command = -maxSideslipAngle;
        }
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
        pitch_command = aah_parameters.gain_altitude*(alt_command - alt_measured); //Should be rads
        float delta_elevator = aah_parameters.gain_pitch*(pitch_command - pitch); //Should be rads
        float delta_aileron = aah_parameters.gain_phi*(phi_command - roll); //Should be in rads
        float delta_rudder = aah_parameters.gain_psi*(psi_command - yaw); //Should be in rads
        
        // Update servo outputs (remember our control law is deviation from trim so we sum our perturbational change delta)
        throttle_servo_out = throttle_servo_out + delta_throttle;
        pitch_servo_out = pitch_servo_out + delta_elevator / max_elevator_deflection;
        roll_servo_out = roll_servo_out + delta_aileron / max_aileron_deflection;
        yaw_servo_out = yaw_servo_out + delta_rudder / max_rudder_deflection;
    }
    
    // ############################################################################################################
    // Store data to write to pixhawk output, will help analyze our control low easier
    //initial states
    high_data.field1 = roll_desired;
    high_data.field2 = pitch_desired;
    high_data.field3 = yaw_desired;
    high_data.field4 = throttle_desired;
    high_data.field5 = alt_initial;
    high_data.field6 = u_initial;
    high_data.field7 = servo_aileron_initial;
    high_data.field8 = servo_elevator_initial;
    high_data.field9 = servo_throttle_initial;
    high_data.field10 = servo_rudder_initial;
    // intermediate commands
    high_data.field11 = pitch_command;
    high_data.field12 = yaw_temp;
    high_data.field13 = phi_command;
    high_data.field14 = u_command;
    high_data.field15 = alt_command;
    high_data.field16 = heading_command;
	
	
	
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
