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
 * @file aa241x_fw_control_params.c
 *
 * Definition of custom parameters for fixedwing controllers
 * being written for AA241x.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 */

#include "aa241x_high_params.h"

/*
 *  controller parameters, use max. 15 characters for param name!
 */

/**
 *THROTTLE TRIM
 *
 *Default throttle trim will be 0
 *
 * @unit thrust? voltage to motor?	(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TRIMTHROTTL, 0.0f);

/**
 *ELEVATOR TRIM
 *
 *Default elevator trim will be 0
 *
 * @unit degrees			(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TRIMELEV, 0.0f);

/**
 *AILERON TRIM
 *
 *Default aileron trim will be 0
 *
 * @unit degrees			(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TRIMAILERON, 0.0f);

/**
 *RUDDER TRIM
 *
 *Default rudder trim will be 0
 *
 * @unit degrees			(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TRIMRUDDER, 0.0f);

/**
 *THROTTLE GAIN
 *Throttle gain K from our control law. K*(u_c - u_meas)=delta_throttle
 *
 *Default throttle gain will be 1
 *
 * @unit none				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_GAINTHROTTL, 1.0f);

/**
 *ALTITUDE GAIN
 *Altitude to pitch gain K from our control law. K*(h_c - h_meas)=theta_command
 *
 *Default altitude gain will be 1
 *
 * @unit none				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_GAINALT, 0.05f);

/**
 *PITCH GAIN
 *Pitch to elevator gain K from our control law. K*(theta_c - theta_meas)=delta_elevator
 *
 *Default pitch gain will be 1
 *
 * @unit none				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_GAINPITCH, -6.0f);

/**
 *YAW RATE GAIN
 *r to deltarcommand feedback gain K from our control law. K*r=r_washout that feeds into deltarcommand summing junction
 *
 *Default yaw rate (r) gain will be 1
 *
 * @unit none				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_GAINYAWRATE, 50.5f);

/**
 *BETA GAIN
 *beta to deltarbeta gain K from our control law. K*(beta_c-beta)=deltarbeta that feeds into deltarcommand summing junction
 *
 *Default beta gain will be 1
 *
 * @unit none				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_GAINBETA, 65.0f);

/**
 *ROLL GAIN
 *roll to deltaaileron gain K from our control law. K*(roll_c-roll)=deltaaileron that closes roll loop
 *
 *Default roll gain will be 1
 *
 * @unit none				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_GAINPHI, 0.04f);

/**
 *HEADING GAIN
 *psi to phicommand gain K from our control law. K*(heading_c-heading)=roll_c that closes heading loop
 *
 *Default heading gain will be 1
 *
 * @unit none				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_GAINPSI, 10.0f);

/**
 *TRACKING GAIN
 *y to phi gain K from our control law. K*(y_c-y)=phi_c that closes tracking loop
 *
 *Default tracking gain will be 1
 *
 * @unit none				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_GAINTRACK, -0.7f);

/**
 *Velocity command U
 *For steady level flight
 *
 *Default speed will be 14.17 m/s
 *
 * @unit meters/second				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_CMDU, 14.17f);

/**
 *Altitude command
 *for steady level flight, do not exceed 400m
 *
 *Default altitude will be 50m
 *
 * @unit meter  				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_CMDALT, 50.0f);

/**
 *Sideslip beta command
 *for lateral control
 *
 *Default sideslip will be zero
 *
 * @unit degrees				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_CMDBETA, 0.0f);

/**
 *PHI, Roll command
 *for lateral control
 *
 *Default roll command will be 0
 *
 * @unit degrees				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_CMDPHI, 0.0f);

/**
 *PSI, heading command
 *Heading command for lateral control
 *
 *Default heading will be 0 (north? or initial orientation?)
 *
 * @unit degrees				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_CMDPSI, 0.0f);

/**
 *Tracking command
 *For lateral control to follow a certain trajectory
 *
 *Default tracking command will be zero
 *
 * @unit none				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_CMDTRACKING, 0.0f);

/**
 *Case Logic for Control Case
 *Allows us to have multiple
 *
 *Default case will be zero, manual control
 *
 * @unit none				(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_INT32(AAH_CASE, 0);


//rollgain
PARAM_DEFINE_FLOAT(AAH_PROPROLLGAIN, 0.004f);

//trim params
PARAM_DEFINE_FLOAT(AAH_TRIMPITCH, 0.0f);
PARAM_DEFINE_FLOAT(AAH_TRIMVEL, 16.0f);
PARAM_DEFINE_FLOAT(AAH_BANKLIMIT, 45.0f);

int aah_parameters_init(struct aah_param_handles *h)
{
    
    /* for each of your custom parameters, make sure to define a corresponding
     * variable in the aa_param_handles struct and the aa_params struct these
     * structs can be found in the aa241x_fw_control_params.h file
     *
     * NOTE: the string passed to param_find is the same as the name provided
     * in the above PARAM_DEFINE_FLOAT
     */
    // Case integer
    h->ctrl_case            = param_find("AAH_CASE");
    
    // Trim parameters
    h->trim_throttle		= param_find("AAH_TRIMTHROTTL");
    h->trim_elevator		= param_find("AAH_TRIMELEV");
    h->trim_aileron			= param_find("AAH_TRIMAILERON");
    h->trim_rudder			= param_find("AAH_TRIMRUDDER");
    h->trim_pitch			= param_find("AAH_TRIMPITCH");
    h->trim_velocity		= param_find("AAH_TRIMVEL");
    
    h->banklimit            = param_find("AAH_BANKLIMIT");
    
    // Command parameters
    h->cmd_u                = param_find("AAH_CMDU");
    h->cmd_alt              = param_find("AAH_CMDALT");
    h->cmd_beta             = param_find("AAH_CMDBETA");
    h->cmd_phi              = param_find("AAH_CMDPHI");
    h->cmd_psi              = param_find("AAH_CMDPSI");
    h->cmd_tracking         = param_find("AAH_CMDTRACKING");
    
    // Longitudinal Gain parameters
    h->gain_throttle		= param_find("AAH_GAINTHROTTL");
    h->gain_altitude		= param_find("AAH_GAINALT");
    h->gain_pitch			= param_find("AAH_GAINPITCH");
    
    // Lateral Gain parameters
    h->gain_yawrate			= param_find("AAH_GAINYAWRATE");
    h->gain_beta			= param_find("AAH_GAINBETA");
    h->gain_phi		    	= param_find("AAH_GAINPHI");
    h->gain_psi			    = param_find("AAH_GAINPSI");
    h->gain_tracking		= param_find("AAH_GAINTRACK");
    
    
    //roll gain
    h->proportional_roll_gain 	= param_find("AAH_PROPROLLGAIN");
    
    return OK;
}

int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p)
{
    
    // for each of your custom parameters, make sure to add this line with
    // the corresponding variable name
    
    // Case integer
    param_get(h->ctrl_case, &(p->ctrl_case));
    
    // Trim parameters
    param_get(h->trim_throttle, &(p->trim_throttle));
    param_get(h->trim_elevator, &(p->trim_elevator));
    param_get(h->trim_aileron, &(p->trim_aileron));
    param_get(h->trim_rudder, &(p->trim_rudder));
    param_get(h->trim_pitch, &(p->trim_pitch));
    param_get(h->trim_velocity, &(p->trim_velocity));
    
    param_get(h->banklimit, &(p->banklimit));
    
    // Command parameters
    param_get(h->cmd_u, &(p->cmd_u));
    param_get(h->cmd_alt, &(p->cmd_alt));
    param_get(h->cmd_beta, &(p->cmd_beta));
    param_get(h->cmd_phi, &(p->cmd_phi));
    param_get(h->cmd_psi, &(p->cmd_psi));
    param_get(h->cmd_tracking, &(p->cmd_tracking));
    
    // Longitudinal Gain Parameters
    param_get(h->gain_throttle, &(p->gain_throttle));
    param_get(h->gain_altitude, &(p->gain_altitude));
    param_get(h->gain_pitch, &(p->gain_pitch));
    
    // Lateral Gain Parameters
    param_get(h->gain_yawrate, &(p->gain_yawrate));
    param_get(h->gain_beta, &(p->gain_beta));
    param_get(h->gain_phi, &(p->gain_phi));
    param_get(h->gain_psi, &(p->gain_psi));
    param_get(h->gain_tracking, &(p->gain_tracking));
    
    //roll gain....
    param_get(h->proportional_roll_gain, &(p->proportional_roll_gain));
    
    return OK;
}
