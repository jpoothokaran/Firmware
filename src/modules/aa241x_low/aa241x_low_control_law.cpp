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
 * @file aa241x_low.cpp
 *
 * Secondary control law file for AA241x.  Contains control/navigation
 * logic to be executed with a lower priority, and "slowly"
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */


// include header file
#include "aa241x_low_control_law.h"
#include "aa241x_low_aux.h"
#include<stdio.h>
#include<conio.h>
#include<iostream>
#include <queue>          // std::queue
//using namespace std;

#include <uORB/uORB.h>
#define PI 3.14159265
using namespace aa241x_low;

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */

/*
// calculate the coordinates of four possible center of circles, 
for each pair of points in output_points
{
   CR1 = (x1 + r*cos(phi), y1 - sin(phi));
   CL1 = (x1 + r*cos(phi), y1 - sin(phi));
   CR2 = (x2 + r*cos(phi), y2 - sin(phi));
   CL2 = (x2 + r*cos(phi), y2 - sin(phi));
}
*/

/*
 * This loop executes at ~50Hz, but is not guaranteed to be 50Hz every time.
 */
void low_loop()
{
        // calculate the shortest path by TSP
        input_points;
	queue<int> unvisited;
        // ouput_points = tsp(input_points);
        float Radius = r;
        float angle = yaw;
	float x0 = position_N;
	float y0 = -position_E;
	int case ; // case variable (case = 0 go straight, case = 1 left bank, case = 2 right bank)
	float goal_angle = atan2 (x - x0, y - y0) * 180 / PI; //set the goal angle in degrees

	// getting high data value example
	// float my_high_data = high_data.field1;
	
	float diff;
	
	if((-180 < angle - goal_angle < 0) || (0 < angle - goal_angle < 180)){
		diff = angle - goal_angle;
	}
	else {
		diff = angle - goal_angle + 2*PI;
	}
	
	// choose path for each point pair
	if (-10 < diff < 10){
	float low_data.field1 = 0;
        float low_data.field2 = 0;
	float low_data.field2 = x0;
	float low_data.field2 = y0;
	float low_data.field2 = x;
	float low_data.field2 = y;
	}
	else if(diff > 10){
			float low_data.field1 = 1;
                        float low_data.field2 = 30;
	                float low_data.field2 = x0;
	                float low_data.field2 = y0;
	                float low_data.field2 = x;
	                float low_data.field2 = y;
	}
	else (diff < -10){
		float low_data.field1 = 2;
                float low_data.field2 = -30;
	        float low_data.field2 = x0;
	        float low_data.field2 = y0;
	        float low_data.field2 = x;
	        float low_data.field2 = y;
	}
	

/*	
	case RSL
	        eita = pi/2 - atan((y2-y1)/(x2-x1));
	        gamma = atan(2*r/d);
	        theta = eita - gamma + pi/2;
		while phi < theta
			make right turn
			end while
	        dsum = 0;
	        while dsum < d
			go straight
			end while
		while phi < phi2
			make left turn
			end while
	case LSL
		theta = pi/2 - atan((y2-y1)/(x2-x1));
		while phi < theta
			make left turn
			end while
	        dsum = 0;
	        while dsum < d
			go straight
			end while
		while phi < phi2
			make left turn
			end while
	case LSR
	        eita = pi/2 + atan((y2-y1)/(x2-x1));
	        gamma = acos(2*r/d);
	        theta = eita + gamma - pi/2;
		while phi < theta
			make right turn
			end while
	        dsum = 0;
	        while dsum < d
			go straight
			end while
		while phi < phi2
			make left turn
			end while
*/				


}
