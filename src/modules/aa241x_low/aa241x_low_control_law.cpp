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

// for now, we do not use TSP, and just hard code three points in order
// suppose we have 3 waypoints in the correct order, each point has three numbers: its x&y coordinates, and its radius required
        float input_points[][3] = {
		{10, 19, 10},
		{300, 129, 10},
		{700, 310, 10},
	};
// for this case we assume all points require radius of 10
        float goalRadius = 10;

// predefine the cooridnates of the first goal
        x = input_points[0][0];
        y = input_points[0][1];


// calculate how many points are given (calculate the number of columns)
        int cols = sizeof input_points[0] / sizeof(int); // 3 cols in this case
				   
// distance is the euclidean distance between the plane position and the goal point
        float distance; 
// create a variable to count for the number of points visited, initially 0 poinst are visited
        int visited = 0;

/*
 * This loop executes at ~50Hz, but is not guaranteed to be 50Hz every time.
 */

// condition for the loop to stop running is the plane has visited all points and the distance is within the desired radius range 
void low_loop((visited < cols) || (distance > goalRadius))
{
        // check the distance between the plane and the first point
        distance = sqrt((x-x0)^2 + (y-y0)^2);
        
	// set the minimum turning radius
        float Radius = 15;
	// get the current plane orientation, and x, y coordinates
        float angle = yaw;
	float x0 = position_N;
	float y0 = -position_E;
	
	int case ; // case variable (case = 0 go straight, case = 1 left bank, case = 2 right bank)
	float goal_angle = atan2 (x - x0, y - y0) * 180 / PI; //set the goal angle to the next goal point and convert it from raidans 
	// into degrees

	// getting high data value example
	// float my_high_data = high_data.field1;
	
	// set the difference angle between the current orientation and our desired angle to the goal point
	float diff;
	
	// make sure the difference angle is within -180 and 180 degrees
	if((-180 < angle - goal_angle < 0) || (0 < angle - goal_angle < 180)){
		diff = angle - goal_angle;
	}
	else if (-360 < angle - goal_angle <= -180){
		diff = angle - goal_angle + 360;
	}
	else {
		diff = angle - goal_angle - 360;
	}
	
	// determine which mode to take, straight line, left turn or right turn, and output corresponding parameters to high control law
	if (-10 < diff < 10){
	float low_data.field1 = 0;
        float low_data.field2 = 0;
	float low_data.field3 = x0;
	float low_data.field4 = y0;
	float low_data.field5 = x;
	float low_data.field6 = y;
	}
	else if(diff > 10){
			float low_data.field1 = 1;
                        float low_data.field2 = 30;
	                float low_data.field3 = x0;
	                float low_data.field4 = y0;
	                float low_data.field5 = x;
	                float low_data.field6 = y;
	}
	else (diff < -10){
		float low_data.field1 = 2;
                float low_data.field2 = -30;
	        float low_data.field3 = x0;
	        float low_data.field4 = y0;
	        float low_data.field5 = x;
	        float low_data.field6 = y;
	}
	
	// check whether the plane reaches the radius range of a certain goal point, and if yes, then change the goal to the next point
       	float x0 = position_N;
	float y0 = -position_E;
        distance = sqrt((x-x0)^2 + (y-y0)^2);;
	
	// when the plane reaches the desired range of a goal point
        if (distance <= goalRadius) {
		// check if the plane has visited all points, if yes, then break the loop
		if (visited == cols){
			break
		}
		// if the plane has not visited all points, then head to the next point 
                else{
                        visited += 1;
                        x = input_points[visited][0];
			y = input_points[visited][1];
		}
	}

}
