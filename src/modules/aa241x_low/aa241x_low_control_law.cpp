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
#include <math.h>
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
//Format that we'll receive
float goal_N[5] = {-2500,-2450,-2400,-44,2};
float goal_E[5] = {1950,1950,2000,-33,12};
float goal_r[5] = {2,5,3,-1,-1}; //-1 means point is not active
//Convert from N and E to our x,y grid
float goal_x=goal_N;
float goal_y=-goal_E;

// create array to monitor for the number of points visited, 0 is false (not visited) 1 is visited
int visited[5] = {0,0,0,0,0};

float low_data.field1 = -2410.0f;
float low_data.field2 = 2000.0f;
float low_data.field3 = -2411.0f;
float low_data.field4 = 2000.0f;
float low_data.field5 = 0.0f; //heading to best next point

/*
 * This loop executes at ~50Hz, but is not guaranteed to be 50Hz every time.
 */

// condition for the loop to stop running is the plane has visited all points and the distance is within the desired radius range 
void low_loop()
{
	//Get current state of aircraft
        float angle = -yaw;
	float x00 = position_N;
	float y00 = -position_E;
        // check the distance and heading cost to each point
        // use point with lowest cost
        
        float mincost=10000.0f;
        int mincostind=-1;
	float minangle=0.0f;
        for (int i = 0; a < 5; a = a + 1 ) {
            if (goal_r[i] != -1) {
                //Calculate cost of each
                float distcost = sqrtf(powf(goal_x[i]-x00,2.0f)+powf(goal_y[i]-y00,2.0f));
                float angletemp = atan2f(goal_y[i]-y00, goal_x[i]-x00,);
                float anglecost = abs(angletemp - angle);
                float cost = 100.0f*anglecost + distcost;
                if (cost < mincost) {
                        mincost=cost;
                        mincostind=i;
			minangle=angletemp;
                }
                //Check if visited
                if (distcost < goal_r[i]) {
                        goal_r[i]=-1; //deactivates a visited goal
                }
             }
        }
        //assign goal point to low field that high control law can use
	//if there is a new target point, otherwise continue to use the first line found
	//this should avoid drift and keep us following the same straight line to a point
	if ((low_data.field1 != goal_x[mincostind]) && (low_data.field2 != goal_y[mincostind])) {
        	float low_data.field1 = goal_x[mincostind];
		float low_data.field2 = goal_y[mincostind];
		float low_data.field3 = x00-(goal_x[mincostind]-x00);
		float low_data.field4 = y00-(goal_y[mincostind]-y00);
		float low_data.field5 = -minangle; //heading to best next point
	}
        /*
	// set the minimum turning radius
        float Radius = 15;
	
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
        */
        
        //check for visited checkpointts
        

}
