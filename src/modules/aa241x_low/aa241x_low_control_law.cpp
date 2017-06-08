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
#include <stdio.h>
#include <math.h>
//#include <conio.h>
//#include <iostream>
//#include <queue>          // std::queue
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
float goal_N[5] = {-2540,-2450,-2400,-2425,2};
float goal_E[5] = {1900,1900,1950,1930,12};
float goal_r[5] = {10,20,8,20,-1}; //-1 means point is not active
//Convert from N and E to our x,y grid

float goal_x[5] = {goal_N[0],goal_N[1],goal_N[2],goal_N[3],goal_N[4]};// = goal_N;
float goal_y[5] = {-goal_E[0],-goal_E[1],-goal_E[2],-goal_E[3],-goal_E[4]};// = -goal_E;

// create array to monitor for the number of points visited, 0 is false (not visited) 1 is visited
int visited[5] = {0,0,0,0,0};

float previous_time = 0.0f;

// low_data.field1 = -2410.0f;
// low_data.field2 = 2000.0f;
// low_data.field3 = -2411.0f;
// low_data.field4 = 2000.0f;
// low_data.field5 = 0.0f; //heading to best next point

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
        //float costs[5] = {100,100,100,100,100};
        float mincost=10000.0f;
        int mincostind=-1;
	float minangle=0.0f;
        for (int i = 0; i < 5; i = i + 1 ) {
            if (goal_r[i] > 0) {
                //Calculate cost of each
                float distcost = sqrtf(powf(goal_x[i]-x00,2.0f)+powf(goal_y[i]-y00,2.0f));
                float angletemp = atan2f(goal_y[i]-y00, goal_x[i]-x00);
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
        if (mincostind == -1) { //if no suitable goal is found idle
                // low_data.field1 = goal_x[mincostind];
//                 low_data.field2 = goal_y[mincostind];
			low_data.field1 = -2507.0;
			low_data.field2 = -1905.0;
		//low_data.field3 = x00-(goal_x[mincostind]-x00);
		//low_data.field4 = y00-(goal_y[mincostind]-y00);
			low_data.field3 = -2425.0;
			low_data.field4 = -1905.0;
			low_data.field5 = 0;
		//low_data.field5 = -minangle;
        }
	else if ((abs(low_data.field1 - goal_x[mincostind]) > 1.0) || (abs(low_data.field2 - goal_y[mincostind]) > 1.0)) { //as long as new goal is different from old goal update
                low_data.field1 = goal_x[mincostind];
		low_data.field2 = goal_y[mincostind];
		low_data.field3 = x00-(goal_x[mincostind]-x00);
		low_data.field4 = y00-(goal_y[mincostind]-y00);
		low_data.field5 = -minangle; //heading to best next point
		previous_time = hrt_absolute_time(); //time of most recent update to line
	}
	else if (hrt_absolute_time() - previous_time > 2500000.0f) { //get new line if following old one too long (2 secs)
                low_data.field1 = goal_x[mincostind];
		low_data.field2 = goal_y[mincostind];
		low_data.field3 = x00-(goal_x[mincostind]-x00);
		low_data.field4 = y00-(goal_y[mincostind]-y00);
		low_data.field5 = -minangle; //heading to best next point
		previous_time = hrt_absolute_time(); //time of most recent update to line
	}
        //hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,

}
