# Path Planning
### Code structure:

The main code is present in `src/main.cpp`.

The main libraries used for this project are :- 

`Eigen 3.3`: To help on defining matrices and vectors.

`spline.h`: To create spline/polynominal.
<br>(Source:http://kluge.in-chemnitz.de/opensource/spline/ )

`uWS.h` : WebSocket & HTTP server implementation 
<br>(Source: https://github.com/uNetworking/uWebSockets)


### Introduction
The goal of this project is to build a path planner for car to so that it can drive autonomously on the highway. Here is the project rubric :-
- The car is able to drive at least 4.32 miles without incident.
- The car drives according to the speed limit.
- Max acceleration and jerk are not exceeded 10 m/s^2 and 10 m/s^3.
- The car must not come into contact with any of the other cars on the road.
- The car stays in its lane except for time between changing lange and it should not take more than 3 seconds for the lane change execution.
- The car is able to change lanes.

### Understanding the code

The highway waypoints map for the simulator from `highway_map.csv` file is loaded.

This file includes coordinate x, y, s of each waypoint and (d_x, d_y) for d coordinates.

The length of the entire track is 6945.554 meters.

We will be using Frenet coordinate system so that it will be easy to code and understand.
![Frenet Coordinates](/assets/FreNet.png?raw=true "Frenet Coordinates")

(Image taken from [Udacity's Self Driving Car Nano Degree Program](https://www.udacity.com/drive))

 We load the previous data of the car's location, yaw, speed, and path
 if available. (line 277 - line 292)
 
 
 `sensor_fusion` from simulator provides the list of all cars on the right side of the road. Each car has `d,s` coordinate and velocity so that we can determine its location.
 <br> All cars' `s` coordinates, presented by `vehicle_s`, are sorted into three vector `mid_lane`, `left_lane` and `right_lane` using their `d` coordinates. The `s` coordinates here represent the future position of each car by the end of
  previous planning path. (line 297 - line 318)
 
 We need to check if there is any vehicle in front of the self-driving car.
 If the future position of the car in front of the our car is less than the threshold distance of 37 meters away. Then two cars  are `too close` to each other, safety action is needed to be done. (line 320 - line 329)
 
### How to change the lane?
 <p align="center">
  <img src="https://media.giphy.com/media/xUOxf44pUXfq4BxpbG/giphy.gif?raw=true" alt="Lane Change"/>
</p>
 <br><br>If our car is too close to the car ahead, first action is to check the distance and determine whether is it very very close or not. If it is very very close then decrease the current velocity by 0.96 m/s and if is just close then decrease the current velocity by of 0.46 m/s. 
 <br>If the car has velocity lower than speed limit (49.5 mph), then we should increase with the velocity by 0.23 m/s.
 
 If the velocity is decreased lower than 42 mph, then our car 
 can think of switching lane to move faster with the traffic.
 
 If the car is in either left or right lane, it can switch to the middle lane. Or if the car is in the middle lane, it can switch to either left or right lane. Before the switch is done, some collision check is carried out to make sure the switch is certainly a safe choice. (line 331 - line 376)
 
 So we need a cost function which will help us to decide which lane should we switch too. ( line 197 - line 203):
 
```
 double calculateCollisionCost(vector<double> lane_s, double car_s, double upper_threshold, double lower_threshold)
{
	double cost = 0;
	for (int i = 0; i < lane_s.size(); i++)
	{
		if (isTooClose(lane_s[i], car_s, upper_threshold, lower_threshold))
		{
			cost += 10;
		}
		cost += 1;
	}
	return cost;
}
```
 `lane_s` is the lane to be checked.
 
 `car_s` is the our car's current s coordinates
 
 `upper_threshold` is the threshold distance to check if the car in front of our sscar is too close.
 
 `lower_threshold` is the threshold to check if the car behind our car is too close.
 
 If `isTooClose` returns true then we increase the cost by 10 as the car is too close and there are more chances of collision.For other cases we just add 1 to the cost. (line 187 - line 194)

 If cost is lower than 10, the lane is safe to switch into. If there are multiple options available then the car should switch into the lane with lesser cars (lesser cost).
```
bool isTooClose(double vehicle_s, double car_s, double upper_threshold, double lower_threshold)
{
	if (((vehicle_s - car_s) > upper_threshold) || ((vehicle_s - car_s) < lower_threshold))
	{
		return false;
	}
	return true;
}
```


If the previous path has less than 2 waypoints, then the previous reference of the car is one point 
 behind the current position in the same yaw direction. 
 <br>Or if previous waypoints have more than 2 points, then the last second point and the last point are considered as the reference points to trajectory for future waypoints.
 
 Then we find more future points of 30 meters, 60 meters and 90 meters away in `s` from the current position, by adding new distance to current `car_s`.

All points are added to vector `ptsx` and `ptsy`. From these points we create a spline function of the future path using `spline.h`. Before all points can be added, we need to bring them in car refernce and so we transform the coordinates.


Remaining waypoints can be added using the target reference velocity and target distance.We know the time between two waypoints is 0.02 seconds. The number of points that make up the target distance to meet the target velocity can be calculated. (line 445)

Lastly, we add future points to the previous path to make the complete path.

This whole can be seen from line 378 to line 456.
This process goes on continously.

### Demo

Please visit https://youtu.be/nsPnmmqy0gc for the demo video.

### Enhancements to do
- Can use PID or MPC.
- Can use Recurrent Neural Networks to take decisions for lane change.
