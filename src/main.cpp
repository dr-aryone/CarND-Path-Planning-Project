#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

#define REF_VELOCITY_FOR_LANE_CHANGE 42
#define THRESHOLD_DISTANCE 37
#define MAX_VELOCITY 49.5
#define VERY_VERY_CLOSE_DISTANCE 15

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = abs(theta - heading);

	if (angle > pi() / 4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x, y};
}

//This function will populate the vectors according to the magnitude of d
void populateAppropriateLane(double d, double vehicle_s, vector<double> &left_lane, vector<double> &mid_lane, vector<double> &right_lane)
{
	if ((d < 4) && (d > 0))
	{
		left_lane.push_back(vehicle_s);
	}

	if ((d < 8) && (d > 4))
	{
		mid_lane.push_back(vehicle_s);
	}

	if ((d < 12) && (d > 8))
	{
		right_lane.push_back(vehicle_s);
	}
}

//This function determines whether a car is too close or not
bool isTooClose(double vehicle_s, double car_s, double upper_threshold, double lower_threshold)
{
	if (((vehicle_s - car_s) > upper_threshold) || ((vehicle_s - car_s) < lower_threshold))
	{
		return false;
	}
	return true;
}

//Cost is the number of vehicles in the particular lane and also cost depends if the vehicle is too close or not
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

int main()
{
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line))
	{
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	int lane = 1;
	double ref_vel = 0.0;
	double distance_between_car = 0.0;

	h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &ref_vel, &lane, &distance_between_car](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
																																					 uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{

			auto s = hasData(data);

			if (s != "")
			{
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry")
				{
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					/***Sensor Fusion starts here***/
					int prev_size = previous_path_x.size();
					bool too_close = false;

					if (prev_size > 2)
					{
						car_s = end_path_s;
					}

					vector<double> left_lane;  //Lane 0
					vector<double> mid_lane;   //Lane 1
					vector<double> right_lane; //Lane 2

					for (int i = 0; i < sensor_fusion.size(); i++)
					{
						float d = sensor_fusion[i][6];
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double vehicle_speed = sqrt(vx * vx + vy * vy);
						double vehicle_s = sensor_fusion[i][5];
						vehicle_s += vehicle_speed * 0.02 * previous_path_x.size();

						populateAppropriateLane(d, vehicle_s, left_lane, mid_lane, right_lane);

						// Check whether the vehicle in front of our vehicle is too close or not by calculating the distance and using some threshold.
						if ((d < (4 * lane + 2 + 2)) && (d > (4 * lane)))
						{
							distance_between_car = vehicle_s - car_s;
							if ((vehicle_s > car_s) && (distance_between_car < THRESHOLD_DISTANCE))
							{
								too_close = true;
							}
						}
					}

					if (too_close)
					{
						if (distance_between_car < VERY_VERY_CLOSE_DISTANCE)
						{
							ref_vel -= 0.96; //Declearte more as the car is very very close
						}
						else
						{
							ref_vel -= 0.46; //Normal decleration
						}
						if (ref_vel < REF_VELOCITY_FOR_LANE_CHANGE)
						{
							if (lane == 0 or lane == 2)
							{
								double collision_cost = calculateCollisionCost(mid_lane, car_s, 25, -10);
								if (collision_cost < 10)
								{
									lane = 1;
								}
							}

							else if (lane == 1)
							{
								double left_cost = calculateCollisionCost(left_lane, car_s, 25, -10);
								double right_cost = calculateCollisionCost(right_lane, car_s, 25, -10);
								if (left_cost < right_cost)
								{
									if (left_cost < 10)
									{
										lane = 0;
									}
								}
								else
								{
									if (right_cost < 10)
									{
										lane = 2;
									}
								}
							}
						}
					}
					else if (ref_vel < MAX_VELOCITY)
					{
						ref_vel += 0.23;
					}

					vector<double> next_x_vals;
					vector<double> next_y_vals;
					vector<double> ptsx;
					vector<double> ptsy;

					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = car_yaw;

					double ref_prev_x, ref_prev_y;

					if (prev_size < 2)
					{
						ref_prev_x = ref_x - cos(ref_yaw);
						ref_prev_y = ref_y - sin(ref_yaw);
					}
					else
					{
						ref_prev_x = previous_path_x[prev_size - 2];
						ref_prev_y = previous_path_y[prev_size - 2];
						ref_x = previous_path_x[prev_size - 1];
						ref_y = previous_path_y[prev_size - 1];
						ref_yaw = atan2(ref_y - ref_prev_y, ref_x - ref_prev_x);
					}

					ptsx.push_back(ref_prev_x);
					ptsx.push_back(ref_x);
					ptsy.push_back(ref_prev_y);
					ptsy.push_back(ref_y);

					vector<double> waypoint0 = getXY(car_s + 30, 4 * lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> waypoint1 = getXY(car_s + 60, 4 * lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> waypoint2 = getXY(car_s + 90, 4 * lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

					ptsx.push_back(waypoint0[0]);
					ptsx.push_back(waypoint1[0]);
					ptsx.push_back(waypoint2[0]);
					ptsy.push_back(waypoint0[1]);
					ptsy.push_back(waypoint1[1]);
					ptsy.push_back(waypoint2[1]);

					// Corrdinates transform
					for (int i = 0; i < ptsx.size(); i++)
					{
						double shift_x = ptsx[i] - ref_x;
						double shift_y = ptsy[i] - ref_y;
						ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
						ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
					}

					tk::spline s;
					s.set_points(ptsx, ptsy);

					double target_x = 30;
					double target_y = s(30);
					double target_dist = sqrt(target_x * target_x + target_y * target_y);

					for (int i = 0; i < prev_size; i++)
					{
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					double x_additional = 0.0;

					for (int i = 0; i < 50 - prev_size; i++)
					{
						double N = target_dist / (0.02 * ref_vel / 2.24);
						double x_ = x_additional + target_x / N;
						double y_ = s(x_);
						x_additional = x_;

						double x_ref = x_, y_ref = y_;
						x_ = ref_x + (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
						y_ = ref_y + (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

						next_x_vals.push_back(x_);
						next_y_vals.push_back(y_);
					}

					// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
					   size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
						   char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
