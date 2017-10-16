#ifndef behavior_hpp
#define behavior_hpp

#include <vector>
#include <string>
#include <deque>

using namespace std;

class BehaviorPlanner {

	public:
		int current_lane;
		double SPEED_MAX_LIMIT = 49.5;//mph
		double target_vehicle_speed = SPEED_MAX_LIMIT;
		double previous_ref_vel=0.0;
		double TIME_INTERVAL = 0.02;//in seconds
		double mycar_s;
		double mycar_d;
		// The size of the previous_path_x vector
		int prev_size;
		// Average scores keeping track of
		vector<double> avg_scores = {0,0,0};
		vector<deque<double>> cost_lane = {deque<double>(),deque<double>(),deque<double>()};

		int COSTS_LIMIT =50;
		// The current metrics of sensor fusion data
		//vector<vector<double>> current_sensor_fusion;

		// Given the lateral coordinate d identify to which lane our car is located
		int identifyLane(double d);

		// For the lane specified search for the closest vehicle in front of and back of the car located in my_vehicle_s position
		vector<vector<double>> closestVehicle(int lane, double mycar_s, vector<vector<double>> sensor_fusion);
		// Propose lane for best driving behavior
		int laneIndicator(vector<vector<double>> sensor_fusion);
		// Compute cost for lanes and suggest the lane with the minimum cost
		int laneCostEstimator(double s, int lane, vector<vector<double>> sensor_fusion);
		// Adjust vehicle target speed taking acount the car in front of it
		void adjustVehicleTargetSpeed(int lane,double mycar_s,double previous_ref_vel,vector<vector<double>> sensor_fusion);
		// Predict lane traffic and estimate possible collision
		bool isLaneSafe(int lane,int current_lane ,double mycar_s,double previous_ref_vel,vector<vector<double>> sensor_fusion);
		// Update costs of lanes
		void updateLaneCost(double costLane, deque<double>&dq);
		// Calculate average cost of all the elements in queue
		double calculateAvgCost(deque<double> dq);
};



#endif
