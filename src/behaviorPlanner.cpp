/*
 * BehaviorPlanner.cpp
 *
 *  Created on: Oct 7, 2017
 *      Author: tasos
 */

#include "behaviorPlanner.h"

// Identify to which lane the car is located. 0 for the line at the left, 1 for the middle lane and 2 for the right lane
int BehaviorPlanner::identifyLane(double d) {
	int lane;
	if (d>=0 && d<=4){
		lane = 0;
	}else if(d>4 && d<=8){
		lane =1;
	}else if (d>8 && d<12){
		lane=2;
	}
	return lane;
}

vector<vector<double>> BehaviorPlanner::closestVehicle(int lane,double mycar_s,vector<vector<double>> sensor_fusion){

	double vehicle_s;
	double vehicle_d;
	double vehicle_v;
	int vehicleLane;
	double distanceFromFrontVehicle = 10000; //Sth big - infinite (in case of no cars)
	double distanceFromBackVehicle = 10000; //Sth big - infinite (in case of no cars)
	double frontVehicleVelocity = SPEED_MAX_LIMIT; //mph -  Set in case of no cars
	double backVehicleVelocity = 0.1; //mph -  Set in case of no cars

	// Check each vehicle in sensor range
	  for(int vehicleId = 0; vehicleId < sensor_fusion.size(); vehicleId++) {
		  vehicle_s = sensor_fusion[vehicleId][5];
		  vehicle_d = sensor_fusion[vehicleId][6];
		  double vx = sensor_fusion[vehicleId][3];
		  double vy = sensor_fusion[vehicleId][4];
		  vehicle_v = sqrt(vx*vx+vy*vy);// m/s
		  // Project car position since our car s is already projected
		  vehicle_s += ((double)prev_size*TIME_INTERVAL*vehicle_v);
		  // Identify the lane this vehicle belongs to
		  vehicleLane = identifyLane(vehicle_d);

		  if (vehicleLane==lane){
			  if ((vehicle_s>mycar_s) && (vehicle_s-mycar_s <distanceFromFrontVehicle) ){
				  distanceFromFrontVehicle = vehicle_s-mycar_s;
				  frontVehicleVelocity = vehicle_v;
			  }

			  if ( (mycar_s>=vehicle_s) && (mycar_s-vehicle_s<distanceFromBackVehicle) ){
				  distanceFromBackVehicle  = mycar_s - vehicle_s;
				  backVehicleVelocity = vehicle_v;
			  }
		  }
		  if (distanceFromFrontVehicle <= 0) { // Avoid dividing by zero
			  distanceFromFrontVehicle = 1.0;
		  }
		  if (distanceFromBackVehicle <=0){
			  distanceFromBackVehicle = 1.0;
		  }
	  }

	  return { {distanceFromFrontVehicle, frontVehicleVelocity},{distanceFromBackVehicle,backVehicleVelocity} };
}

int BehaviorPlanner::laneIndicator(vector<vector<double>>  sensor_fusion){
	// My current lane
	current_lane = identifyLane(mycar_d);
	// The proposed lane
	int new_lane = current_lane;
	// Suggest new lane
	new_lane = laneCostEstimator(mycar_s, current_lane, sensor_fusion);
	bool isNewLaneSafe = true;
	if ( new_lane != current_lane){
		// Check if new lane is safe
		isNewLaneSafe = isLaneSafe(new_lane, current_lane, mycar_s,previous_ref_vel,sensor_fusion);
	}

	// If new lane is not safe stay in your lane
	if (!isNewLaneSafe){
		new_lane = current_lane;
	}
	// Adjust proper speed to that lane
	adjustVehicleTargetSpeed(new_lane, mycar_s, previous_ref_vel,sensor_fusion);
	return new_lane;

}

void BehaviorPlanner::adjustVehicleTargetSpeed(int lane,double mycar_s,double previous_ref_vel,vector<vector<double>> sensor_fusion){
	vector<vector<double>> lane_vehicles = closestVehicle(lane,mycar_s,sensor_fusion);
	vector<double> laneFrontVehicle = lane_vehicles[0];
	double laneFrontVehicleDistance = laneFrontVehicle[0];
	double laneFrontVehicleVelocity = laneFrontVehicle[1];
	//cout<< "my_s: "<<mycar_s<<" distanceFront_s: "<<laneFrontVehicleDistance <<endl;
	// Stabilize velocity and avoid speed oscillations
	if (laneFrontVehicleDistance<30 && previous_ref_vel>laneFrontVehicleVelocity*2.24){
		target_vehicle_speed = laneFrontVehicleVelocity;
	}else if (laneFrontVehicleDistance<15){
		target_vehicle_speed -= 0.224;
	}
	else {
		target_vehicle_speed = SPEED_MAX_LIMIT; //mph
	}
}

int BehaviorPlanner::laneCostEstimator(double myS, int myLane, vector<vector<double>> sensor_fusion){
	//Parameter calibration
	double Kdistance_front = 1.0;
	double Kdistance_back = 0.8;
	double Ksame_lane = 0.001;
	double Ktoofar_lane = 5.0;
	double Kavg = 0.2;
	// The less cost the better the lane
	 vector <double> laneScores = {0,0,0};
	 for (int i = 0; i < 3; i++) {
		 vector<vector<double>> lane_vehicles = closestVehicle(i,mycar_s,sensor_fusion);
		 double frontVehicleDistance = lane_vehicles[0][0];
		 double frontVehiclevelocity = lane_vehicles[0][1];
		 double backVehicleDistance = lane_vehicles[1][0];
		 double backVehiclevelocity = lane_vehicles[1][1];

		 // Distance of front and back vehicle cost
		 laneScores[i] += Kdistance_front*(1/frontVehicleDistance) + Kdistance_back*(1/backVehicleDistance);
		 // Lane too far from current lane
		 laneScores[i] += abs(myLane-i)>1?Ktoofar_lane:0;
	     // Favor current lane
		 laneScores[i] += Ksame_lane*(abs(myLane-i));
		 // Take account the cost lane history
		 laneScores[i] += Kavg*avg_scores[i];

		 updateLaneCost(laneScores[i], cost_lane[i]);
		 avg_scores[i] = calculateAvgCost(cost_lane[i]);

	 }

	 cout<<"avgcost lane 0:"<<avg_scores[0]<<" avgcost lane 1:"<<avg_scores[1]<<" avgcost lane 2:"<<avg_scores[2]<<endl;
	 cout<<"cost lane 0:"<<laneScores[0]<<" cost lane 1:"<<laneScores[1]<<" cost lane 2:"<<laneScores[2]<<endl;
	 // Find index with the minimum cost value
	 int laneIndex = min_element(laneScores.begin(), laneScores.end()) - laneScores.begin();
	 return laneIndex;
}

//void updateAvgCost

void BehaviorPlanner::updateLaneCost(double costLane, deque<double> &dq){
	if (dq.size()>COSTS_LIMIT) {
			dq.pop_back();
		}
	dq.push_front(costLane);
}

double BehaviorPlanner::calculateAvgCost(deque<double> dq){
	if (dq.size()==0){
		return 0.0;
	}
	double sum = 0.0;
	for (unsigned int i=0; i<dq.size(); ++i) {
		sum += dq.at(i);
	}
	return sum/dq.size();
}

bool BehaviorPlanner::isLaneSafe(int lane,int current_lane ,double mycar_s,double previous_ref_vel,vector<vector<double>> sensor_fusion){
	 vector<vector<double>> lane_vehicles = closestVehicle(lane,mycar_s,sensor_fusion);
	 vector<vector<double>> current_lane_vehicles = closestVehicle(current_lane,mycar_s,sensor_fusion);
	 double frontVehicleDistanceCurrentLane = current_lane_vehicles[0][0];
	 double backVehicleDistanceCurrentLane = current_lane_vehicles[1][0];
	 double frontVehicleDistance = lane_vehicles[0][0];
	 double frontVehiclevelocity = lane_vehicles[0][1];
	 double backVehicleDistance = lane_vehicles[1][0];
	 double backVehiclevelocity = lane_vehicles[1][1];
	 bool safe = false;

	 if(frontVehicleDistance<10 or backVehicleDistance<10 or frontVehicleDistanceCurrentLane<10 or backVehicleDistanceCurrentLane<10){
		 safe=false;

	 }else{
		 // Consider this my velocity
		 double futureRefVel = frontVehiclevelocity*2.24;

		 // Project my car position after the end of the path given with speed the lane's front car
		 double mycar_s_predicted = ((double)(50-prev_size)*TIME_INTERVAL*futureRefVel) + mycar_s;
		 // Project front vehicle position after the end of the path given
		 double frontVehicle_s_predicted = ((double)(50-prev_size)*TIME_INTERVAL*frontVehiclevelocity*2.24) + mycar_s + frontVehicleDistance;
		 // Project back vehicle position after the end of the path given
		 double backVehicle_s_predicted = ((double)(50-prev_size)*TIME_INTERVAL*backVehiclevelocity*2.24) + mycar_s - backVehicleDistance;
		 if ( (mycar_s_predicted-backVehicle_s_predicted) >10  and (frontVehicle_s_predicted-mycar_s_predicted)> 10){
			 safe=true;
		 }
	 }
	 cout<<"Safe:"<<safe<<endl;
	 return safe;


}
