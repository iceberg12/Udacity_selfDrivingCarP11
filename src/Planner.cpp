#include <iostream>
#include "Planner.h"
#include "Vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>


/**
 * Initializes Road
 */

int check_lane(double d){
  int l = 1;
  for (int i=0; i<3; i++){
    if (d < (4*i + 4) && d >= (4*i)) {
      l = i;
      break;
    }
  }
  return l;
}

Planner::Planner(double speed_limit, int num_lanes) {
    this->num_lanes = num_lanes;
    this->speed_limit = speed_limit;
}

Planner::~Planner() {}

Vehicle Planner::get_ego() {

	return this->vehicles.find(this->ego_key)->second;
}

void Planner::populate_traffic(vector<vector<double>> sensor_fusion) {
  // add the other cars' state
  int vehicles_added = 0;
  map<int, Vehicle> other_vehicles;
  for (auto sf: sensor_fusion){
    double sf_d = sf[6];
    int sf_l = check_lane(sf_d);
    double sf_s = sf[5];
    // assume that s_d is in similar direction with car heading for other cars
    double sf_v = sqrt(pow(sf[3], 2.0) + pow(sf[4], 2.0));
    Vehicle car = Vehicle(sf_l, sf_s, sf_v, 0.0, "CS");

    vehicles_added += 1;
    other_vehicles.insert(std::pair<int, Vehicle>(vehicles_added, car));
  }
  this->vehicles_added = vehicles_added;
  this->vehicles = other_vehicles;
}

void Planner::advance() {

	map<int ,vector<Vehicle> > predictions;

	map<int, Vehicle>::iterator it = this->vehicles.begin();
    while(it != this->vehicles.end())
    {
        int v_id = it->first;
        vector<Vehicle> preds = it->second.generate_predictions();
        predictions[v_id] = preds;
        it++;
    }
	it = this->vehicles.begin();
	while(it != this->vehicles.end())
    {
    	int v_id = it->first;
        if(v_id == ego_key)
        {
        	vector<Vehicle> trajectory = it->second.choose_next_state(predictions);
        	it->second.realize_next_state(trajectory);
        }
        else {
            it->second.increment(1);
        }
        it++;
    }

}


void Planner::add_ego(Vehicle ego, vector<double> config_data) {
    ego.configure(config_data);
    this->vehicles.insert(std::pair<int,Vehicle>(ego_key,ego));

}
