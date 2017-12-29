#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "Vehicle.h"

using namespace std;

class Planner {
public:

	int ego_key = -1;

	int num_lanes;

  double speed_limit;

  map<int, Vehicle> vehicles;

  int vehicles_added = 0;

  /**
	* Constructor
	*/
	Planner(double speed_limit, int num_lanes);

	/**
	* Destructor
	*/
	virtual ~Planner();

	Vehicle get_ego();

	void populate_traffic(vector<vector<double>> sensor_fusion);

	void advance();

	void add_ego(Vehicle car, vector<double> config_data);

};
