#pragma once
#include <unordered_map>
#include <string>
#include "StreetsDatabaseAPI.h"
#include <vector>



using namespace std;

extern unordered_multimap<string, unsigned> global_POI_name_ids_map;
//extern vector<const char*> POI_name_vector;

vector<unsigned> find_POI_id_by_name(string name);
void POI_name_to_ids_setup();
unsigned find_closest_intersection_high_precision(LatLon ref);  //defined in m1.cpp, for use when precision must be guaranteed





