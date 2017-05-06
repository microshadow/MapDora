#include "m1.h"
#include "pathfinding_helper.h"
#include "fuzzy_search.h"
#include "StreetsDatabaseAPI.h"
#include <algorithm>
#include <cmath> 
#include <unordered_map>
#include <vector>
#include <set>
#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <limits>

//extern std::vector<std::string> segments_name;

/*******START OF DATA STRUCTURE DECLARATION*******/

//DATA STRUCTURE 0
//Mapping: geographic point -> POIIndex OR IntersectionIndex
//Used by: find_closest_intersection(...), find_closest_POI(...)
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef std::pair<point, unsigned> value;
static bgi::rtree<value, bgi::linear<16>> POI_rtree;
static bgi::rtree<value, bgi::linear<16>> Intersection_rtree;
double lat_avg;

//DATA STRUCTURE 1
//Mapping: string street name -> StreetIndex
//Used by:
static std::unordered_multimap<std::string, StreetIndex> street_name_street_index_map;

//DATA STRUCTURE 2
//Mapping: StreetIndex -> set<StreetSegmentIndex>
//Used by:
static std::vector<std::set<StreetSegmentIndex>> street_index_segment_index_vector;

//DATA STRUCTURE 3
//Mapping: string StreetIndex -> set<IntersectionIndex>
//Used by:
static std::vector<std::set<IntersectionIndex>> street_index_intersection_index_vector;

//DATA STRUCTURE 4
//Mapping: StreetSegmentIndex -> double length
//Used by:
static std::vector<std::pair<double, double>> segment_lengths_and_time;
std::pair<double, double> load_segment_lengths_and_time(unsigned street_segment_id);

//DATA STRUCTURE 5
//Mapping: IntersectionIndex -> vector<StreetSegmentIndex>
//Used by:
std::vector<std::vector<StreetSegmentIndex>> intersection_index_street_segment_index_vector;
std::vector<StreetSegmentIndex> load_intersection_street_segments(unsigned intersection_id);

/*******END OF DATA STRUCTURE DECLARATION*******/



bool load_map(std::string map_name) {
    
    /*load BIN*/
    bool load_success = loadStreetsDatabaseBIN(map_name);
    if (!(load_success))
        return false;
    /*end of load BIN*/

    int street_count = getNumberOfStreets();
    int segment_count = getNumberOfStreetSegments();
    int intersection_count = getNumberOfIntersections();
    int POI_count = getNumberOfPointsOfInterest();
    
    /*load data structure 0*/
    lat_avg = 0;
    
    //find the average latitude of all intersections
    for (int i=0; i<intersection_count; i++)
        lat_avg += getIntersectionPosition(i).lat();
    lat_avg = cos(lat_avg * DEG_TO_RAD / intersection_count);
    
    
    //populate Intersection_rtree
    for (unsigned i=0; i<intersection_count; i++){
        point p(getIntersectionPosition(i).lon() * lat_avg * DEG_TO_RAD, getIntersectionPosition(i).lat() * DEG_TO_RAD);
        Intersection_rtree.insert(std::make_pair(p,i));
    }
    
    //populate POI_rtree
    for (unsigned i=0; i<POI_count; i++){
        point p(getPointOfInterestPosition(i).lon() * lat_avg * DEG_TO_RAD, getPointOfInterestPosition(i).lat() * DEG_TO_RAD);
        POI_rtree.insert(std::make_pair(p,i));
    }
    /*end of load data structure 0*/
    

    /*load data structure 1*/
    street_name_street_index_map.reserve(street_count);
    
    for (int i=0; i<street_count; i++)
        street_name_street_index_map.emplace(getStreetName(i), i);
    /*end of load data structure 1*/  
    
    
    /*load data structure 2*/
    street_index_segment_index_vector.resize(street_count);
    
    for (unsigned i=0; i<segment_count; i++){
        street_index_segment_index_vector[getStreetSegmentInfo(i).streetID].insert(i);
        
    }
    /*end of load data structure 2*/  
        
    
    /*load data structure 3*/
    street_index_intersection_index_vector.resize(street_count);
    
    //loops through all intersections
    for (int i=0; i<intersection_count; i++){
        
        //finds how many street segments are connected to an intersection
        int segments_in_intersection = getIntersectionStreetSegmentCount(i);
        
        //creates a set for storing all unique streets connected to an intersection
        std::set<StreetIndex> intersection_unique_street_indices;
        
        //loops through all segments connected to an intersection
        for (int j=0; j<segments_in_intersection; j++){
            
            //finds the streetID of a segment
            StreetIndex temp = getStreetSegmentInfo(getIntersectionStreetSegment(i,j)).streetID;
            //insert this streetID into intersection_unique_indices (note: all set elements are automatically unique)
            intersection_unique_street_indices.insert(temp);
        }
        
        //loops through all unique streets connected to intersection
        for (std::set<StreetIndex>::iterator iter = intersection_unique_street_indices.begin(); iter != intersection_unique_street_indices.end(); iter++)
            
            //appends intersection to the vector of all intersections in a given street
            street_index_intersection_index_vector[*iter].insert(i);
    }
    /*end of load data structure 3*/
   
    /*load data structure 4*/
    segment_lengths_and_time.resize(segment_count);
    
    //finds the length of every street segment
    for (unsigned i=0; i<segment_count; i++)
        segment_lengths_and_time[i] = load_segment_lengths_and_time(i);
    
    /*end of load data structure 4*/
    
    /*load data structure 5*/
    intersection_index_street_segment_index_vector.resize(intersection_count);
    
    for (unsigned i=0; i<intersection_count; i++){
        intersection_index_street_segment_index_vector[i] = load_intersection_street_segments(i);
    }
    
    /*end of load data structure 5*/
    
    /*pathfinding helper data structures*/
    POI_name_to_ids_setup();
    
    /*data structure for auto-completion search*/
    POI_name_vector_setup();
    street_name_vector_setup();
    
    
    
    
    return true;
}

void close_map() {
    
    POI_rtree.clear();
    Intersection_rtree.clear();
    street_name_street_index_map.clear();
    street_index_segment_index_vector.clear();
    street_index_intersection_index_vector.clear();
    segment_lengths_and_time.clear();
    intersection_index_street_segment_index_vector.clear();
    global_POI_name_ids_map.clear();
    //clean up data structures for auto-completion
    POI_name_vector_cleanup(); 
    street_name_vector_cleanup();
    
    closeStreetDatabase();
}

//return street id(s) for the given street name
//If no street with this name exists, returns a 0-length vector.
std::vector<unsigned> find_street_ids_from_name(std::string street_name)
{
    
    std::vector<StreetIndex> street_indices;
    auto range = street_name_street_index_map.equal_range(street_name);

    for(auto iter=range.first; iter!=range.second; iter++)
    {
        street_indices.push_back(iter->second);       
    }
    
    return street_indices;
}

//returns the street names at the given intersection (includes duplicate street names in returned vector)
std::vector<std::string> find_intersection_street_names(unsigned intersection_id) 
{
    
    //make a vector that contains all the street segments connected to the intersection
    std::vector <unsigned> seg_connected_to_inter = find_intersection_street_segments (intersection_id); //this is a function m1.cpp written by Frank
    std::vector<std::string> street_names; //a vector that will store all streets connected 
    int street_id; //id of a street
    std::string st_name; //name of a street
    
    //go through each element in the vector, retrieve streetID from SegmentInfo,
    //convert the streetID to streetName
    //for (int i = 0; i < seg_connected_to_inter.size(); i++){
    for (auto iter = seg_connected_to_inter.begin(); iter != seg_connected_to_inter.end(); iter++){
        street_id = getStreetSegmentInfo(*iter).streetID;
        st_name = getStreetName (street_id);
        street_names.push_back (st_name); //put the street name into the vector
    }
    
    return street_names;
}

// Returns true if you can get from intersection1 to intersection2 using a single
//street segment ( hint : check for 1 - way streets too)
// corner case : an intersection is considered to be connected to itself
bool are_directly_connected(unsigned intersection_id1, unsigned intersection_id2)
{
    // corner case : an intersection is considered to be connected to itself
    if(intersection_id1 == intersection_id2)
    {
        return true;
    }
    
    unsigned intersection1_segCount = getIntersectionStreetSegmentCount(intersection_id1);
    unsigned intersection2_segCount = getIntersectionStreetSegmentCount(intersection_id2);
    
    //I think this is a better method, but how come it gives me the same error
    //message as the old one. I might need partners' help for this function.
    for(unsigned i=0; i < intersection1_segCount; i++)
    {
        for(unsigned j=0; j < intersection2_segCount; j++)
        {
            if(getIntersectionStreetSegment(intersection_id2, j) == getIntersectionStreetSegment(intersection_id1, i))
                return true;
        }
    }
    
    return false;
    
}

//Returns the street segments for the given intersection 
std::vector<unsigned> find_intersection_street_segments(unsigned intersection_id){
    
    return intersection_index_street_segment_index_vector[intersection_id];
}



//Returns all intersections reachable by traveling down one street segment 
//from given intersection (hint: you can't travel the wrong way on a 1-way street)
//the returned vector should NOT contain duplicate intersections
std::vector<unsigned> find_adjacent_intersections(unsigned intersection_id)
{    
    //use a vector as a container and return this vector
    std::vector<unsigned> all_adjacent_intersections;
    //we already have find_intersection_street_segments,
    //so we can create another vector to contain the segment_id incident to that intersection.
    std::vector<unsigned> segment_incident = find_intersection_street_segments(intersection_id);
    
    StreetSegmentInfo getStreetSegmentInfo(StreetSegmentIndex streetSegmentIdx);
    
    StreetSegmentInfo segment_info;
    
    for(unsigned i=0; i< segment_incident.size(); i++)
    {
        segment_info = getStreetSegmentInfo(segment_incident[i]);
        
        if(!segment_info.oneWay)
        {
            if(segment_info.from != intersection_id)
            {
                all_adjacent_intersections.push_back(segment_info.from);
            }
            else
            {
                all_adjacent_intersections.push_back(segment_info.to);
            }
        }
        else if(segment_info.oneWay)
        {
            if(segment_info.from == intersection_id)
            {
                all_adjacent_intersections.push_back(segment_info.to);
            }
        }
    }
    
    std::sort(all_adjacent_intersections.begin(), all_adjacent_intersections.end());
    all_adjacent_intersections.erase(std::unique(all_adjacent_intersections.begin(), all_adjacent_intersections.end()), all_adjacent_intersections.end());
    
    return all_adjacent_intersections;
}


/* Return street segments belong to the given street id*/
std::vector<unsigned> find_street_street_segments(unsigned street_id){

    std::vector<StreetSegmentIndex> ret_vector;
    ret_vector.assign(street_index_segment_index_vector[street_id].begin(), street_index_segment_index_vector[street_id].end());
    return ret_vector;
}

//Returns all intersections along the a given street
std::vector<unsigned> find_all_street_intersections(unsigned street_id)     
{   
 
    std::vector<IntersectionIndex> ret_vec;
    ret_vec.assign(street_index_intersection_index_vector[street_id].begin(), street_index_intersection_index_vector[street_id].end());
    
    return ret_vec;
}

std::vector<unsigned> find_intersection_ids_from_street_names(std::string street_name1, std::string street_name2){ 
    
    //fetches all street indices associated with each of the two streets
    std::vector<StreetIndex> name1_street_indices = find_street_ids_from_name(street_name1);
    std::vector<StreetIndex> name2_street_indices = find_street_ids_from_name(street_name2);
   
    //creates an empty vector for storing the indices of intersections where one or more streets are named street_nameX
    //may contain duplicate intersection indices in cases where two different streets both named street_nameX intersect each other
    std::set<IntersectionIndex> street_name1_intersections;
    std::set<IntersectionIndex> street_name2_intersections;
    
    //creates an empty vector for storing intersection indices where a street named stree_name1 intersects with a street named street_name2
    //this vector is returned at the end of the function
    std::set<IntersectionIndex> common_intersections;
        
    //loops through all street indices with attribute street_name1
    for (std::vector<StreetIndex>::iterator iter = name1_street_indices.begin(); iter != name1_street_indices.end(); iter++){
        
        std::set<IntersectionIndex> temp = street_index_intersection_index_vector[*iter];
        //reserves space for temp at the end of street_name1_intersections
        //appends temp to the end of street_name1_intersection
        street_name1_intersections.insert(temp.begin(), temp.end());
    }

    //performs the same operations as the above loop, except for street indices with attribute street_name2
    for (std::vector<StreetIndex>::iterator iter = name2_street_indices.begin(); iter != name2_street_indices.end(); iter++){
        std::set<IntersectionIndex> temp = street_index_intersection_index_vector[*iter];
        street_name2_intersections.insert(temp.begin(), temp.end());
    }
    
    
    std::vector<IntersectionIndex> common;
    set_intersection(street_name1_intersections.begin(),street_name1_intersections.end(),street_name2_intersections.begin(),street_name2_intersections.end(),std::inserter(common,common.begin()));
    return common;
}


/*Calculate the distance between the two points and return the result*/
double find_distance_between_two_points(LatLon point1, LatLon point2){
    
    double lat_ave = (point1.lat() + point2.lat())/2;
    double x1 = (point1.lon() * DEG_TO_RAD) * cos(lat_ave * DEG_TO_RAD);
    double y1 = (point1.lat() * DEG_TO_RAD);
    
    double x2 = (point2.lon() * DEG_TO_RAD) * cos(lat_ave * DEG_TO_RAD);
    double y2 = (point2.lat() * DEG_TO_RAD);
    
    double distance =  EARTH_RADIUS_IN_METERS * sqrt (pow (x2-x1,2) + pow (y2-y1, 2));
    
    return distance;
}



// Returns the length of the given street segment in meters
double find_street_segment_length(unsigned street_segment_id)
{
    return segment_lengths_and_time[street_segment_id].first;
}

double find_street_length(unsigned street_id)
{
    double length = 0;
    
    std::vector<StreetSegmentIndex> segments = find_street_street_segments(street_id);
    for (std::vector<StreetSegmentIndex>::iterator iter = segments.begin(); iter != segments.end(); iter++)
        length += segment_lengths_and_time[*iter].first;
    
    return length;
}

unsigned find_closest_intersection(LatLon my_position){
    
    std::vector<value> closest_intersection;
    Intersection_rtree.query(bgi::nearest(point(my_position.lon() * lat_avg * DEG_TO_RAD, my_position.lat() * DEG_TO_RAD), 1), std::back_inserter(closest_intersection));
    
    return closest_intersection[0].second;
}

//Returns the travel time to drive a street segment in seconds (time = distance/speed_limit)
/*Function 12: Seongju Yun
 * return the travel time to drive the street segment 
 */
double find_street_segment_travel_time (unsigned street_segment_id){
    return segment_lengths_and_time[street_segment_id].second;
}


unsigned find_closest_point_of_interest(LatLon my_position)
{
    std::vector<value> closest_POI;;
    POI_rtree.query(bgi::nearest(point(my_position.lon() * lat_avg * DEG_TO_RAD, my_position.lat() * DEG_TO_RAD), 1), std::back_inserter(closest_POI));
    
    return closest_POI[0].second;
}

//helper function to load length and time data for each segment
std::pair<double, double> load_segment_lengths_and_time(unsigned street_segment_id){
    
    double length = 0;
    double time = 0;
    
    unsigned curvepoint_num = getStreetSegmentInfo(street_segment_id).curvePointCount;
    unsigned intersection_from = getStreetSegmentInfo(street_segment_id).from;
    unsigned intersection_to = getStreetSegmentInfo(street_segment_id).to;
    float spd_lim = getStreetSegmentInfo(street_segment_id).speedLimit;
    
    if(curvepoint_num == 0)
    {
        length = find_distance_between_two_points(getIntersectionPosition(intersection_from),getIntersectionPosition(intersection_to));
    }
    
    else if(curvepoint_num == 1)
    {
        length += find_distance_between_two_points(getIntersectionPosition(intersection_from),getStreetSegmentCurvePoint(street_segment_id,0));
        length += find_distance_between_two_points(getStreetSegmentCurvePoint(street_segment_id,0),getIntersectionPosition(intersection_to));
    }
    
    else
    {
        length += find_distance_between_two_points(getIntersectionPosition(intersection_from),getStreetSegmentCurvePoint(street_segment_id,0));
        
        for(unsigned i=0; i<curvepoint_num-1; i++)
        {
            length += find_distance_between_two_points(getStreetSegmentCurvePoint(street_segment_id,i), getStreetSegmentCurvePoint(street_segment_id,i+1));
        }
        
        length += find_distance_between_two_points(getStreetSegmentCurvePoint(street_segment_id,curvepoint_num-1), getIntersectionPosition(intersection_to));
    }
    
    time = length / ((spd_lim / 3600) * (1000));
    
    return std::make_pair(length, time);
    
}

std::vector<StreetSegmentIndex> load_intersection_street_segments(unsigned intersection_id){
    
    //counts the number of street segments connected to an intersection
    unsigned segment_count = getIntersectionStreetSegmentCount(intersection_id);
    
    //creates an empty vector for storing street segment indices
    std::vector<StreetSegmentIndex> segments;
    
    //loops through all street segments connected to the intersection
    //appends the street segment index to the end of the vector
    for (unsigned i=0; i<segment_count; i++)
        segments.push_back(getIntersectionStreetSegment(intersection_id, i));
    return segments;    
}

unsigned find_closest_intersection_high_precision(LatLon ref)
{
    const int k = 2;
    
    std::vector<value> result_k;
    Intersection_rtree.query(bgi::nearest(point(ref.lon() * lat_avg * DEG_TO_RAD, ref.lat() * DEG_TO_RAD), k), std::back_inserter(result_k));
    
    unsigned min_index = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (int i=0; i<k; i++)
    {
        double temp_dist = find_distance_between_two_points(getIntersectionPosition(result_k[i].second), ref);
        if (temp_dist < min_dist)
        {
            min_dist = temp_dist;
            min_index = i;
        }
    }
    
    return result_k[min_index].second;
}
