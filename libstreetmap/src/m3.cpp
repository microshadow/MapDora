#include "m3_extra.h"
#include "pathfinding_helper.h"
#include <algorithm>
#include <unordered_set>
#include <boost/heap/fibonacci_heap.hpp>

using namespace std;

static unordered_map<IntersectionIndex, double> fCost;
static unordered_map<IntersectionIndex, double> gCost;
static unordered_map<IntersectionIndex, IntersectionIndex> cameFrom;

template <template<class,class,class...> class C, typename K, typename V, typename... Args>
V GetWithDef(const C<K,V,Args...>& m, K const& key, const V & defval)
{
    //template for fetching the value of a <key, value> pair in a container, given the key
    //if the the key does not exist, return a default value defval
    //the container is unchanged in this process
    typename C<K,V,Args...>::const_iterator it = m.find( key );
    if (it == m.end())
        return defval;  
    return it->second;
}

struct EuclideanVec
{    
private:
    
    LatLon start;
    LatLon end;
    LatLon vec;

public:
    
    EuclideanVec(){}
    EuclideanVec(const LatLon& _start, const LatLon& _end){start = _start; end = _end; vec = LatLon(end.lat() - start.lat(), end.lon() - start.lon()); }
    
    int relative_angle(const EuclideanVec& ref)
    {
        double dot = ref.vec.lon() * vec.lon() + ref.vec.lat() * vec.lat();
        double det = ref.vec.lon() * vec.lat() - ref.vec.lat() * vec.lon();
        
        double angle = atan2(det, dot) / DEG_TO_RAD;
            
        return int(angle);
    };
    
    string true_bearing()
    {    
        
        const string bearing[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
        
        double lat1 = DEG_TO_RAD * (start.lat());
        double lat2 = DEG_TO_RAD * (end.lat());
        
        double diffLong = DEG_TO_RAD * (end.lon() - start.lon());

        double x = sin(diffLong) * cos(lat2);
        double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(diffLong);
        
        double angle = atan2(x, y) / DEG_TO_RAD;
        double compass_bearing = fmod((angle + 360), 360);
        
        int key = (int(compass_bearing + 23 + 360) % 360) / 45;
        
        return bearing[key];
    
    }
};

double compute_path_travel_time(const vector<unsigned>& path, const double turn_penalty)
{   
    //special case for empty path
    //this function does not distinguish between no path due to inaccessible destination
    //or no path due to the source and destination being the same node    
    if (path.empty())
        return 0;
    
    
    double time = find_street_segment_travel_time(path[0]);
    
    for (int i=1; i<path.size(); i++)
    {
        //apply a turn penalty if the two consecutive segments do not share the same streetID
        if (getStreetSegmentInfo(path[i]).streetID != getStreetSegmentInfo(path[i-1]).streetID)
            time += turn_penalty;
        
        time += find_street_segment_travel_time(path[i]);
    }
    
    return time;
}

vector<unsigned> find_path_between_intersections(  const unsigned intersect_id_start,
                                                        const unsigned intersect_id_end,
                                                        const double turn_penalty)
{   
    //pathfinding algorithm using A*
    //to guarantee an optimal path, use an admissible heuristic function that always underestimates cost
    
    //resets cost and optimal path data associated with each Node
    fCost.clear();
    gCost.clear();
    cameFrom.clear();
    
    //openSet is the set of Node objects on the search frontier
    //closedSet is the set of intersections who immediate neighbours have been explored
    //note that the STL set is an ordered container
    //a Node comparison method is provided in the Node struct definition
    set<Node> openSet;
    unordered_set<IntersectionIndex> closedSet;
    
    //initialize cost data associated with the starting Node
    gCost[intersect_id_start] = 0;
    fCost[intersect_id_start] = 0 + heuristic_cost(intersect_id_start, intersect_id_end);
    
    //adds the starting Node to the search frontier
    openSet.insert(Node(intersect_id_start, fCost[intersect_id_start]));
    
    while (!openSet.empty())
    {
        //set the lowest cost node in openSet as the current node
        Node current = *openSet.begin();
        
        //reached the destination node, return the path
        if (current.ID == intersect_id_end)
            return reconstruct_path(current.ID, intersect_id_start);
        
        //all immediate neighbours of the current Node will have been explored by the end of the for loop
        //as such, current is moved from openSet to closedSet
        openSet.erase(openSet.begin());    
        closedSet.insert(current.ID);
        
        //find all immediate neighbours of the current node, taking into account directed edges
        vector<IntersectionIndex> neighbours = find_adjacent_intersections(current.ID);
        
        for (auto iter = neighbours.begin(); iter != neighbours.end(); iter++)
        {
            //checks if neighbour was already explored
            if (closedSet.find(*iter) != closedSet.end())
                continue;
            
            //calculates the new cost associated with the neighbour
            double new_g_cost = gCost.at(current.ID) + find_street_segment_travel_time(find_best_legal_segment_between_intersections(current.ID, *iter));
            
            //checks if turn penalty is applicable to the new cost
            //if (cameFrom.find(current.ID) != cameFrom.end())
            if (current.ID != intersect_id_start)
            {
                if ((getStreetSegmentInfo(find_best_legal_segment_between_intersections(cameFrom.at(current.ID), current.ID)).streetID 
                        != (getStreetSegmentInfo(find_best_legal_segment_between_intersections(current.ID, *iter))).streetID))
                    new_g_cost += turn_penalty;
            }            
            
            auto found = openSet.find(Node(*iter,GetWithDef(fCost, *iter, numeric_limits<double>::max())));
            bool in_openSet = (found != openSet.end());
            bool lower_cost = (new_g_cost < GetWithDef(gCost, *iter, numeric_limits<double>::max()));
            
            if (in_openSet && (!(lower_cost)))
                continue;
            
            if (in_openSet && lower_cost)
                openSet.erase(found);
            
            cameFrom[*iter] = current.ID;
            
            gCost[*iter] = new_g_cost;
            fCost[*iter] = new_g_cost + heuristic_cost(*iter, intersect_id_end);
            
            openSet.insert(Node(*iter, new_g_cost + heuristic_cost(*iter, intersect_id_end)));
            
        }
        
    }
    
    //all Nodes reachable from the starting Node are explored
    //destination Node is not accessible, as such there is no path
    return {};
}

vector<unsigned> find_path_to_point_of_interest(   const unsigned intersect_id_start,
                                                        const string point_of_interest_name,
                                                        const double turn_penalty)
{   
    //pathfinding algorithm using Dijkstra's method
    //note that Dijkstra's algorithm is simply a special case of A* with the heuristic function set to 0
    //note also that unlike A*, all accessible Nodes are explored in Dijkstra's algorithm
    
    fCost.clear();
    gCost.clear();
    cameFrom.clear();
    
    //find the LatLon coordinates of all POI with the given name
    vector<POIIndex> POI_ids = find_POI_id_by_name(point_of_interest_name);
    
    //no match found
    if (POI_ids.empty())
        return {};
    
    //find the closest (Euclidean distance) intersection to each POI
    vector<IntersectionIndex> POI_intersections;  
    
    for (auto iter = POI_ids.begin(); iter != POI_ids.end(); iter++)
    {
        POI_intersections.push_back(find_closest_intersection_high_precision(getPointOfInterestPosition(*iter)));
    }
        
    vector<StreetSegmentIndex> best_path = {};
    double best_cost = numeric_limits<double>::max();
    const int a_star_threshold = 3;
    
    if (POI_intersections.size() <= a_star_threshold)
    {
        for (auto iter = POI_intersections.begin(); iter != POI_intersections.end(); iter++)
        {
            vector<StreetSegmentIndex> temp_path = find_path_between_intersections(intersect_id_start, *iter, turn_penalty);
            double temp_cost = compute_path_travel_time(temp_path, turn_penalty);
            if (temp_cost < best_cost)
            {
                best_path = temp_path;
                best_cost = temp_cost;
            }
        }
        
        return best_path;
    }     
        
    //take the Euclidean distance from start Node to each POI Node
    //only the proximity_list_size closest destinations are considered
    //set to INF for absolute safety, reduce for faster runtime
    const int proximity_list_size = max(5, int(POI_intersections.size() / 50)) ;
     
    //lambda expression for comparing two IntersectionIndex by their Euclidean distance from intersect_id_start
    //for use with POI_intersections set ordering
    auto proximity_cmp = [&](const IntersectionIndex& lhs, const IntersectionIndex& rhs)-> bool 
    {
        return (heuristic_cost(intersect_id_start, lhs) < heuristic_cost(intersect_id_start, rhs));
    };
    
    if (POI_intersections.size() > proximity_list_size)
    {
        //sort POI_intersections by minimum Euclidean distance from starting Node
        //keep only the first proximity_list_size elements of POI_intersections
        sort(POI_intersections.begin(), POI_intersections.end(), proximity_cmp);
        POI_intersections = vector<IntersectionIndex>(POI_intersections.begin(), POI_intersections.begin() + proximity_list_size);
        if (intersect_id_start == *POI_intersections.begin())
            return {};
    }
    
    //refer to find_path_between_intersections(...) for additional comments about A* / Dijkstra's algorithm
    set<Node> openSet;
    unordered_set<IntersectionIndex> closedSet;
    
    gCost[intersect_id_start] = 0;
    
    openSet.insert(Node(intersect_id_start, 0));
    
    while (!openSet.empty())
    {
        Node current = *openSet.begin();
        
        openSet.erase(openSet.begin());        
        closedSet.insert(current.ID);
        
        vector<IntersectionIndex> neighbours = find_adjacent_intersections(current.ID);
        
        for (auto iter = neighbours.begin(); iter != neighbours.end(); iter++)
        {
            if (closedSet.find(*iter) != closedSet.end())
                continue;
                
            double new_g_cost = gCost.at(current.ID) + find_street_segment_travel_time(find_best_legal_segment_between_intersections(current.ID, *iter));
            
            if (current.ID != intersect_id_start)
            {
                if ((getStreetSegmentInfo(find_best_legal_segment_between_intersections(cameFrom.at(current.ID), current.ID)).streetID 
                        != (getStreetSegmentInfo(find_best_legal_segment_between_intersections(current.ID, *iter))).streetID))
                    new_g_cost += turn_penalty;
            }            
            
            auto found = openSet.find(Node(*iter,GetWithDef(gCost, *iter, numeric_limits<double>::max())));
            bool in_openSet = (found != openSet.end());
            bool lower_cost = (new_g_cost < GetWithDef(gCost, *iter, numeric_limits<double>::max()));
            
            if (in_openSet && (!(lower_cost)))
                continue;
            
            if (in_openSet && lower_cost)
                openSet.erase(found);
            
            cameFrom[*iter] = current.ID;
            
            gCost[*iter] = new_g_cost;
            
            openSet.insert(Node(*iter, new_g_cost));
            
        }
        
    }
    
    //finds the closest path from the starting Node to each POI
    for (auto iter = POI_intersections.begin(); iter != POI_intersections.end(); iter++)
    {
        if (cameFrom.find(*iter) != cameFrom.end())
        {
            vector<StreetSegmentIndex> temp_path = reconstruct_path(*iter, intersect_id_start);
            double temp_cost = compute_path_travel_time(temp_path, turn_penalty);
            
            //best_cost and best_path are updated if a new path has a lower cost
            if (temp_cost < best_cost)
            {
                best_path = temp_path;
                best_cost = temp_cost;
            }
        }
    }
    
    return best_path;
    
}

inline float fast_cosine(float val)
{
    //three term cosine series for heuristic cost function
    
    float val_2 = val * val;
    float val_4 = val_2 * val_2;
    float val_6 = val_4 * val_2;
    
    return (1 - val_2 * 0.5 + val_4 * 0.0417 - val_6 * 0.00139);
}

float heuristic_cost(IntersectionIndex source, IntersectionIndex dest) 
{   
    //heuristic function for A* algorithm
    //set this function to 0 for Dijkstra's algorithm
    if (source == dest)
        return 0;
    
    const float heuristic_speed = 30.56;       //assumed speed in in [m/s], for finding travel time; set this value lower for faster pathfinding
    const float R = 6371000.0;              //radius of the earth in [m]
    
    //WGS-84 coordinates
    LatLon source_pos = getIntersectionPosition(source);
    LatLon dest_pos = getIntersectionPosition(dest);
    
    //Euclidean distance approximation using equirectangular projection, in [m]
    float x = (dest_pos.lon() - source_pos.lon()) * DEG_TO_RAD * fast_cosine(DEG_TO_RAD * 0.5 * (dest_pos.lat() + source_pos.lat()));
    float y = (dest_pos.lat() - source_pos.lat()) * DEG_TO_RAD;
    
    float dist = R * sqrt(x*x + y*y);   
    
    //find heuristic travel time in [s]
    return dist/heuristic_speed;
    
}

vector<unsigned> reconstruct_path(IntersectionIndex current, IntersectionIndex origin)
{
    //this function reconstructs the path vector from the cameFrom map
    
    vector<IntersectionIndex> intersection_path = {current};
    vector<StreetSegmentIndex> street_segment_path;
    
    //reconstruct the a vector of intersections using the cameFrom map
    while (current != origin)
    {
        current = cameFrom.at(current);
        intersection_path.push_back(current);
    }
   
    //reverses the intersection vector such that it starts from origin and ends at current
    reverse(intersection_path.begin(), intersection_path.end()); 
    
    //reconstructs the path vector from the the intersection vector
    //note that a given intersection vector may yield yield more than one unique segment vector
    //see find_best_legal_segment_between_intersections(...) for a more detailed explanation
    for (int i=1; i<intersection_path.size(); i++)
    {
        street_segment_path.push_back(find_best_legal_segment_between_intersections(intersection_path[i-1], intersection_path[i]));
    }
    
    return street_segment_path;
}

StreetSegmentIndex find_best_legal_segment_between_intersections(IntersectionIndex source, IntersectionIndex dest)
{    
    //given two neighbouring intersections
    //return the fastest legal street segment between the intersections
    
    vector<StreetSegmentIndex> segments = find_intersection_street_segments(source);
    vector<StreetSegmentIndex>::iterator best = segments.end();
    double best_cost = numeric_limits<double>::max();
    
    for (auto iter = segments.begin(); iter != segments.end(); iter++)
    {
        StreetSegmentInfo info = getStreetSegmentInfo(*iter);
        double cost = find_street_segment_travel_time(*iter);
        
        bool legal = ((info.oneWay && info.from == source && info.to == dest) || ((!info.oneWay) && (info.from == dest || info.to == dest)));
        
        if (legal && (cost < best_cost))
        {
            best_cost = cost;
            best = iter;
        }
        
    }
    
    return *best;  
}

int subpath_boundary_angle(const StreetSegmentIndex current, const StreetSegmentIndex next, const IntersectionIndex boundary)
{
    LatLon boundary_pt = getIntersectionPosition(boundary);
    
    StreetSegmentInfo current_info = getStreetSegmentInfo(current);
    bool current_curved = (current_info.curvePointCount != 0);
    bool current_forward = (current_info.to == boundary);
    EuclideanVec current_vec = EuclideanVec();
    
    StreetSegmentInfo next_info = getStreetSegmentInfo(next);
    bool next_curved = (next_info.curvePointCount != 0);
    bool next_forward = (next_info.from == boundary);
    EuclideanVec next_vec = EuclideanVec();
    
    if (!(current_curved))
    {
        LatLon current_vec_start = getIntersectionPosition((boundary == current_info.to)?(current_info.from):(current_info.to));
        current_vec = EuclideanVec(current_vec_start, boundary_pt);
    }
    
    else 
    {
        LatLon current_vec_start = getStreetSegmentCurvePoint(current, 
            (current_forward)?(current_info.curvePointCount-1):(0));
        current_vec = EuclideanVec(current_vec_start, boundary_pt);
    }
    
    if (!(next_curved))
    {
        LatLon next_vec_end = getIntersectionPosition((boundary == next_info.from)?(next_info.to):(current_info.from));
        next_vec = EuclideanVec(boundary_pt, next_vec_end);
    }
    
    else 
    {
        LatLon next_vec_end = getStreetSegmentCurvePoint(next, 
            (next_forward)?(0):(next_info.curvePointCount-1));
        next_vec = EuclideanVec(boundary_pt, next_vec_end);
    }
    
    return next_vec.relative_angle(current_vec);
    
}

string format_readable_distance(float length_m)
{
    if (length_m < 10)
        return to_string(int(length_m + 0.5)) + " m";
    
    if (length_m < 1000)
        return to_string((int(length_m) + 5) / 10 * 10) + " m";
    
    if (length_m < 10000)
        return to_string(int(length_m) / 1000) + "." + to_string(int(length_m) % 1000 / 100) + " km";
    
}

vector<string> make_directions(const vector<StreetSegmentIndex>& path, const IntersectionIndex source)
{
    //makes a direction string for each subpath in path
    //where a subpath is defined as consecutive street segments sharing the same streetID
    
    //note that it is neccesary to pass in the source intersection because
    //in cases where path consists of only one street segment, there is not enough information to determine which intersection is the source
    
    vector<string> directions = {};
    
    //no directions to make
    if (path.empty())
        return {"Your destination is inaccessible or your destination is your starting point."};
    
    //the current_start_intersection and current_end_intersection are the two intersections associated with a street segment
    //taken into account the travel direction of path
    IntersectionIndex current_start_intersection = numeric_limits<unsigned>::max();
    IntersectionIndex current_end_intersection = source;
    
    //the streetID of the previous segment when iterating through the path
    StreetIndex prev_street = numeric_limits<unsigned>::max();
    
    vector<StreetSegmentIndex> current_subpath = {};
    
    bool new_subpath = false;
    bool last_segment_in_subpath = false;
    bool last_segment_in_path = false;
        
    for (auto iter = path.begin(); iter != path.end(); iter++)
    {
        StreetSegmentInfo current_info = getStreetSegmentInfo(*iter);
        
        //update the current_start_intersection and current_end_intersection
        current_start_intersection = current_end_intersection;
                //(iter == path.begin())?(source):((current_info.from == current_start_intersection)?(current_info.to):(current_info.from));
        current_end_intersection = (current_start_intersection != current_info.from)?(current_info.from):(current_info.to);
        
        new_subpath = (current_info.streetID != prev_street);
        last_segment_in_path = (iter == path.end()-1);
        last_segment_in_subpath = (last_segment_in_path)?(true):(getStreetSegmentInfo(*(iter+1)).streetID != current_info.streetID);
        
        current_subpath.push_back(*iter);
        
        if (new_subpath)
        {   
            prev_street = current_info.streetID;
            
            current_subpath.clear();
            current_subpath.push_back(*iter);
            
            directions.push_back("Head ");
            
            LatLon next_point = getIntersectionPosition(current_end_intersection);
        
            if (current_info.curvePointCount != 0)
            {
                next_point = (current_info.from == current_start_intersection)?
                    (getStreetSegmentCurvePoint(*iter, 0)):
                    (getStreetSegmentCurvePoint(*iter, current_info.curvePointCount-1));
            }
            
            EuclideanVec v = EuclideanVec(getIntersectionPosition(current_start_intersection), next_point);
            string bearing = v.true_bearing();
                    
            *(directions.end()-1) += v.true_bearing() + " on " + string((getStreetName(current_info.streetID) == "<unknown>")?("unknown road"):(getStreetName(current_info.streetID)));
                       
        }
        
        if (last_segment_in_subpath)
        {      
            string next_street_name = string((last_segment_in_path)?("your destination"):
                ((getStreetName(getStreetSegmentInfo(*(iter+1)).streetID) == "<unknown>")?
                    ("unknown road"):
                    (getStreetName(getStreetSegmentInfo(*(iter+1)).streetID))));
            
            *(directions.end()-1) +=  " towards " + next_street_name + ". ";
            
            float subpath_length_in_m = 0;
            
            for (auto subpath_iter = current_subpath.begin(); subpath_iter != current_subpath.end(); subpath_iter++)
                subpath_length_in_m += find_street_segment_length(*subpath_iter);
            
            
            *(directions.end()-1) += "After " + format_readable_distance(subpath_length_in_m) + ", ";
            
            if (last_segment_in_path)
            {
                *(directions.end()-1) += "you will arrive at your destination.";
                break;
            }
            
            string turn_instruction = "";
            
            vector<int> segment_angles = {};
            
            int next_angle = subpath_boundary_angle(*iter, *(iter+1), current_end_intersection);
            int boundary_segment_count = getIntersectionStreetSegmentCount(current_end_intersection);
            
            bool min_angle = true;
            bool right = (next_angle < 0);
            
            for (int i=0; i<boundary_segment_count;i++)
            {
                StreetSegmentIndex temp = getIntersectionStreetSegment(current_end_intersection, i);
                if (temp != *iter)
                {
                    int temp_angle = subpath_boundary_angle(*iter, temp, current_end_intersection);
                    min_angle = (abs(temp_angle) > abs(next_angle));
                }
            }
            
            if (min_angle)
            {
                if (boundary_segment_count == 1)
                    turn_instruction += "make a U-turn onto ";
            
                else if (abs(next_angle) > 90)
                    turn_instruction += "turn " + string((right)?("right"):("left")) + " onto ";
                
                else
                    turn_instruction += "continue straight onto ";
                
            }
            
            else
            {
                if (abs(next_angle) > 45)
                    turn_instruction += "turn " + string((right)?("right"):("left")) + " onto ";
                
                else
                    turn_instruction += "keep " + string((right)?("right"):("left")) + " to enter ";
            }
            
            *(directions.end()-1) += turn_instruction + next_street_name + ".";
                        
        }
        
    }
  
    return directions;    
   
}