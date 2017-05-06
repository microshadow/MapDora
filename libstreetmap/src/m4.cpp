#pragma once
#include "m1.h"
#include "m4.h"
#include "m3.h"
#include "m3_extra.h"
#include "StreetsDatabaseAPI.h"
#include <vector>
#include <list>
#include <algorithm>
#include <set>
#include <limits>
#include <tuple>

#define TEMP_FACTOR 100
#define INITIAL_BREADTH 5

int Temp;
int setup_initial_Temp (const int delivery_num);
int determine_breadth (int T);

class Delivery
{
public:
    struct DeliveryNode;
    
    typedef std::vector<DeliveryNode>::iterator NodeHandle;
    
    struct DeliveryNode
    {
        DeliveryNode(unsigned _intersection, NodeHandle _handle):intersection(_intersection), traversed(false), handle(_handle) {}

        NodeHandle handle;
        unsigned intersection;
        bool traversed;
    };
    
private: 
    
    int heuristic_breadth = 5;
    const float turn_penalty;
    std::vector<DeliveryNode> pickup_nodes;
    std::vector<DeliveryNode> dropoff_nodes;
    std::vector<unsigned> depots;
    
    std::vector<std::pair<unsigned, NodeHandle>> tour;
    
    void untraverse_all()
    {
        //reset traverse flags for all delivery nodes
        
        for (auto it = pickup_nodes.begin(); it != pickup_nodes.end(); it++)
            it->traversed = false;
        
        for (auto it = dropoff_nodes.begin(); it != dropoff_nodes.end(); it++)
            it->traversed = false;
        
    }
    
    unsigned find_start()
    {
        //finds the centroid of all delivery locations
        //finds the depot location closest to the centroid
        
        unsigned centroid_depot = *depots.begin();
        float min_distance = numeric_limits<float>::max();
        
        double lon_avg = 0;
        double lat_avg = 0;
        int node_count = pickup_nodes.size() + dropoff_nodes.size();
        
        for (auto it = pickup_nodes.begin(); it != pickup_nodes.end(); it++)
        {
            lon_avg += getIntersectionPosition(it->intersection).lon();
            lat_avg += getIntersectionPosition(it->intersection).lat();
        }
        
        for (auto it = dropoff_nodes.begin(); it != dropoff_nodes.end(); it++)
        {
            lon_avg += getIntersectionPosition(it->intersection).lon();
            lat_avg += getIntersectionPosition(it->intersection).lat();
        }
        
        lon_avg /= node_count;
        lat_avg /= node_count;
        
        unsigned centroid_intersection = find_closest_intersection(LatLon(lat_avg, lon_avg));
        
        for (auto it = depots.begin(); it != depots.end(); it++)
        {
            float new_distance = heuristic_cost(*it, centroid_intersection);
            
            if (new_distance < min_distance)
            {
                min_distance = new_distance;
                centroid_depot = *it;
            }
            
        }
        
        return centroid_depot;
    }
    
    std::vector<std::pair<unsigned, NodeHandle>> legal_neighbours()
    {
        std::vector<std::pair<unsigned, NodeHandle>> neighbours;
        
        for (int i=0; i<pickup_nodes.size(); i++)
        {
            if (!pickup_nodes[i].traversed)
                neighbours.push_back(std::make_pair(pickup_nodes[i].intersection, pickup_nodes[i].handle));
            else if (!dropoff_nodes[i].traversed && pickup_nodes[i].traversed)
                neighbours.push_back(std::make_pair(dropoff_nodes[i].intersection, dropoff_nodes[i].handle));
        }
        
        return neighbours;
    }
    
    std::vector<std::pair<unsigned, NodeHandle>> k_closest_neighbours(unsigned current) 
    {
        
        auto proximity_cmp = [&](const std::pair<unsigned, NodeHandle>& lhs, const std::pair<unsigned, NodeHandle>& rhs)-> bool 
        {
            return (heuristic_cost(current, lhs.first) < heuristic_cost(current, rhs.first));
        };
        
        std::vector<std::pair<unsigned, NodeHandle>> all_neighbours = legal_neighbours();
        std::nth_element(all_neighbours.begin(), all_neighbours.begin()+ std::min(heuristic_breadth,int(all_neighbours.size())), all_neighbours.end(), proximity_cmp);
        
        return std::vector<std::pair<unsigned, NodeHandle>> (all_neighbours.begin(), all_neighbours.begin() + std::min(heuristic_breadth, int(all_neighbours.size())));
    
    }
    
    unsigned find_end(unsigned current_intersection)
    {
        std::vector<unsigned> end_depots = depots;
        
        auto proximity_cmp = [&](const IntersectionIndex& lhs, const IntersectionIndex& rhs)-> bool 
        {
            return (heuristic_cost(current_intersection, lhs) < heuristic_cost(current_intersection, rhs));
        };
        
        std::nth_element(end_depots.begin(), end_depots.begin()+ std::min(heuristic_breadth,int(end_depots.size())), end_depots.end(), proximity_cmp);
        end_depots = vector<unsigned>(end_depots.begin(), end_depots.begin()+ std::min(heuristic_breadth,int(end_depots.size())));
        
        unsigned best_end = *end_depots.begin();
        double best_end_cost = numeric_limits<float>::max();
        
        for (auto it = end_depots.begin(); it != end_depots.end(); it++)
        {
            std::vector<unsigned> path = find_path_between_intersections(current_intersection, *it, turn_penalty);
            double new_cost = compute_path_travel_time(path, turn_penalty);
            
            if (new_cost < best_end_cost)
            {
                best_end_cost = new_cost;
                best_end = *it;
            }
        }
        
        return best_end;  
    }
    
    
public:
    Delivery(const std::vector<DeliveryInfo>& _deliveries,
                const std::vector<unsigned>& _depots,
                const float _turn_penalty) :
                turn_penalty(_turn_penalty)
    {
                    
        pickup_nodes.reserve(_deliveries.size());
        dropoff_nodes.reserve(_deliveries.size());
        
        for (auto it = _deliveries.begin(); it != _deliveries.end(); it++)
        {   
            //keep a pointer to the DeliveryNode being pushed back
            //this pointer will allow us to update the traverse flag later on
            pickup_nodes.push_back(DeliveryNode(it->pickUp, pickup_nodes.end()));
            dropoff_nodes.push_back(DeliveryNode(it->dropOff, dropoff_nodes.end()));
        }
//                    
        depots = _depots;     
    }
    
    void greedy_best_tour(int T)
    {
        unsigned current_intersection = find_start();
        heuristic_breadth = determine_breadth(T);
        tour = {std::make_pair(current_intersection, pickup_nodes.end())};  //because depots have no handle, a nonsensical value is used with the depot
        
        while(true) 
        {
            
            //find the k closest legal neighbour delivery nodes by Euclidean distance
            std::vector<std::pair<unsigned, NodeHandle>> neighbours = k_closest_neighbours(current_intersection);
            
            //if the current node has no legal neighbours, all deliveries have been made
            if (neighbours.empty())
                break;
            
            std::pair<unsigned, NodeHandle> best_neighbour = *neighbours.begin();
            double best_cost = numeric_limits<float>::max();
            
            int unreachable_count = 0;
            
            //find the neighbour with the lowest cost, out of the k closest neighbours
            for (int i=0; i<neighbours.size(); i++)
            {
                if (current_intersection = neighbours[i].first)
                {
                    best_cost = 0;
                    best_neighbour = neighbours[i];
                    break;
                }
                
                std::vector<unsigned> path = find_path_between_intersections(current_intersection, neighbours[i].first, turn_penalty);
                double new_cost = compute_path_travel_time(path, turn_penalty);
                
                if (path.empty())  //no legal route to this neighbour, check next neighbour
                {
                    unreachable_count++;
                    continue;
                }
                else if (new_cost < best_cost)
                {
                    best_cost = new_cost;
                    best_neighbour = neighbours[i];
                }
            }
            
            if (unreachable_count == neighbours.size())
                std::cout << "WARNING: out of k closest legal neighbours, none are reachable. Choosing random neighbour..." << std::endl;
            
            //set the current intersection to the neighbour with the lowest cost
            current_intersection = best_neighbour.first;
            
            //mark the lowest cost neighbour as traversed
            best_neighbour.second->traversed = true;
            
            //upodate the tour
            tour.push_back(std::make_pair(current_intersection, best_neighbour.second));
            Temp = Temp/ 3;
        }
        
        tour.push_back(std::make_pair(find_end(current_intersection), dropoff_nodes.end()));
        
        //reset all traverse flags for delivery nodes
        untraverse_all();
    }
    
    void mutate_tour(int tour_index, int neighbour_index, int Temp)
    {
        heuristic_breadth = determine_breadth(Temp);
        if (neighbour_index > heuristic_breadth)//-1))
            throw std::runtime_error("neighbour_index is out of heuristic breadth range");
        
        if (tour_index == 0 || tour_index == tour.size() - 2)
            throw std::runtime_error("cannot mutate depots");
        
        if (tour_index >= tour.size() - 1)
            throw std::runtime_error("tour_index out of range");
        
        //set current intersection to the last node that will not be mutated
        unsigned current_intersection = tour[tour_index - 1].first;
        
        //mark all nodes that will not be mutated as already traversed
        //note that the sub-tour before tour_index will not be mutated
        for (int i=0; i<tour_index; i++)
            tour[i].second->traversed = true;
        
        //find the k closest neighbours of the current intersection
        //that is, the closest neighbours of the last un-mutated node
        std::vector<std::pair<unsigned, NodeHandle>> neighbours = k_closest_neighbours(current_intersection);
        
        //note that it is possible that the heuristic breadth is greater than the number of legal neighbours
        //in that case, k_closest_neighbours will return whatever legal neighbours that are available
        //as such, neighbour_index may be out of range if it is greater than the number of available neighbours 
        if (neighbour_index >= neighbours.size())
        {
            untraverse_all();
            throw std::runtime_error("neighbour index is greater than number of available neighbours");
        }
        
        //comparison function by travel time
        //note that compute_path_travel_time does not distinguish between 1. no legal route between two intersections and 2. the start and end intersections are the same
        //as such, to account for case 1, this function will set travel time to double::max() when an intersection is not reachable
        auto travel_time_cmp = [&](const std::pair<unsigned, NodeHandle>& lhs, const std::pair<unsigned, NodeHandle>& rhs)-> bool 
        {            
            std::vector<unsigned> lhs_path = find_path_between_intersections(current_intersection, lhs.first, turn_penalty);
            std::vector<unsigned> rhs_path = find_path_between_intersections(current_intersection, rhs.first, turn_penalty);
            
            double lhs_cost = compute_path_travel_time(lhs_path, turn_penalty);
            double rhs_cost = compute_path_travel_time(rhs_path, turn_penalty);
            
            if (lhs_path.empty() && lhs.first != current_intersection)
                lhs_cost = std::numeric_limits<double>::max();
            
            if (lhs_path.empty() && lhs.first != current_intersection)
                lhs_cost = std::numeric_limits<double>::max();
            
            return lhs_cost < rhs_cost;
        };
        
        
        //sorts the first neighbour_index elements in closest neighbours by travel time
        std::partial_sort(neighbours.begin(), neighbours.begin() + neighbour_index + 1, neighbours.end(), travel_time_cmp);
        
        //if the neighbour referred to by neighbour index (that is, the neighbour you would like to mutate to) is unreachable
        //throw an exception
        if (find_path_between_intersections(current_intersection, neighbours[neighbour_index].first, turn_penalty).empty() && current_intersection != neighbours[neighbour_index].first)
            throw std::runtime_error("neighbour referred to by neighbour index is not reachable");
              
       //set tour to be equal to the sub-tour that is not mutated, PLUS the first node that will be mutated
        tour.assign(tour.begin(), tour.begin() + tour_index + 1);
        
        //mutate the node referred to by tour_index
        tour[tour_index] = neighbours[neighbour_index];
        
        //we will be finding the remainder of the tour starting from the first mutated node
        current_intersection = tour[tour_index].first;
        
        //set all traversal flags up to the mutated node to true
        for (int i=0; i<tour_index+1; i++)
            tour[i].second->traversed = true;
      
        
        while(true) 
        {
            //heuristic_breadth = determine_breadth(Temp);
            //find the k closest legal neighbour delivery nodes by Euclidean distance
            neighbours = k_closest_neighbours(current_intersection);
            
            //if the current node has no legal neighbours, all deliveries have been made
            if (neighbours.empty())
                break;
            
            std::pair<unsigned, NodeHandle> best_neighbour = *neighbours.begin();
            double best_cost = numeric_limits<float>::max();
            
            int unreachable_count = 0;
            
            //find the neighbour with the lowest cost, out of the k closest neighbours
            for (int i=0; i<neighbours.size(); i++)
            {
                if (current_intersection = neighbours[i].first)
                {
                    best_cost = 0;
                    best_neighbour = neighbours[i];
                    break;
                }
                
                std::vector<unsigned> path = find_path_between_intersections(current_intersection, neighbours[i].first, turn_penalty);
                double new_cost = compute_path_travel_time(path, turn_penalty);
                
                if (path.empty())  //no legal route to this neighbour, check next neighbour
                {    
                    unreachable_count++;
                    continue;
                }
                else if (new_cost < best_cost)
                {
                    best_cost = new_cost;
                    best_neighbour = neighbours[i];
                }
            }
            
            if (unreachable_count == neighbours.size())
                std::cout << "WARNING: out of k closest legal neighbours, none are reachable. Choosing random neighbour..." <<std::endl;
            
            //set the current intersection to the neighbour with the lowest cost
            current_intersection = best_neighbour.first;
            
            //mark the lowest cost neighbour as traversed
            best_neighbour.second->traversed = true;
            
            //upodate the tour
            tour.push_back(std::make_pair(current_intersection, best_neighbour.second));
            Temp = Temp/ 3;
        }
        
        //use a similar process to find the best ending depot
        
        
        tour.push_back(std::make_pair(find_end(current_intersection), dropoff_nodes.end()));
        
        //reset all traverse flags for delivery nodes
        untraverse_all();
    }
    
    
    void set_tour(const std::vector<std::pair<unsigned, NodeHandle>>& ref)
    {
        tour = ref;
    }
    
    std::vector<std::pair<unsigned, NodeHandle>> get_tour()
    {
        return tour;
    }

};

//For simulated annealing, this function decrease the temperature
int setup_initial_Temp (const int delivery_num){
    int T = delivery_num * TEMP_FACTOR;
    return T;
}

int determine_breadth (int T){
    int breadth;
    if (T >= 150 * TEMP_FACTOR){
        breadth = 5;
    }
    else if (T < 150 * TEMP_FACTOR && T >=  100 * TEMP_FACTOR){
        breadth = 4;
    }
    else if (T < 100 * TEMP_FACTOR && T >=  50 * TEMP_FACTOR){
        breadth = 3;
    }
    else{
        breadth = 2;
    }
    return breadth;
}
struct path_and_travelTime{
    int permuation_node;
    double total_travel_time;
    std::vector<unsigned> travel_path;
};

struct path_and_travelTime path_reconstruction_and_travel_time_calcalation(std::vector<std::pair<unsigned, Delivery::NodeHandle>> travel_order, const float turn_penalty){
     //reconstruct the full path of the initial solution
    std::vector <unsigned> path_from_int_to_int;
    double highest_travel_time = -100;
    //std::vector<unsigned> travel_full_path;
    path_and_travelTime travel_order_PT;
    for (int i = 0 ; i + 1 < travel_order.size(); i++){
        //the path of a intersection pair
        path_from_int_to_int = find_path_between_intersections(travel_order[i].first, travel_order[i+1].first, turn_penalty);
        
        //there is a legal path between the "i"th intersection in the vector to the next one, insert to the full-path vector
        if (path_from_int_to_int.empty() == false){
            //keep track of which intersection pair takes the longest time
            //This tells us which portion of our initial delivery order to keep
            if (i != 0 &&  i+1 != travel_order.size() - 1){
                if (compute_path_travel_time (path_from_int_to_int, turn_penalty) > highest_travel_time){
                    travel_order_PT.permuation_node = i;
                    //initial_travel_order_subvector (initial_travel_order.begin(), initial_travel_order.begin() + (i-1));
                }
                travel_order_PT.travel_path.insert (travel_order_PT.travel_path.end(), path_from_int_to_int.begin(), path_from_int_to_int.end());
            }
        }
    }
    
    //this is to compare with other solutions to determine better delivery order
    travel_order_PT.total_travel_time = compute_path_travel_time(travel_order_PT.travel_path, turn_penalty);
    
    return travel_order_PT;
}

std::vector<unsigned> traveling_courier(const std::vector<DeliveryInfo>& deliveries, 
                                        const std::vector<unsigned>& depots, 
                                        const float turn_penalty){
    
    //an ordered vector of intersection indices where the depots and delivery points are located
    std::vector<std::pair<unsigned, Delivery::NodeHandle>> initial_travel_order;
    std::vector<std::pair<unsigned, Delivery::NodeHandle>> permutated_travel_order;
    path_and_travelTime initial_order_PT;
    path_and_travelTime permutated_order_PT;
    double fastest_delivery_order_travelTime;
    std::vector<unsigned> fastest_delivery_order_full_path;

    int delivery_node_num = deliveries.size();
    
    Temp = 70;
    
    //create a new stance of delivery task
    Delivery new_delivery (deliveries, depots, turn_penalty);
    //travel_order = new_delivery.get_tour(delivery_num);
    new_delivery.greedy_best_tour(delivery_node_num);
    initial_travel_order = new_delivery.get_tour();
    
    //reconstruct the full path of the initial solution
    initial_order_PT = path_reconstruction_and_travel_time_calcalation (initial_travel_order,turn_penalty);
    fastest_delivery_order_travelTime = initial_order_PT.total_travel_time;
    fastest_delivery_order_full_path = initial_order_PT.travel_path;
    
    //set up the initial temperature based on the size of the delivery task
    Temp = setup_initial_Temp (delivery_node_num);
    
    while (Temp > 0){
    //generate permutations of the initial solution
    //call frank's function
    // FUNCTION IS CALLED HERE
        new_delivery.mutate_tour(initial_order_PT.permuation_node, 2, Temp);//2nd closest instead of 1st closest
        permutated_travel_order = new_delivery.get_tour();
        permutated_order_PT = path_reconstruction_and_travel_time_calcalation (permutated_travel_order,turn_penalty);
        if (permutated_order_PT.total_travel_time < fastest_delivery_order_travelTime){
           fastest_delivery_order_travelTime = permutated_order_PT.total_travel_time;
           fastest_delivery_order_full_path = permutated_order_PT.travel_path;
        }
        Temp = Temp / 3;
    }
    return fastest_delivery_order_full_path;
}


//COMMENTS FOR ANN:

//every time a new delivery errand/task is needed, call the Delivery constructor to create a new "instance" of delivery
//example: Delivery new_delivery(delivery_info_vector, depot_vector, 15);


//Then, call greedy_best_tour to generate a "tour" (an ordered vector of intersection indices where the depots and delivery points are located)
//example: new_delivery.greedy_best_tour();


//the tour will traverse all delivery points
//if two or more dropoff/pickup points share the same intersection index, that particular intersection index will be traversed more than once
//in other words, the number of intersections in a tour is always equal to the number of drop off points + the number of pickup points + 2 depots (start and end)

//you may vary the parameter heuristic_breadth in greedy_best_tour()
//example: if heuristic_breadth is 3, greedy_best_tour() will only examine the three closest nodes by Euclidean distance

//Delivery objects are NOT copy-safe