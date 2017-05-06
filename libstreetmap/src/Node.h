#pragma once
#include <cmath>

struct Node
{    
    unsigned ID;    //the IntersectionIndex of the node
    double cost;    //the current lowest travel time associated with the node
       
    Node(unsigned _ID, double _cost){ID = _ID; cost = _cost; }
    Node(const Node& rhs) {ID = rhs.ID; cost = rhs.cost; }
    
    Node& operator = (const Node& rhs) {if (this == &rhs) return *this; ID = rhs.ID; cost = rhs.cost; return *this; }
    
    //comparison operator that allows ordered containers to sort nodes by cost
    //note that to update the cost of a node in an ordered container, the said node must be removed and re-inserted 
    //failing to do so will break the ordering inside the container
    bool operator < (const Node& rhs) const {return (cost < rhs.cost); }
    
    //equality operator for determining whether two nodes are the same
    bool operator == (const Node& rhs) const {return (ID == rhs.ID); }
    
};