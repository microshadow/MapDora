#include "pathfinding_helper.h"
#include "m1.h"

unordered_multimap<string, unsigned> global_POI_name_ids_map;

void POI_name_to_ids_setup()
{    
    int POI_count = getNumberOfPointsOfInterest();
    for (int i=0; i<POI_count; i++)
    {
        global_POI_name_ids_map.emplace(getPointOfInterestName(i), i);
    }
      
}

vector<unsigned> find_POI_id_by_name(string name)
{
    
    auto range = global_POI_name_ids_map.equal_range(name);
    
    vector<unsigned> ids = {};
    
    for (auto iter = range.first; iter != range.second; iter++)
    {
        ids.push_back(iter->second);
    }
    
    return ids;
    
}
