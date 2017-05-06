#pragma once
#include "m1.h"
#include "m2_extra.h"
#include "StreetsDatabaseAPI.h"
#include "Node.h"

#include <unordered_map>
#include <set>
#include <cmath>
#include <limits>
#include <algorithm>
#include <unistd.h>
#include <string>

using namespace std;

vector<StreetSegmentIndex> reconstruct_path(IntersectionIndex current, IntersectionIndex origin);
float heuristic_cost(IntersectionIndex source, IntersectionIndex dest);
inline float fast_cosine(float val);
StreetSegmentIndex find_best_legal_segment_between_intersections(IntersectionIndex source, IntersectionIndex dest);
vector<unsigned> reconstruct_intersection_path(IntersectionIndex current, IntersectionIndex origin);

int subpath_boundary_angle(const StreetSegmentIndex current, const StreetSegmentIndex next, const IntersectionIndex boundary);
vector<string> make_directions(const vector<StreetSegmentIndex>& path, const IntersectionIndex source);
string format_readable_distance(float length_m);




