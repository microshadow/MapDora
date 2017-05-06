#pragma once

#include "m1.h" //********************8=,/l 
#include "graphics.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include <cstring>

void draw_map();

typedef std::tuple<uint8_t, uint8_t, uint8_t, uint8_t> colour;

//create a struct that contains position, name of an intersection
struct intersection_data 
{
    LatLon position;
    std::string name;
};

//this is analogous to LatLon structure.Use it in the same way we access LatLon
struct cartesian {
    double x;
    double y;
};

//a struct that contains street information. Used to draw all the street segments
struct street_data {
    
    std::string street_name;
    //all the street segments that belong to the street. In vector form.
    std::vector<StreetSegmentIndex> segments_id; 
    std::vector <unsigned> curve_point_count;
};

struct feature_data
{
    feature_data()
    {}
    
    feature_data(const feature_data& src)
    {
        pt_count = src.pt_count;
        open = src.open;
        
        point = new t_point[pt_count];
        for (int i=0; i<pt_count; i++)
        {
            point[i] = src.point[i];
        }
        feature_colour = src.feature_colour; //this is okay because the address does not change during runtime
    }
    
    feature_data& operator= (const feature_data& src)
    {
        
        if (this == &src)
            return *this;
        
        delete[] point;
        
        pt_count = src.pt_count;
        open = src.open;
        
        point = new t_point[pt_count];
        for (int i=0; i<pt_count; i++)
        {
            point[i] = src.point[i];
        }
        
        feature_colour = src.feature_colour; //this is okay because the address does not change during runtime
        
        return *this;
    }
    
    ~feature_data()
    {
        delete[] point;
    }
    
    t_point* point;
    int pt_count;
    bool open;
    colour* feature_colour;   
};

struct showing_path {
    colour st1;
    colour st2;
    colour st3;
    colour st4;
    colour st5;
    
    showing_path()
    {
    }
    
    showing_path (colour _st1, colour _st2, colour _st3, colour _st4, colour _st5): //initializer list 
        st1(_st1),
        st2(_st2),
        st3(_st3),
        st4(_st4),
        st5(_st5)
    {}
};

struct colour_scheme
{
    //each tuple element corresponds to r, g, b, a
    
    colour_scheme(){}
    
    colour_scheme  (
                            colour _background,
                            colour _text_streetname, 
                            colour _text_POIname, 
                            colour _street_major, 
                            colour _street_highway, 
                            colour _street_minor, 
                            colour _feature_water, 
                            colour _feature_green, 
                            colour _feature_building,
                            colour _feature_generic,
                            colour _feature_shore,
                            colour _feature_sand,
                            colour _intersection,
                            colour _POI,
                            colour _oneway_arrow,
                            colour _highlight
                            ):
                            background(_background),
                            text_streetname(_text_streetname), 
                            text_POIname(_text_POIname), 
                            street_major(_street_major),
                            street_highway(_street_highway),
                            street_minor(_street_minor), 
                            feature_water(_feature_water), 
                            feature_green(_feature_green),
                            feature_building(_feature_building),
                            feature_generic(_feature_generic),
                            feature_shore(_feature_shore),
                            feature_sand(_feature_sand),
                            intersection(_intersection),
                            POI(_POI),
                            oneway_arrow (_oneway_arrow),
                            highlight (_highlight)
    {}
     
    colour background;
    colour intersection;
    colour POI, text_POIname;
    colour text_streetname, street_major, street_highway, street_minor;
    colour oneway_arrow;
    colour feature_water, feature_green, feature_building, feature_shore, feature_generic, feature_sand;   
    colour highlight;
};

std::vector<LatLon> find_POI_positions_by_name(std::string name);