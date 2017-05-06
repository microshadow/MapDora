#include "m2_extra.h"
#include "m2.h"
#include "m3.h"
#include "m3_extra.h"
#include "pathfinding_helper.h"
#include "fuzzy_search.h"
#include "OSMDatabaseAPI.h"
#include <iostream>
#include <cstdlib>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <set>
#include <boost/bind.hpp>
#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <readline/readline.h>
#include <readline/history.h>

//helper functions
//return LatLon of the closest intersection of that POI,given by a LatLon POI's position
LatLon get_closest_intersection_position(LatLon POI_position);

//current map
static std::string current_path = "/cad2/ece297s/public/maps/toronto_canada.streets.bin";

//world coordinate bound box
const t_bound_box INITIAL_COORDS = t_bound_box(0, 0, 1000, 1000);
const t_bound_box INITIAL_SEARCHBAR = t_bound_box(0,800,350,850);
const t_bound_box SEARCHBAR_TITLE = t_bound_box(0,850,350,880);
const t_bound_box SEARCHBAR_TEXT = t_bound_box(50,850,300,880);

t_bound_box SCREEN_SIZE;
float x_left;
float y_bottom;
float x_right;
float y_top;
t_bound_box HELP_WINDOW_SIZE;

IntersectionIndex start_intersection;
IntersectionIndex end_intersection;

//colour scheme, global_colour_scheme is the active scheme
static colour_scheme global_colour_scheme; //set all colors using global_colour_scheme
static colour_scheme scheme_day;
static colour_scheme scheme_night;
static colour_scheme scheme_deuteranopia;
static colour_scheme scheme_protanopia;
static colour_scheme scheme_tritanopia;
static short int active_scheme = 0;

static showing_path colour_for_find_path; //this is colour scheme for showing the shortest path (M3)

//graphic data structure initialization functions
void colour_scheme_setup();
void path_showing_setup(); //set up colour scheme for path_showing (M3)
colour path_clr;
int previous_street_highlighted = -1; //for (M3)
int path_showing_color_rotation = 1; //keeps track of which color is being used to highlight a street in shortest path (M3)
void feature_setup();
void intersection_setup();
void street_setup();
void POI_setup();

//Street/POI/Feature set up function std::vector<StreetIndex> 
void Street_vector_setup();
void Feature_vector_setup();

//graphic data structure cleanup functions
inline void feature_cleanup();
inline void intersection_cleanup();
inline void street_cleanup();
inline void POI_cleanup();

//void input_using_readline();
//variables for coordinate transformation
static double lat_avg;
static double min_x, min_y, max_x, max_y; //note: do not use for boundary points
static double coordinate_scaling_factor;

//coordinate transformation helper functions
void coord_setup();
double lon_to_x(double lon);
double lat_to_y(double lat);
double x_to_lon(double x);
double y_to_lat(double y);

//parameters for graphics with constant on-screen size, in screen coordinates
const float POI_ICON_RADIUS = 5;
const float POI_TEXT_OFFSET = 12;//the radius of the highlight circle by mouse click
//the radius of highlighting intersections or POI
const double RADIUS = 2.25;
const double POI_HIGHLIGHT_RADIUS = 5;

//zoom related variables
static double POI_vis_world_area_threshold;
static double open_feature_vis_world_area_threshold;
static double street_vis_world_area_threshold;
static double street_arrow_vis_world_area_threshold;
static double minor_street_vis_world_area_threshold;

//unit in meters; POI icons are visible when the zoomed in map area is smaller than the square with side length POI_zoom_side_length
const double POI_VIS_REAL_DIM_THRESHOLD = 1000; 
const double STREET_VIS_REAL_DIM_THRESHOLD = 800;
const double MINOR_STREET_VIS_REAL_DIM_THRESHOLD = 500;
const double STREET_ARROW_VIS_REAL_DIM_THRESHOLD = 1800;
const double OPEN_FEATURE_VIS_REAL_DIM_THRESHOLD = 13000;

//OSM element count
static int street_segment_count;
static int street_count;
static int intersection_count;
static int POI_count;
static int feature_count;

//callbacks for event-driven window handling.
void draw_screen(void);
void act_on_find_button(void (*drawscreen_ptr) (void));
void act_on_load_map_button(void (*drawscreen_ptr) (void));
void act_on_addition_find_func(void (*drawscreen_ptr) (void));
void act_on_findPath_button(void (*drawscreen_ptr)(void));
void act_on_findPOIPath_button(void (*drawscreen_ptr)(void));
void act_on_button_press(float x, float y, t_event_buttonPressed event);
void act_on_mouse_move(float x, float y);
void act_on_keypress(char c, int keysym);
void act_on_mouse_button(float x, float y, t_event_buttonPressed buttonPressed);
void act_on_second_mouse_button(float x, float y, t_event_buttonPressed buttonPressed);
void act_on_colour_scheme(void (*drawscreen_ptr) (void));
void act_on_help_button(void (*drawscreen_ptr) (void));

//data structure for intersections
std::vector<intersection_data> intersections;
std::vector<std::pair<double, double>> int_cartesian_coor;

//data structures for street/street segments
std::vector<street_data> streets;

//data structures for closed_features
static std::vector<feature_data> closed_features;
static std::vector<feature_data> open_features;
static std::vector<std::pair<unsigned, double>> closed_feature_precedence;
double find_feature_area(t_point pt[], int n);

//create another vector as same as above just to avoid redundant and segmentation fault.
void POI_find_setup();
void POI_find_cleanup();

//for highlighting intersections (either by find button or clicking)
static bool find_highlight = false;
static bool click_highlight = false;
static bool path_visualization = false;
static bool help_window = false; //for help_window (M3))

//data structures for findpath
std::vector<StreetSegmentIndex> path_highlight;
//data structure for find_POI_path
std::vector<StreetSegmentIndex> POI_path_highlight;

//store POI name as key while POI LatLon as value into an unordered_map
std::unordered_multimap<std::string, LatLon> POI_info;

static std::vector<std::pair<double, double>> intersection_highlights;
static std::vector<std::pair<double, double>> intersection_highlights_1;
//create the data structure as same as above to avoid redundant
static std::vector<std::pair<double, double>> intersection_highlights_2;

static std::vector<std::pair<double, double>> intersection_highlights_findPOI;
static std::vector<std::pair<double, double>> POI_highlights;
//for the FindPOIPath button, as same as above in case of redundant and segmentation fault.
static std::vector<std::pair<double, double>> POI_highlights_POIPath;
//for the closest intersections from the POI
static std::vector<std::pair<double,double>> closest_intersection_from_POI;

//for the findPath button
static bool find_path_input_1 = false; //for intersection now, will develop further
static bool find_path_input_2 = false; //for intersection or POI
//for the finPOIPath button
static bool findPOIPath_input_1 = false; //for the intersection determined by command
static bool findPOIPath_input_2 = false; //for the POI by 2nd input

//for highlighting in mouse click
static double click_highlight_x;
static double click_highlight_y;

//for highlighting in 2nd mouse click
static bool second_click_highlight = false;
static double second_click_x;
static double second_click_y;

//for highlight POI
static bool highlight_POI = false;

//for drawing the closest intersection
static bool closest_intersecion_highlighting = false;

void highlight_cleanup();

void draw_map()
{    
    //initializes all data structures
    coord_setup();
    colour_scheme_setup();
    path_showing_setup();
    intersection_setup();
    street_setup();
    feature_setup();
    
    //set up Street/POI/Feature names
    POI_find_setup();
    
    //start message
    std::cout << "Starting map application..." << std::endl;
    
    //creates a new window for the map application
    init_graphics("Map", t_color(234, 234, 234, 255));
    
    
    set_drawing_buffer(OFF_SCREEN);
    set_visible_world(INITIAL_COORDS);
    set_mouse_move_input(true);
    set_keypress_input(true);
    
    create_button("Window", "Find", act_on_find_button);
    create_button("Find", "Load Map", act_on_load_map_button);
    create_button("Load Map", "Theme", act_on_colour_scheme);
    create_button("Find", "FindPOI",act_on_addition_find_func);
    create_button("Exit", "Help", act_on_help_button);
    create_button("FindPOI", "inter->inter",act_on_findPath_button);
    create_button("inter->inter", "inter->POI",act_on_findPOIPath_button);
    
    event_loop(act_on_mouse_button, act_on_mouse_move, act_on_keypress, draw_screen);
    
    close_graphics();
    
    //clean up data structures
}

void draw_screen()
{
    clearscreen(); //erase old graphics

    //multiply by this value to fix on-screen size of primitive
    double factor = get_visible_world().get_width() / std::max(INITIAL_COORDS.get_width(), INITIAL_COORDS.get_height());
    
    //draw background
    {
        setcolor(global_colour_scheme.background);
        fillrect(INITIAL_COORDS);
    }
    
    //draw closed features
    {
        for (auto iter=closed_feature_precedence.begin(); iter != closed_feature_precedence.end(); iter++)
        {
            setcolor(*(closed_features[iter->first].feature_colour));
            fillpoly(closed_features[iter->first].point, closed_features[iter->first].pt_count);
        }
    }
    
    //draw open features
    {
        if(LOD_area_test(open_feature_vis_world_area_threshold))
        {
            setlinewidth(1);
            setlinestyle(SOLID, ROUND);

            for (auto iter=open_features.begin();iter!=open_features.end();iter++)
            {
                for (int i=1; i<(iter->pt_count); i++)
                {
                    setcolor(*(iter->feature_colour));
                    drawline((iter->point)[i], (iter->point)[i-1]);
                }
            }
        }
    }
    
    //draw intersections
    {
        setcolor(global_colour_scheme.intersection);
        for (int i=0; i < int_cartesian_coor.size(); i++)
        {
            double x = int_cartesian_coor[i].first;
            double y = int_cartesian_coor[i].second;
            fillarc(x, y, 0.0001, 0, 360); 
        }
    }
    
    //draw street segments 
    for (unsigned int st_id = 0; st_id < streets.size(); st_id ++){
        for (unsigned int j = 0; j <streets[st_id].segments_id.size(); j++){
            StreetSegmentIndex current_seg_id = streets[st_id].segments_id[j];
            colour original_clr;
            colour street_clr;
            int width_factor;
            //based on the classification (using speed limit)of the segment, colour and the width of a line change
            //1. Highway:  > 80Km/h
            if (getStreetSegmentInfo(current_seg_id).speedLimit >= 80){ 
                width_factor = 3;
                setlinewidth(4);
                street_clr = global_colour_scheme.street_highway;
            }
            //2. major roads:  greater than 60km/h but slower than  80Km/h
            else if (getStreetSegmentInfo(current_seg_id).speedLimit >= 60){ //this case is for major roads
                width_factor = 2;
                setlinewidth(2);
                street_clr = global_colour_scheme.street_major;
            }
            //3. minor roads:  slower than  60Km/h
            else{        
                width_factor = 1;
                setlinewidth(1);
                street_clr = global_colour_scheme.street_minor;
            }
            
            //overwrite the street colour if it needs to be highligted for shortest path
            if (path_visualization && std::find(path_highlight.begin(), path_highlight.end(), current_seg_id)!= path_highlight.end()){
                //this is the first street that is getting highlighted
                if (previous_street_highlighted == -1){
                    previous_street_highlighted = st_id;
                    path_clr = colour_for_find_path.st1; //set to the first colour
                    path_showing_color_rotation++;
                }
                //change color only if this is different street from the previous one
                else if (st_id != previous_street_highlighted){ 
                    previous_street_highlighted = st_id;
                    if (path_showing_color_rotation == 1){
                        path_showing_color_rotation++;
                        path_clr = colour_for_find_path.st1;
                    }
                    else if (path_showing_color_rotation == 2){
                        path_showing_color_rotation++;
                        path_clr = colour_for_find_path.st2;
                    }
                    else if (path_showing_color_rotation == 3){
                        path_showing_color_rotation++;
                        path_clr = colour_for_find_path.st3;
                    }
                    else if (path_showing_color_rotation == 4){
                        path_showing_color_rotation++;
                        path_clr = colour_for_find_path.st4;
                    }
                    else if (path_showing_color_rotation == 5){
                        path_showing_color_rotation = 1; //reset to colour st1
                        path_clr = colour_for_find_path.st5;
                    }
                }
                street_clr = path_clr;
            }
            setcolor(street_clr);
            original_clr = street_clr;
            
            //******************all the variables *****************************
            LatLon from_lat_lon; 
            LatLon to_lat_lon;
            
            double from_x;
            double from_y;
            
            double to_x;
            double to_y;
            
            double x_difference;
            double y_difference;
            
            double normalization;
            
            double unit_vector_x;
            double unit_vector_y;
            
            double start_pt_x;
            double start_pt_y;
            
            double normal_vector_x;
            double normal_vector_y; 
            
            bool text_rotation_flag;
            
            t_point arrow_head[3];
            //********************end of variable declaration*******************
            
            from_lat_lon = getIntersectionPosition(getStreetSegmentInfo(current_seg_id).from);
 
            for (unsigned k = 0; k <= streets[st_id].curve_point_count[j]; k++){
                if (streets[st_id].curve_point_count[j] == k){
                    to_lat_lon = getIntersectionPosition(getStreetSegmentInfo(current_seg_id).to);
                }
                else{
                    to_lat_lon = getStreetSegmentCurvePoint (current_seg_id, k);
                    
                }
                
                from_x = lon_to_x(from_lat_lon.lon());
                from_y = lat_to_y(from_lat_lon.lat());

                to_x = lon_to_x(to_lat_lon.lon());
                to_y = lat_to_y(to_lat_lon.lat());
                    
                drawline (from_x, from_y, to_x, to_y); //draw street segments
                
                from_lat_lon = to_lat_lon;
                
                
                //In order to view street names and one-way arrows, the user need to zoom in to a certain level
                if (LOD_area_test(street_arrow_vis_world_area_threshold) && k % 7 == 0)
                { //k==condition is to prevent drawing arrows and segments for each curve point
                    x_difference = to_x - from_x;
                    y_difference = to_y - from_y;
                    normalization = sqrt (pow ((x_difference),2) + pow((y_difference),2));
                    unit_vector_x = x_difference / normalization;
                    unit_vector_y = y_difference / normalization;
                    normal_vector_x = -unit_vector_y;
                    normal_vector_y = unit_vector_x;
                    text_rotation_flag = false;

                    //Drawing arrows for one way streets to indicate directions. Ignore those street segments that are connected to a single intersection
                    if (getStreetSegmentInfo(current_seg_id).oneWay == true && getStreetSegmentInfo(current_seg_id).from != getStreetSegmentInfo(current_seg_id).to){

                        start_pt_x = from_x + (0.8)*(unit_vector_x);
                        start_pt_y = from_y + (0.8)*(unit_vector_y);

                        //This if statement makes sure that the arrow body is not longer than the actual street segments
                        if (normalization >=1){ 
                            to_x = from_x + unit_vector_x;
                            to_y = from_y + unit_vector_y;

                            //draw a triangular arrow head
                            arrow_head[0].x = start_pt_x + normal_vector_x * 0.125 * width_factor ;
                            arrow_head[0].y = start_pt_y + normal_vector_y * 0.075 * width_factor;

                            //Consider the thickness of the arrow body (this is because of street segment width) when drawing arrows
                            if (width_factor != 3){
                            arrow_head[1].x = to_x;
                            arrow_head[1].y = to_y;
                            } 
                            else {
                                arrow_head[1].x = to_x + unit_vector_x * 0.07;
                                arrow_head[1].y = to_y + unit_vector_y * 0.25;;
                            } 
                            arrow_head[2].x = start_pt_x + (-normal_vector_x) * 0.100 * width_factor;
                            arrow_head[2].y = start_pt_y + (-normal_vector_y) * 0.060 * width_factor;

                            setcolor(global_colour_scheme.oneway_arrow);
                            drawline (from_x, from_y, to_x, to_y);//draw arrow "body" - the line part of the arrow
                            fillpoly (arrow_head, 3);

                        }
                        else{
                            setcolor(global_colour_scheme.oneway_arrow);

                            //if the angle of rotation is already calculated, then don't need to do the calculation again
                            if (text_rotation_flag != true){
                                settextrotation (int(atan(unit_vector_y/unit_vector_x) * 180 / PI));
                            }
                            text_rotation_flag = false; //reset the flag

                            //setfontsize(1 + width_factor);

                            //there are two possibilities for arrows: "->" OR "<-" 
                            if (to_x> from_x){
                                drawtext(t_point((from_x + to_x) / 2, (from_y + to_y) / 2), "->");
                            }
                            else if (to_x < from_x){
                                drawtext(t_point((from_x + to_x) / 2, (from_y + to_y) / 2),"<-");
                            }
                        }
                    }
                }
                
                //make arrows pop up first and then the street names
                if (LOD_area_test(street_vis_world_area_threshold)&& k % 7 == 0){
                    
                    if ((width_factor == 1 && LOD_area_test(minor_street_vis_world_area_threshold)) || width_factor != 1){
                        //indicate street name every 5 segments if the street has multiple segments OR any street that has a single segment 
                        if ((j % 5 == 0 || streets[st_id].segments_id.size() == 1)||normalization > 10){
                            settextrotation (int(atan(unit_vector_y/unit_vector_x) * 180 / PI));
                            setcolor(global_colour_scheme.text_streetname);
                            //drawtext(t_point((from_x + to_x) / 2, (from_y + to_y) / 2), getStreetName(st_id)); 
                            drawtext(t_point(((from_x + to_x) / 2) + normal_vector_x * (width_factor/10), ((from_y + to_y) / 2) + normal_vector_y * (width_factor/10)), getStreetName(st_id)); 
                            text_rotation_flag = true;
                        }
                    }
                }
                setcolor(original_clr);
            }
            
        }
    }
    
    //draw POIs
    {  
        settextattrs(9, 0);
        if (LOD_area_test(POI_vis_world_area_threshold))
        {   
            
            for (int i=0; i<POI_count; i++)
            {   
                    double x = lon_to_x(getPointOfInterestPosition(i).lon());
                    double y = lat_to_y(getPointOfInterestPosition(i).lat());
                    
                    setcolor(global_colour_scheme.POI);
                    //debugging only, comment for final build
                    fillarc(x, y, POI_ICON_RADIUS * factor, 0, 360);

                    //NOTE: draw_surface now centers at bottom center
                    //icon not used for now to increase speed
                    //draw_surface(load_png_from_file("arrow_BW_thin_left.png"), x, y);
                    setcolor(global_colour_scheme.text_POIname);
                    drawtext(t_point(x,y - POI_TEXT_OFFSET * factor), getPointOfInterestName(i));            
            }
        }
             
    }
    
    //draw the intersection by left mouse click
    if(click_highlight)
    {    
        setcolor(global_colour_scheme.highlight);
        fillarc(click_highlight_x, click_highlight_y, POI_HIGHLIGHT_RADIUS*factor, 0, 360);
    }
    
    //draw the intersection by right mouse click
    if(second_click_highlight)
    {
        setcolor(6);
        fillarc(second_click_x, second_click_y, POI_HIGHLIGHT_RADIUS*factor, 0, 360);
    }
    
    //draw the intersection given by two street names
    if (find_highlight)
    {
        setcolor(global_colour_scheme.highlight);
        for (auto iter=intersection_highlights.begin(); iter != intersection_highlights.end(); iter++)   
            fillarc(iter->first, iter->second, POI_HIGHLIGHT_RADIUS*factor, 0, 360);
    }
    
    //draw the 1st intersection by command 
    if(find_path_input_1)
    {
        setcolor(4);
        for (auto iter=intersection_highlights_1.begin(); iter != intersection_highlights_1.end(); iter++)   
            fillarc(iter->first, iter->second, POI_HIGHLIGHT_RADIUS*factor, 0, 360);
    }
    
    //draw the 2nd intersection by command
    if(find_path_input_2)
    {
        setcolor(6);
        for (auto iter=intersection_highlights_2.begin(); iter != intersection_highlights_2.end(); iter++)   
            fillarc(iter->first, iter->second, POI_HIGHLIGHT_RADIUS*factor, 0, 360);
    }
    
    //draw the highlighted intersection input by command
    if(findPOIPath_input_1)
    {
        setcolor(13);
        for (auto iter=intersection_highlights_findPOI.begin(); iter != intersection_highlights_findPOI.end(); iter++)   
            fillarc(iter->first, iter->second, POI_HIGHLIGHT_RADIUS*factor, 0, 360);
    }
    
    //draw the highlighted POI for 2nd input in FindPOIPath button
    if(findPOIPath_input_2)
    {
        setcolor(5);
        for(auto iter=POI_highlights_POIPath.begin(); iter!=POI_highlights_POIPath.end(); iter++)
        {
            fillarc(iter->first, iter->second, POI_HIGHLIGHT_RADIUS*factor, 0, 360);
        }
    }
    
    //draw the highlighting intersection from the POI position
    if(closest_intersecion_highlighting)
    {
        setcolor(18);
        for(auto iter = closest_intersection_from_POI.begin();iter!=closest_intersection_from_POI.end();iter++)
        {
            fillarc(iter->first, iter->second, POI_HIGHLIGHT_RADIUS*factor, 0, 360);
        }
    }
    
    //what to draw when the help button is pressed by the user
    if (help_window){
        set_coordinate_system(GL_SCREEN); //need to use screen coordinate to make fixed screen, not World coordinate
        SCREEN_SIZE = get_visible_screen();
        float x_center = (SCREEN_SIZE.left() + SCREEN_SIZE.right())/2;
        float y_center = (SCREEN_SIZE.bottom() + SCREEN_SIZE.top())/2;
        //float width = SCREEN_SIZE.get_width()/4;
        float y_offset = 63;
        
        t_point upper_left_corner (x_center, y_center - y_offset);
        draw_surface(load_png_from_file("help_instructions_ver2.png"),upper_left_corner);
        set_coordinate_system(GL_WORLD); //set back to world coordinate
    }
    
    //draw the highlighted POI
    if(highlight_POI)
    {
        setcolor(global_colour_scheme.highlight);
       
        
        for(auto iter=POI_highlights.begin(); iter!=POI_highlights.end(); iter++)
        {
            fillarc(iter->first, iter->second, POI_HIGHLIGHT_RADIUS*factor, 0, 360);
        }
    }
    
    //draw scale
    {
        t_bound_box vis_world = get_visible_world();
        
        float x1 = vis_world.left();
        float y1 = vis_world.bottom();
        float x2 = vis_world.right();
        float y2 = vis_world.top();

        
        float width = vis_world.get_width();
        float height = vis_world.get_height();
        
        float sc_x = x1 + width * 0.95;
        float sc_y = y1 + height * 0.05;
        
        float sc_width = width * 0.02;
        float sc_height = height * 0.005;
        
        setcolor(BLACK);
        fillrect(sc_x, sc_y, sc_x+sc_width, sc_y+sc_height);
        setcolor(WHITE);
        fillrect(sc_x-sc_width, sc_y, sc_x, sc_y+sc_height);
        setcolor(BLACK);
        fillrect(sc_x- 2 * sc_width, sc_y, sc_x-sc_width, sc_y+sc_height);
        
        float horizontal_dist = find_distance_between_two_points(LatLon(y_to_lat(INITIAL_COORDS.bottom()), x_to_lon(INITIAL_COORDS.left())), 
                                                                    LatLon(y_to_lat(INITIAL_COORDS.bottom()), x_to_lon(INITIAL_COORDS.right())));
        
        int scale_val = int(horizontal_dist * (sc_width*3/INITIAL_COORDS.get_width())); //m
        
        settextattrs(12, 0);
        setcolor(global_colour_scheme.text_streetname);
        drawtext(t_point((sc_x-sc_width + sc_x)/2 , sc_y - 10 * factor), std::to_string(scale_val)+ "m");
        
    }
    //reset these variables to avoid path colours changing as the user zoom in/out
    previous_street_highlighted = -1; 
    path_showing_color_rotation = 1;
    copy_off_screen_buffer_to_screen();
}

void POI_find_setup()
{  
    std::vector<POIIndex> POI_ids;
    std::vector<std::string> POI_names;
    
    for(unsigned i=0; i<POI_count; i++)
    {
        POI_ids.push_back(i);
    }
    
    for(std::vector<POIIndex>::iterator iter = POI_ids.begin(); iter!=POI_ids.end(); iter++)
    {
        POI_names.push_back(getPointOfInterestName(*iter));
        POI_info.insert(std::make_pair(getPointOfInterestName(*iter), getPointOfInterestPosition(*iter)));
    }
    
}

void POI_find_cleanup()
{
    POI_info.clear();
}

//help button callback function for M3
void act_on_help_button(void (*drawscreen_ptr) (void)){
    help_window = true;
    drawscreen_ptr(); 
}

void remove_help_window(void (*drawscreen_ptr) (void)){
    help_window = false;
    drawscreen_ptr();
    
}

/*
 *This is primarily for closing Help window with ESC key
 */
void act_on_keypress (char key_pressed, int keysym){
    //This if statement makes sure that it only reacts to ESC key
    if (keysym == 65307){ 
        help_window = false;
        draw_screen();
    }
}

void act_on_mouse_move(float x, float y) 
{
    // function to handle mouse move event, the current mouse position in the current world coordinate
    // system (as defined in your call to init_world) is returned

    //debugging only
    //std::cout << "Mouse move at " << x << "," << y << ")\n";
}

void act_on_mouse_button(float x, float y, t_event_buttonPressed buttonPressed)
{
   
    path_highlight.clear();
    POI_path_highlight.clear();
    intersection_highlights.clear();
    intersection_highlights_1.clear();
    intersection_highlights_2.clear();
    intersection_highlights_findPOI.clear();
    POI_highlights.clear();
    
    double user_click_lat;// = y_to_lat(y);
    double user_click_lon;// = x_to_lon(x);
    //LatLon click_position(user_click_lat, user_click_lon);
    
    double user_click_lat_2;// = y_to_lat(y);
    double user_click_lon_2;// = x_to_lon(x);
    //LatLon click_position_2(user_click_lat_2, user_click_lon_2);
    
    //we want to find the most optimal path between those two intersections, local variable
    //IntersectionIndex start_intersection;
    //IntersectionIndex end_intersection;
    
    //first click represents the start intersection
    if(buttonPressed.button == 1)
    {
        //std::cout<<"button 1 pressed"<<std::endl; //debugging - Ann
        user_click_lat = y_to_lat(y);
        user_click_lon = x_to_lon(x);
        
        LatLon click_position(user_click_lat, user_click_lon);

        click_highlight = true;
        second_click_highlight = false;
        find_highlight = false;
        highlight_POI = false;
        find_path_input_1 = false;
        find_path_input_2 = false;
        findPOIPath_input_1 = false;
        findPOIPath_input_2 = false;
        closest_intersecion_highlighting = false;
        path_visualization = false;
        
        std::cout << std::endl;
        //std::cout << "the clicked coordinate is " << "(" << x << " , " << y << ")" << std::endl;
        //std::cout << "the clicked location is " << "(" << user_click_lon << " , " << user_click_lat << ")" << std::endl;
        
        //find the nearest intersection via the map coordinates by user click 
        //given a range, for our design, radius is 0.75
        IntersectionIndex closest_intersection = find_closest_intersection(click_position); 
        LatLon closest_intersection_position = getIntersectionPosition(closest_intersection);
        //local variable
        start_intersection = closest_intersection;
        std::cout<<"Starting at: "<<getIntersectionName(start_intersection)<<std::endl;
        
        //to calculate the distance
        //first convert the LatLon of nearest intersection to coordinate (x1,y1);
        double intersection_x = lon_to_x(closest_intersection_position.lon());
        double intersection_y = lat_to_y(closest_intersection_position.lat());
        
        click_highlight_x = intersection_x;
        click_highlight_y = intersection_y;
        
        //then calculate the distance
        double distance = sqrt( pow(x - intersection_x, 2) + pow(y - intersection_y, 2) );
        
        //if the distance is > our setting radius
        if(distance > RADIUS)
        {
            std::cout << "sorry, the point you click is not an intersection..." << std::endl;
            std::cout << "please try again" << std::endl;
            click_highlight = false;
        }
        
        //if the distance is <= our setting radius
        if((distance < RADIUS) || (distance == RADIUS))
        {
            //to debug only
//            std::cout << "--------------------------------------------------------" << std::endl;
//            std::cout << "the closest intersection coordinate is ( " << intersection_x << ", " << intersection_y << " )" << std::endl;
//            
//            std::cout << "the closest intersection is " << closest_intersection << std::endl;
//            std::cout << std::endl;
        
        //output the street name about this intersection
        std::vector<std::string> street_from_intersection = find_intersection_street_names(closest_intersection);
        
        for(StreetIndex i=0; i<street_from_intersection.size(); i++) //**Get rid of this for loop if we don't need it ************
        {
            std::cout << "one of the street at this given intersection is " << street_from_intersection[i] << std::endl;
        }
        
//        std::vector<StreetSegmentIndex> segments_from_intersection = find_intersection_street_segments(closest_intersection);
//        for(StreetSegmentIndex i=0; i<segments_from_intersection.size(); i++)
//        {
//            std::cout << "one of the segments of this intersection is " << segments_from_intersection[i] << std::endl;
//
//            StreetSegmentInfo segment_info = getStreetSegmentInfo(segments_from_intersection[i]);
//            std::cout << "the speed limit is " << segment_info.speedLimit << std::endl;
//
//            if(segment_info.oneWay)
//            {
//                std::cout << "it is a one-way road" << std::endl;
//                std::cout << std::endl;
//            }  
//        }
               
        }
         
    }
    
    //draw_screen();
    
    //for the 2nd click by right mouse button, presenting the 2nd intersection
     if(buttonPressed.button == 3)
      {
             
        user_click_lat_2 = y_to_lat(y);
        user_click_lon_2 = x_to_lon(x);
        LatLon click_position_2(user_click_lat_2, user_click_lon_2);
        
        if(click_highlight)
        {
            second_click_highlight = true;
            path_visualization = true;
        }
        
        //second_click_highlight = true;
        
        //find the nearest intersection via the map coordinates by user 2nd click 
        //given a range, for our design, radius is 0.75
        IntersectionIndex closest_intersection_2 = find_closest_intersection(click_position_2); 
        LatLon closest_intersection_position_2 = getIntersectionPosition(closest_intersection_2);
        end_intersection = closest_intersection_2;

        //to calculate the distance
        //first convert the LatLon of nearest intersection to coordinate (x2,y2);
        double intersection_x_2 = lon_to_x(closest_intersection_position_2.lon());
        double intersection_y_2 = lat_to_y(closest_intersection_position_2.lat());
        
        second_click_x = intersection_x_2;
        second_click_y = intersection_y_2;
        
        //then calculate the distance
        double distance_2 = sqrt( pow(x - intersection_x_2, 2) + pow(y - intersection_y_2, 2) );
        
        //if the distance is > our setting radius
        if(distance_2 > RADIUS)
        {
            std::cout << "sorry, the point you click is not an intersection..." << std::endl;
            std::cout << "please try again" << std::endl;
            second_click_highlight = false;
        }
        
         //if the distance is <= our setting radius
        if((distance_2 < RADIUS) || (distance_2 == RADIUS))
        {
        //output the street name about this intersection
        std::vector<std::string> street_from_intersection = find_intersection_street_names(closest_intersection_2);
//        for(StreetIndex i=0; i<street_from_intersection.size(); i++)
//        {
//            std::cout << "one of the street at this given intersection is " << street_from_intersection[i] << std::endl;
//        }
        
        //path_visualization = true;
        path_highlight = find_path_between_intersections(start_intersection,end_intersection,15);
        
        std::vector<std::string> dir = make_directions(path_highlight, start_intersection);
        std::cout << "Ending intersection is " << getIntersectionName(end_intersection) << std::endl;
        for (auto iter = dir.begin(); iter != dir.end(); iter++)
            std::cout << *iter << std::endl;

        }
      }
    
    draw_screen();
}

void act_on_find_button(void (*drawscreen_ptr) (void))
{
    // Callback function for the new button we created. This function will be called
    // when the user clicks on the button. It just counts how many
    // times you have clicked the button.  

    
    std::cout << std::string(100, '\n' );
    //to clear the previous highlights
    highlight_cleanup();
    
    update_message("Please input the two streets name in terminal...");
    
    std::string street_name1;
    std::string street_name2;
    std::cout << "please enter 1st street names" << std::endl;
    //getline(std::cin,street_name1);
    street_name1 = input_using_readline(1);
    
    std::cout << "please enter 2nd street names" << std::endl;
    //getline(std::cin,street_name2);
    street_name2 = input_using_readline(1);
    
    //can find the intersection of two streets by the function we implement in m1
    std::vector<IntersectionIndex> intersection_ids = find_intersection_ids_from_street_names(street_name1, street_name2);
    std::cout << "there are total " << intersection_ids.size() << " intersection(s) along those two streets" << std::endl;
    
    if(intersection_ids.size()==0)
    {
        std::cout << "there are no intersections between those two streets" << std::endl;
    }
    if(intersection_ids.size()!=0)
    {
        for(unsigned i=0; i<intersection_ids.size();i++)
        {
            std::cout << "one of the intersections is " << intersection_ids[i] << std::endl;
            
            //return the LatLon from the coordinate(x,y) of intersection.
            LatLon intersection_position = getIntersectionPosition(intersection_ids[i]);
            
            //return the real world coordinate (x,y) from intersection position
            double intersection_x = lon_to_x(intersection_position.lon());
            double intersection_y = lat_to_y(intersection_position.lat());
            
            //to debug if the intersection coordinates are output correctly
            std::cout << "the coordinates is (" << intersection_x << ", " << intersection_y << ")" << std::endl;
            std::cout << "the real world location is (" << intersection_position.lon() << ", " << intersection_position.lat() << ")" << std::endl;
            
            find_highlight = true;
            intersection_highlights.push_back(std::make_pair(intersection_x, intersection_y));
            
            std::vector<StreetSegmentIndex> segments_incidents = find_intersection_street_segments(intersection_ids[i]);
            StreetSegmentInfo seg_info = getStreetSegmentInfo(segments_incidents[i]);
            std::cout << "the speed limit is " << seg_info.speedLimit << std::endl;
            if(seg_info.oneWay)
            {
                std::cout << "this is a one-way road " << std::endl;
            }
        }
    } 

    update_message(" ");
    
    // Re-draw the screen (a few squares are changing colour with time)
    drawscreen_ptr();
}

//the "find POI" feature
void act_on_addition_find_func(void (*drawscreen_ptr) (void))
{
    //to clear the previous highlights
    highlight_cleanup();
    
    update_message("search the POI you want to find...");
    std::string user_input_name;
    std::cout << "please enter a name (press tab for fuzzy search): " << std::endl;
    user_input_name = input_using_readline(2);
    
    auto range = POI_info.equal_range(user_input_name);
    
    if(range.first == range.second)
    {
        std::cout << "Sorry, no matching POI for your input..." << std::endl;
                std::cout << "please try again" << std::endl;
    }
    for(auto iter = range.first; iter!=range.second; iter++)
    {
        std::cout << "the POI: " << iter->first << " is highlighted " << std::endl;
        //convert this POI coordinate from LatLon
            double POI_location_x = lon_to_x(iter->second.lon());
            double POI_location_y = lat_to_y(iter->second.lat());
                   
            std::cout << "its coordinate is ( " << POI_location_x << ", " << POI_location_y << " )" << std::endl;
            
            highlight_POI = true;
            
            POI_highlights.push_back(std::make_pair(POI_location_x, POI_location_y));
    }
    
    // Re-draw the screen (a few squares are changing colour with time)
    drawscreen_ptr();
}

//inter1 -> inter2 button, from intersection to intersection
void act_on_findPath_button(void (*drawscreen_ptr)(void))
{
    std::cout << std::string(100, '\n' );
    
    //to clear the previous highlights
    //highlight_cleanup();
    
    intersection_highlights.clear();
    intersection_highlights_1.clear();
    intersection_highlights_2.clear();
    POI_highlights.clear();
    
    click_highlight = false;
    second_click_highlight = false;
    find_highlight = false;
    highlight_POI = false;
    find_path_input_1 = false;
    find_path_input_2 = false;
    findPOIPath_input_1 = false;
    findPOIPath_input_2 = false;
    closest_intersecion_highlighting = false;
    path_visualization = false;

    //we want to find the most optimal path between those two intersections, local variable
    //IntersectionIndex start_intersection;
    //IntersectionIndex end_intersection;
    
    update_message("Please input the two streets name in terminal...");
    
    std::string street_name1;
    std::string street_name2;
    std::string street_name3;
    std::string street_name4;
    
    //for the 1st input
    std::cout << std::endl;
    std::cout << "please enter 1st street name" << std::endl;
    street_name1 = input_using_readline(1);
    
    std::cout << "please enter 2nd street name" << std::endl;
    street_name2 = input_using_readline(1);
    
    //can find the intersection of two streets by the function we implement in m1
    std::vector<IntersectionIndex> intersection_ids_1 = find_intersection_ids_from_street_names(street_name1, street_name2);
    
    if(intersection_ids_1.size()==0)
    {
        std::cout << "there are no intersections between those two streets" << std::endl;
    }
    if(intersection_ids_1.size()!=0)
    {
        std::cout << "There are total " << intersection_ids_1.size() << " intersections between those two streets" << std::endl;
        for(unsigned i=0; i<intersection_ids_1.size();i++)
        {
            std::cout << "one of the intersections is " << intersection_ids_1[i] << std::endl;
            
            //return the LatLon from the coordinate(x,y) of intersection.
            LatLon intersection_position_1 = getIntersectionPosition(intersection_ids_1[i]);
            
            //return the real world coordinate (x,y) from intersection position
            double intersection_x = lon_to_x(intersection_position_1.lon());
            double intersection_y = lat_to_y(intersection_position_1.lat());
            
            //to debug if the intersection coordinates are output correctly
            std::cout << "the coordinates is (" << intersection_x << ", " << intersection_y << ")" << std::endl;
            
            find_path_input_1 = true;
            //find_path_input_2 = false;
            intersection_highlights_1.push_back(std::make_pair(intersection_x, intersection_y));
            
        }
    }
    drawscreen_ptr();
    
    //for the 2nd input
    std::cout << std::endl;
    std::cout << "for the 2nd intersection you want to search" << std::endl;
    std::cout << "please enter 1st street name" << std::endl;
    street_name3 = input_using_readline(1);
    
    std::cout << "please enter 2nd street name" << std::endl;
    street_name4 = input_using_readline(1);
    
    //can find the intersection of two streets by the function we implement in m1
    std::vector<IntersectionIndex> intersection_ids_2 = find_intersection_ids_from_street_names(street_name3, street_name4);
    //std::cout << "there are total " << intersection_ids_2.size() << " intersection(s) along those two streets" << std::endl;
    
    if(intersection_ids_2.size()==0)
    {
        std::cout << "there are no intersections between those two streets" << std::endl;
    }
    if(intersection_ids_2.size()!=0)
    {
        std::cout << "There are total " << intersection_ids_2.size() << " intersections between those two streets" << std::endl;
        for(unsigned i=0; i<intersection_ids_2.size();i++)
        {
            std::cout << "one of the intersections is " << intersection_ids_2[i] << std::endl;
            
            //return the LatLon from the coordinate(x,y) of intersection.
            LatLon intersection_position_2 = getIntersectionPosition(intersection_ids_2[i]);
            
            //return the real world coordinate (x,y) from intersection position
            double intersection_x = lon_to_x(intersection_position_2.lon());
            double intersection_y = lat_to_y(intersection_position_2.lat());
            
            //to debug if the intersection coordinates are output correctly
            std::cout << "the coordinates is (" << intersection_x << ", " << intersection_y << ")" << std::endl;
            
            //find_path_input_1 = true;
            find_path_input_2 = true;
            intersection_highlights_2.push_back(std::make_pair(intersection_x, intersection_y));           
        }
    }
    
    path_highlight = find_path_between_intersections(intersection_ids_1[0],intersection_ids_2[0],15);
    
    for(auto iter1 = intersection_ids_1.begin(); iter1!=intersection_ids_1.end(); iter1++)
    {
        for(auto iter2 = intersection_ids_2.begin(); iter2!=intersection_ids_2.end(); iter2++)
        {
            path_highlight = find_path_between_intersections(*iter1,*iter2,15);
        }
    }
    
    path_visualization = true;
    
    drawscreen_ptr();
}

//inter -> POI button, from intersection to POI
void act_on_findPOIPath_button(void (*drawscreen_ptr)(void))
{
    std::cout << std::string(100, '\n' );
    
    while (true)
    {
        //to clear existing highlights
        highlight_cleanup();

        //update message
        update_message("Please input the two streets name in terminal...");
        usleep(1000000);
        update_message("");

        //starting intersection street names
        std::string street_name1;
        std::string street_name2;

        //destination POI name
        std::string POI_name;

        std::cout << "Please enter the street names" << std::endl;
        street_name1 = input_using_readline(1);
        street_name2 = input_using_readline(1);

        std::vector<IntersectionIndex> candidate_intersections = find_intersection_ids_from_street_names(street_name1, street_name2);

        if (candidate_intersections.empty())
        {
            std::cout << "No matching intersection found." << std::endl;
            break;
        }

        int multi_match_index = 0;

        std::cout << "Enter your POI name " << std::endl;
        getline(std::cin, POI_name);

        auto POI_find_range = POI_info.equal_range(POI_name);

        if(POI_find_range.first == POI_find_range.second)
        {
            std::cout << "No match found." << std::endl;
            break;
        }

        //display path
        path_highlight = find_path_to_point_of_interest(candidate_intersections[multi_match_index], POI_name, 15);
        path_visualization = true;


        //display starting intersection
        findPOIPath_input_1 = true;
        intersection_highlights_findPOI.push_back(std::make_pair(lon_to_x(getIntersectionPosition(candidate_intersections[multi_match_index]).lon()), lat_to_y(getIntersectionPosition(candidate_intersections[multi_match_index]).lat())));

        //display ending POI
        findPOIPath_input_2 = true;

        for (auto it = POI_find_range.first; it != POI_find_range.second; it++)
                POI_highlights_POIPath.push_back(std::make_pair(lon_to_x((it->second).lon()), lat_to_y((it->second).lat())));
		
        //turn-by-turn instructions
        std::vector<std::string> directions = make_directions(path_highlight, candidate_intersections[multi_match_index]);
        for (auto iter = directions.begin(); iter != directions.end(); iter++)
            std::cout << *iter << std::endl;
        
        break;
    }
    
    drawscreen_ptr();
}

void act_on_load_map_button(void (*drawscreen_ptr) (void))
{
    
    std::cout << std::string(100, '\n' );
    
    update_message("Please enter the path of the map file:");
    std::cout << "Please enter the path of the map file..." << std::endl;
    std::string path;
    
    getline(std::cin,path); //this one does not require auto-completion
       
    //close current map
    close_map();
    
    //clean up graphic data structures
    //free(global_user_input);//free char* that stores the user input
    //global_user_input = NULL; // make it point to null to show that we have freed it
    feature_cleanup();
    POI_cleanup();
    street_cleanup();
    intersection_cleanup();
    POI_find_cleanup();
    highlight_cleanup();

    //attempt to load map
    bool load_success = load_map(path);
    if(!load_success) 
    {
        std::cerr << "Failed to load map" << std::endl;
        
        //reload current map
        load_map(current_path);
        coord_setup();
        intersection_setup();
        street_setup();
        feature_setup();
        POI_find_setup();
        
        clearscreen();
        
        drawscreen_ptr();
        return;
    }
    
    //update current path
    current_path = path;
    
    //restart graphics
    clearscreen();
    
    //setup data structures
    coord_setup();
    intersection_setup();
    street_setup();
    feature_setup();
    POI_find_setup();

    //redraw screen
    drawscreen_ptr();  
}

void act_on_colour_scheme(void (*drawscreen_ptr) (void))
{
    if (active_scheme == 0)
    {
        active_scheme++;
        global_colour_scheme = scheme_night;
        update_message("Colour Scheme: Night");
        usleep(1000000);
        update_message("");
    }
    else if (active_scheme == 1)
    {
        active_scheme++; 
        global_colour_scheme = scheme_deuteranopia;
        update_message("Colour Scheme: Deuteranopia");
        usleep(1000000);
        update_message("");
    }
    
    else if (active_scheme == 2)
    {
        active_scheme++; 
        global_colour_scheme = scheme_protanopia;
        update_message("Colour Scheme: Protanopia");
        usleep(1000000);
        update_message("");
    }
    
    else if (active_scheme == 3)
    {
        active_scheme++; 
        global_colour_scheme = scheme_tritanopia;
        update_message("Colour Scheme: Tritanopia");
        usleep(1000000);
        update_message("");
    }
    
    else
    {
        active_scheme = 0; 
        global_colour_scheme = scheme_day;
        update_message("Colour Scheme: Default");
        usleep(1000000);
        update_message("");
    }
    
    drawscreen_ptr();
}

void path_showing_setup(){
    colour_for_find_path = showing_path (
            std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(153, 0, 0, 255),
            std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 153, 0, 255),
            std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 0, 153, 255),
            std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(255, 255, 0, 255),
            std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 255, 255, 255)
            );
}

void colour_scheme_setup()
{   
    scheme_day = colour_scheme(         std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(235, 235, 235, 255),//_background
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 0, 0, 255),//_text_streetname
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(158, 53, 26, 255),//_text_POIname
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(255, 255, 255, 255),//_street_major
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(255, 195, 90, 255),//_street_highway
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(255, 255, 255, 255),//_street_minor
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(163, 204, 255, 255),//_feature_water
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(203, 230, 163, 255),//_feature_green
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(218, 207, 190, 255),//_feature_building
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(100, 100, 100, 100),//_feature_generic
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(226, 248, 255, 255),//_feature_shore
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(250, 242, 199, 255),//_feature_sand
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(255, 255, 255, 255),//_feature_intersection
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(245, 127, 23, 255),//_POI
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(226, 130, 35, 255),//oneway_arrow
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(222, 45, 38, 255) //_highlight
                                        );
    
    scheme_night = colour_scheme(       std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(35, 47, 61, 255),//_background
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(245, 245, 245, 255),//_text_streetname
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(106, 157, 153, 255),//_text_POIname
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(56, 68, 80, 255),//_street_major
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(196, 130, 54, 255),//_street_highway
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(56, 68, 80, 255),//_street_minor
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(23, 38, 112, 255),//_feature_water
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(33, 69, 66, 255),//_feature_green
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(66, 78, 90, 255),//_feature_building
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(100, 100, 100, 100),//_feature_generic
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(226, 248, 255, 255),//_feature_shore
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(154, 130, 104, 255),//_feature_sand
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(56, 65, 78, 255),//_feature_intersection
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(44, 211, 208, 255),//_POI
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(226, 130, 35, 255),//oneway_arrow
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(222, 45, 38, 255) //_highlight
                                        );
    
    scheme_deuteranopia = colour_scheme(std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(218, 218, 218, 255),//_background
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 0, 0, 255),//_text_streetname
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(130, 20, 160, 255),//_text_POIname
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(255, 255, 255, 255),//_street_major
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(240, 240, 50, 255),//_street_highway
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(255, 255, 255, 255),//_street_minor
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(20, 210, 220, 255),//_feature_water
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(10, 180, 90, 255),//_feature_green
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(155, 155, 155, 255),//_feature_building
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(155, 155, 155, 100),//_feature_generic
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 160, 250, 255),//_feature_shore
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(250, 230, 190, 255),//_feature_sand
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(56, 65, 78, 255),//_feature_intersection
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(130, 20, 160, 255),//_POI
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(170, 10, 60, 255),//oneway_arrow
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 0, 0, 255) //_highlight
                                        );
    
    scheme_protanopia = colour_scheme(  std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(218, 218, 218, 255),//_background
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 0, 0, 255),//_text_streetname
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(130, 20, 160, 255),//_text_POIname
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(255, 255, 255, 255),//_street_major
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(160, 250, 130, 255),//_street_highway
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(255, 255, 255, 255),//_street_minor
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 160, 250, 255),//_feature_water
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(10, 180, 90, 255),//_feature_green
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 110, 130, 255),//_feature_building
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 110, 130, 255),//_feature_generic
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 160, 250, 255),//_feature_shore
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(260, 250, 130, 255),//_feature_sand
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(56, 65, 78, 255),//_feature_intersection
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(130, 20, 160, 255),//_POI
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(170, 10, 60, 255),//oneway_arrow
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 0, 0, 255) //_highlight
                                        );
    
    scheme_tritanopia = colour_scheme(  std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(218, 218, 218, 255),//_background
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 0, 0, 255),//_text_streetname
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(130, 20, 160, 255),//_text_POIname
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(255, 255, 255, 255),//_street_major
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(130, 20, 160, 255),//_street_highway
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(255, 255, 255, 255),//_street_minor
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(160, 250, 130, 255),//_feature_water
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 110, 130, 255),//_feature_green
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(155, 155, 155, 255),//_feature_building
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(155, 155, 155, 255),//_feature_generic
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 160, 250, 255),//_feature_shore
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(250, 230, 190, 255),//_feature_sand
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(56, 65, 78, 255),//_feature_intersection
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(130, 20, 160, 255),//_POI
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(250, 120, 80, 255),//oneway_arrow
                                        std::make_tuple<uint8_t, uint8_t, uint8_t, uint8_t>(0, 0, 0, 255) //_highlight
                                        );
    
    global_colour_scheme = scheme_day;
    
}

void coord_setup()
{
    //initializes boundary points for visible world
    min_x = std::numeric_limits<double>::max();
    min_y = std::numeric_limits<double>::max();
    max_x = std::numeric_limits<double>::lowest();
    max_y = std::numeric_limits<double>::lowest();
    
    //initializes OSM element count variables
    intersection_count = getNumberOfIntersections();
    POI_count = getNumberOfPointsOfInterest();
    feature_count = getNumberOfFeatures();
    street_segment_count = getNumberOfStreetSegments();
    street_count = getNumberOfStreets();
    
    //finds the actual value of boundary points
    for (int i=0; i<intersection_count; i++)
    {
        min_x = std::min(getIntersectionPosition(i).lon(), min_x);
        min_y = std::min(getIntersectionPosition(i).lat(), min_y);
        max_x = std::max(getIntersectionPosition(i).lon(), max_x);
        max_y = std::max(getIntersectionPosition(i).lat(), max_y);
    }
    
    //finds lat_avg for use in coordinate conversion helper functions
    lat_avg = cos((min_y + max_y) * DEG_TO_RAD / 2);
    
    //calculates zoom threshold
    double width = find_distance_between_two_points(LatLon(max_y, max_x), LatLon(max_y, min_x));
    double length = find_distance_between_two_points(LatLon(max_y, max_x), LatLon(min_y, max_x));
    double total_map_area = width * length;
    double total_world_area = INITIAL_COORDS.get_width() * INITIAL_COORDS.get_height();
    
    POI_vis_world_area_threshold = total_world_area / total_map_area * (POI_VIS_REAL_DIM_THRESHOLD * POI_VIS_REAL_DIM_THRESHOLD);
    
    open_feature_vis_world_area_threshold = total_world_area / total_map_area * (OPEN_FEATURE_VIS_REAL_DIM_THRESHOLD * OPEN_FEATURE_VIS_REAL_DIM_THRESHOLD);
    street_vis_world_area_threshold = total_world_area / total_map_area * (STREET_VIS_REAL_DIM_THRESHOLD * STREET_VIS_REAL_DIM_THRESHOLD);
    street_arrow_vis_world_area_threshold = total_world_area / total_map_area * (STREET_ARROW_VIS_REAL_DIM_THRESHOLD * STREET_ARROW_VIS_REAL_DIM_THRESHOLD);
    minor_street_vis_world_area_threshold = total_world_area / total_map_area * (MINOR_STREET_VIS_REAL_DIM_THRESHOLD *  MINOR_STREET_VIS_REAL_DIM_THRESHOLD);
    //debugging only
    //std::cout << "when visible world area is greater than " << POI_vis_world_area_threshold << ", POI icons are not displayed." << std::endl;
    
    //converts boundary points from WGS-84 to 2D Cartesian
    min_x = min_x * lat_avg * DEG_TO_RAD;
    min_y = min_y * DEG_TO_RAD;
    max_x = max_x * lat_avg * DEG_TO_RAD;
    max_y = max_y * DEG_TO_RAD;
    
    //debugging only
    //std::cout << min_x << "," << min_y << std::endl; 
    
    //scaling factor scales using the longer side
    coordinate_scaling_factor = INITIAL_COORDS.get_width() / std::max((max_y - min_y), (max_x - min_x));
    
}

void feature_setup()
{   
    int prec_index = 0;
    
    for (int i=0; i<feature_count; i++)
    {
        feature_data temp;
        
        switch(getFeatureType(i))
        {
            case Unknown:
                temp.feature_colour = &global_colour_scheme.feature_generic;
                break;
            case Park:
            case Greenspace:
            case Golfcourse:
                temp.feature_colour = &global_colour_scheme.feature_green;
                break;
            case Lake:
            case River:
            case Stream:
                temp.feature_colour = &global_colour_scheme.feature_water;
                break;
            case Beach:
            case Island:
                temp.feature_colour = &global_colour_scheme.feature_sand;
                break;
            case Shoreline:
                temp.feature_colour = &global_colour_scheme.feature_shore;
                break;
            case Building:    
                temp.feature_colour = &global_colour_scheme.feature_building;
                break;
            default:
                temp.feature_colour = &global_colour_scheme.feature_generic;
                break;
        }          
        
        int count = getFeaturePointCount(i);
        temp.pt_count = count;
        
        temp.point = new t_point[count];
        for (int j=0; j<count; j++)
        {   
            float x = lon_to_x(getFeaturePoint(i,j).lon());
            float y = lat_to_y(getFeaturePoint(i,j).lat());   
            temp.point[j] = t_point(x,y);
        }
                
        temp.open = !((getFeaturePoint(i,0).lat() == getFeaturePoint(i,count-1).lat()) &&
                    ((getFeaturePoint(i,0).lon() == getFeaturePoint(i,count-1).lon()))    
                    );
        
        if (temp.open)
            open_features.push_back(temp);
        else
        {
            double feature_area = find_feature_area(temp.point, temp.pt_count);
            closed_features.push_back(temp);
            closed_feature_precedence.push_back(std::make_pair(prec_index, feature_area));
            prec_index++;
        }
    }
    
    //sorts precedence vector
    std::sort(closed_feature_precedence.begin(), closed_feature_precedence.end(), [](const std::pair<int,double> &left, const std::pair<int,double> &right) {
    return left.second > right.second;
    });
    
}

void intersection_setup()
{
    intersections.resize (intersection_count);
    
    for (int i = 0; i < intersection_count; i ++)
    {
        //std::cout << getIntersectionPosition(i).lat() << "," << getIntersectionPosition(i).lon() << std::endl;
        intersections[i].name = getIntersectionName(i);
        intersections[i].position = getIntersectionPosition(i);
        int_cartesian_coor.push_back(std::make_pair(lon_to_x(intersections[i].position.lon()), lat_to_y(intersections[i].position.lat())));
    }
}

void street_setup()
{
    //loading data
    streets.resize(street_count);
       
    //store all street information into streets vector
    for (unsigned int i = 0; i<getNumberOfStreets(); i++){
    streets[i].street_name = getStreetName(i); 
    streets[i].segments_id = find_street_street_segments(i);
        for (int j = 0;  j < streets[i].segments_id.size(); j++){
            unsigned cp_count = getStreetSegmentInfo(streets[i].segments_id[j]).curvePointCount;
            (streets[i].curve_point_count).push_back(cp_count);
        }
    }
}

inline void feature_cleanup()
{
    open_features.clear();
    closed_features.clear();
    closed_feature_precedence.clear();
}

inline void street_cleanup()
{
    streets.clear();
}

inline void intersection_cleanup()
{
    intersections.clear();
    int_cartesian_coor.clear();
}

inline void POI_cleanup()
{
}

double lon_to_x(double lon)
{
    return ((lon * lat_avg * DEG_TO_RAD) - min_x) * coordinate_scaling_factor;
}

double lat_to_y(double lat)
{
    return ((lat * DEG_TO_RAD) - min_y) * coordinate_scaling_factor;
}

double x_to_lon(double x)
{
    return ((x / coordinate_scaling_factor) + min_x) / (lat_avg * DEG_TO_RAD);
}

double y_to_lat(double y)
{
    return ((y / coordinate_scaling_factor) + min_y) / DEG_TO_RAD;
}

double find_feature_area(t_point pt[], int n)
{
    n=n-1;//this algorithm does not use duplicate first and last points
    
    double* x_array = new double[n];
    double* y_array = new double[n];
    
    for (int i=0; i<n; i++)
    {
        x_array[i] = pt[i].x;
        y_array[i] = pt[i].y;
    }
    
    
    double area = 0.0;
    
    int j = n-1;
    
    for (int i=0; i<n; i++)
    {
        area += (x_array[j] + x_array[i]) * (y_array[j] - y_array[i]);
        j = i;  // j is previous vertex to i
    }
    
    delete[] x_array;
    delete[] y_array;
    
    return fabs(area / 2.0);
}
    
void highlight_cleanup()
{
    click_highlight = false;
    second_click_highlight = false;
    find_highlight = false;
    highlight_POI = false;
    find_path_input_1 = false;
    find_path_input_2 = false;
    findPOIPath_input_1 = false;
    findPOIPath_input_2 = false;
    closest_intersecion_highlighting = false;
    path_visualization = false;
    
    path_highlight.clear();
    POI_path_highlight.clear();
    //POI_info.clear();
    POI_highlights.clear();
    POI_highlights_POIPath.clear();
    closest_intersection_from_POI.clear();
    intersection_highlights.clear();
    intersection_highlights_1.clear();
    intersection_highlights_2.clear();
    intersection_highlights_findPOI.clear();
}

LatLon get_closest_intersection_position(LatLon POI_position)
{
    //find the closest intersection of that POI by the function in m1 we implement
    unsigned intersection_id = find_closest_intersection(POI_position);
    
    //after we have the closest intersection id, we can get the LatLon of that id
    LatLon intersecion_position = getIntersectionPosition(intersection_id);
    
    return intersecion_position;
}

