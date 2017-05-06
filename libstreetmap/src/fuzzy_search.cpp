#include <iostream>
#include <cstdlib>
#include "fuzzy_search.h"

using namespace std;

vector<const char*> POI_name_vector;
vector<const char*> street_name_vector;

void POI_name_vector_setup(){
    int POI_count = getNumberOfPointsOfInterest();
    for (int i=0; i<POI_count; i++)
    {
        const char* POI_name = getPointOfInterestName(i).c_str(); //convert string to const char*
        POI_name_vector.push_back(POI_name);
    }
}

//clean up the vector. Used in close_map function in M1
void POI_name_vector_cleanup(){
    POI_name_vector.clear();
}

void street_name_vector_setup(){
    int street_count = getNumberOfStreets();
    for (int i=0; i<street_count; i++)
    {
        const char* street_name = getStreetName(i).c_str(); //convert string to const char*
        street_name_vector.push_back(street_name);
    }
}

//clean up the vector. Used in close_map function in M1
void street_name_vector_cleanup(){
    street_name_vector.clear();
}

std::string input_using_readline (int poi_or_street_name){
    //Use tab for auto completion
    rl_bind_key('\t', rl_complete);
    
    //Use our function for auto-complete
    if (poi_or_street_name == 1){
        rl_attempted_completion_function = command_completion_ver1; 
    }
    else{
        rl_attempted_completion_function = command_completion_ver2; 
    }
    
    //Tell readline to handle double and single quotes for us
    rl_completer_quote_characters = strdup("\"\'"); 
    
    char* buf;
    string copy_of_buf;
    
    //just getting single input
    buf = readline("");

    if(strcmp(buf, "") != 0) { //Only save non-empty commands
            add_history(buf);
        }
    
    copy_of_buf = string(buf);//this is going to be the return value
    
    free(buf);
    buf = NULL; //Mark it null to show we freed it
    
    return copy_of_buf;
}

char** command_completion_ver2(const char* stem_text, int start, int end) {
    char ** matches = NULL;

    if (stem_text != NULL){
        matches = rl_completion_matches(stem_text, POI_name_generator);
        if (matches == NULL){
            cout<<endl<<"Sorry, there is no POI name that matches your input"<<endl;
        }
  
    }
    return matches;
}

char** command_completion_ver1(const char* stem_text, int start, int end) {
    char ** matches = NULL;
    
    if (stem_text != NULL){
        matches = rl_completion_matches(stem_text, street_name_generator);
        if (matches == NULL){
            cout<<endl<<"Sorry, there is no street name that matches your input"<<endl;
        }
    }
    return matches;
}

//This is for POI name auto-completion
char* POI_name_generator(const char* stem_text, int state) {
    //Static here means a variable's value persists across function invocations
    static int count;

    if(state == 0) { 
        count = -1;
    }

    int text_len = strlen(stem_text);

    //Search through POI_name_vector until we find a match
    while(count < (int) POI_name_vector.size()-1) {
        count++;
        if(strncmp(POI_name_vector[count], stem_text, text_len) == 0) {
            //Must return a duplicate, Readline will handle
            //freeing this string itself.
            return strdup(POI_name_vector[count]);
        }
    }

    //No more matches
    return NULL;
}

char* street_name_generator(const char* stem_text, int state) {
    //Static here means a variable's value persists across function invocations
    static int count;

    if(state == 0) { 
        count = -1;
    }

    int text_len = strlen(stem_text);

    //Search through POI_name_vector until we find a match
    while(count < (int) street_name_vector.size()-1) {
        count++;
        if(strncmp(street_name_vector[count], stem_text, text_len) == 0) {
            //Must return a duplicate, Readline will handle
            //freeing this string itself.
            return strdup(street_name_vector[count]);
        }
    }

    //No more matches
    return NULL;
}

