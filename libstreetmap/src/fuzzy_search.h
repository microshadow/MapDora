#pragma once

#include <iostream>
#include <vector>
#include <cstdlib>

#include <readline/readline.h>
#include <readline/history.h>

#include "StreetsDatabaseAPI.h"

using namespace std;

// these are the databases that contain all the names of POIs and street names
extern vector<const char*> POI_name_vector;
extern vector<const char*> street_name_vector;

void POI_name_vector_setup();
void street_name_vector_setup();

void POI_name_vector_cleanup();
void street_name_vector_cleanup();

/*This is very similar to GNU readline example provided */
char** command_completion_ver2(const char* stem_text, int start, int end); //for POI name auto-completion
char** command_completion_ver1(const char* stem_text, int start, int end); //for street name auto-completion
char* POI_name_generator(const char* stem_text, int state);
char* street_name_generator(const char* stem_text, int state);
std::string input_using_readline (int poi_or_street_name);
