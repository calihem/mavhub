#ifndef _MAIN_H_
#define _MAIN_H_

#include <inttypes.h> //uint8_t
#include <string>
#include <list>
#include "factory.h"
#include "lib/setting.h"

void read_settings(cpp_io::Setting &settings);
void add_links(const std::list<std::string> link_list, cpp_io::Setting &settings);
void read_link_configuration(mavhub::LinkFactory::link_construction_plan_t &link_plan, cpp_io::Setting &settings);
void add_apps(const std::list<std::string> &app_list, cpp_io::Setting &settings);
void parse_argv(int argc, char **argv);
void print_help();

#endif
