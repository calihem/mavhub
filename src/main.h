/****************************************************************************
** Copyright 2011 Humboldt-Universitaet zu Berlin
**
** This file is part of MAVHUB.
**
** MAVHUB is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** MAVHUB is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with MAVHUB.  If not, see <http://www.gnu.org/licenses/>.
**
*****************************************************************************/
/**
 * \file main.h
 * \date created at 2010/07/26
 *
 * \sa main.cpp
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include "factory.h"
#include "core/setting.h"

#include <inttypes.h> //uint8_t
#include <string>
#include <list>

/**
 * \brief Reads all settings from configuration file.
 * \param settings The file to read from.
 */
void read_settings(cpp_io::Setting &settings);

/**
 * \brief Reads multiple link settings from configuration file.
 * \param link_list The list of links for which settings should be read.
 * \param settings The file to read from. 
 */
void add_links(const std::list<std::string> link_list, cpp_io::Setting &settings);

/**
 * \brief Reads link settings from configuration file.
 *
 *  Reads the link settings from current group in configuration file to a
 * construction plan for link factory.
 * \param link_plan The construction plan holding all settings
 * \param settings The file to read from.
 * \sa LinkFactory
 */
void read_link_configuration(mavhub::LinkFactory::link_construction_plan_t &link_plan, cpp_io::Setting &settings);

/**
 * \brief Reads multiple application settings from configuration file.
 * \param app_list The list of applications for which settings should be read.
 * \param settings The file to read from. 
 */
void add_apps(const std::list<std::string> &app_list, cpp_io::Setting &settings);

/**
 * \brief Reads multiple sensor settings from configuration file.
 * \param sensors_list The list of sensors for which settings should be read.
 * \param settings The file to read from. 
 */
void add_sensors(const std::list<std::string> &sensors_list, cpp_io::Setting &settings);

/**
 * \brief Parses the arguments given by command line.
 * \param argc The number of arguments
 * \param argv The arguments given as a vector of strings.
 */
void parse_argv(int argc, char **argv);

/**
 * \brief Prints a help message to stdout.
 */
void print_help();

#endif
