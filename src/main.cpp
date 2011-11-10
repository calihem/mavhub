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
 * \file main.cpp
 * \date created at 2010/07/26
 *
 * \sa main.h
 */

#include "main.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#include "core/core.h"
#include "core/logger.h"
#include "protocol/protocolstack.h"
#include "protocol/protocollayer.h"
#include "core/mavshell.h"
#include "application/app_store.h"
#include "sensor/sensor_factory.h"
#ifdef HAVE_GSTREAMER
#include "lib/gstreamer/video_server.h"
#endif // HAVE_GSTREAMER

#include <iostream>     // cout
#include <cstring>      // strcmp
#include <cstdlib>      // exit

using namespace std;
using namespace mavhub;
using namespace cpp_pthread;
using namespace cpp_io;

int tcp_port(32000);	///< \var TCP port of configuration shell
string cfg_filename("mavhub.d/mavhub.conf"); ///< \var name of configuration file

/**
 * \brief Entry point of mavhub.
 * \param argc The number of arguments given by command line.
 * \param argv The arguments given as a vector of strings.
 */
int main(int argc, char **argv) {
	Core::argc = &argc;
	Core::argv = argv;

	Logger::setLogLevel(Logger::LOGLEVEL_WARN);

	parse_argv(argc, argv);

	// open config file and read settings
	try {
		Setting settings(cfg_filename, std::ios_base::in);
		read_settings(settings);
	}
	catch(const invalid_argument &ia) {
		Logger::log(ia.what(), Logger::LOGLEVEL_WARN);
	}

	// start sensors
	SensorManager::instance().start_all_sensors();

	// start configuration shell
	try {
		MAVShell *mav_shell = new MAVShell(tcp_port);
		mav_shell->start();
		mav_shell->join();
	}
	catch(const std::exception& e) {
		cout << e.what() << endl;
	}

	// join stack thread
#ifdef HAVE_MAVLINK_H
	ProtocolStack<mavlink_message_t>::instance().join();
#endif // HAVE_MAVLINK_H
}

void read_settings(Setting &settings) {
	//read global loglevel
	Logger::log_level_t loglevel;
	if( settings.value("loglevel", loglevel) )
		Logger::log("Loglevel is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
	else
		Logger::setLogLevel(loglevel);

	//read loglevel of stack
	if(settings.begin_group("stack") == 0) {  //stack group available
		if( settings.value("loglevel", loglevel) )
			Logger::log("Loglevel for stack is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
		else {
#ifdef HAVE_MAVLINK_H
				ProtocolStack<mavlink_message_t>::instance().loglevel(loglevel);
#endif // HAVE_MAVLINK_H
#ifdef HAVE_MKHUCHLINK_H
				ProtocolStack<mkhuch_message_t>::instance().loglevel(loglevel);
#endif // HAVE_MKHUCHLINK_H
#ifdef HAVE_MKLINK_H
				ProtocolStack<mk_message_t>::instance().loglevel(loglevel);
#endif // HAVE_MKLINK_H
		}
		settings.end_group();
	}

	//read system ID
	uint16_t system_id(42);
	if( settings.value("system_id", system_id) )
		Logger::log("System ID is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
	Core::system_id(system_id);

	//read TCP port of MAVShell
	if( settings.value("tcp_port", tcp_port) )
		Logger::log("TCP port is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);

	//read interfaces (links)
	list<string> link_list;
	if( settings.value("interfaces", link_list) ) {
		Logger::log("List of links is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
	} else {
		add_links(link_list, settings);
	}

#ifdef HAVE_GSTREAMER
	//read video server
	if(settings.begin_group("video_server") == 0) {  //video_server group available
		list<string> pipeline_list;
		if( settings.value("pipeline_description", pipeline_list) ) {
			Logger::log("Pipeline description for video server missing in config file", Logger::LOGLEVEL_WARN);
		} else {
			string pipeline_description;
			//convert string list to string
			for(list<string>::iterator iter = pipeline_list.begin(); iter != pipeline_list.end(); ++iter) {
				pipeline_description.append(*iter);
				pipeline_description.append(" ");
			}
			if(!Core::video_server)
				Core::video_server = new hub::gstreamer::VideoServer(Core::argc, Core::argv, pipeline_description);
			if(Core::video_server)
				Core::video_server->start();
			else
				Logger::log("Can't create video server", Logger::LOGLEVEL_WARN);
		}
		settings.end_group();
	}
#endif // HAVE_GSTREAMER

	//read apps
	list<string> app_list;
	if( settings.value("applications", app_list) ) {
		Logger::log("List of apps is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
	} else {
		add_apps(app_list, settings);
	}

	//read sensors
	list<string> sensors_list;
	if( settings.value("sensors", sensors_list) ) {
		Logger::log("List of sensors is missing in config file: ", cfg_filename, Logger::LOGLEVEL_WARN);
	} else {
		add_sensors(sensors_list, settings);
	}
}

void add_links(const list<string> link_list, Setting &settings) {
	LinkFactory::link_construction_plan_t link_construction_plan;

	for(list<string>::const_iterator link_iter = link_list.begin(); link_iter != link_list.end(); ++link_iter) {
		try {   //first read from sub config file
			Setting link_settings(settings.path() + string("/") + *link_iter);
			read_link_configuration(link_construction_plan, link_settings);
		}
		catch(const invalid_argument &ia) {
			Logger::log(ia.what(), Logger::LOGLEVEL_DEBUG);
		}

		if(settings.begin_group(*link_iter) == 0) {  //link group available
			//read from global config file
			read_link_configuration(link_construction_plan, settings);

			settings.end_group();
		} else {
			Logger::log(*link_iter, " group missing in config file", Logger::LOGLEVEL_DEBUG);
		}

		cpp_io::IOInterface *layer = LinkFactory::build(link_construction_plan);
		if(!layer) {
			Logger::log("Construction of", *link_iter, "failed", Logger::LOGLEVEL_WARN);
			continue;
		}
		switch(link_construction_plan.protocol_type) {
#ifdef HAVE_MAVLINK_H
			case MAVLINK:
				ProtocolStack<mavlink_message_t>::instance().add_link(layer);
				break;
#endif // HAVE_MAVLINK_H
#ifdef HAVE_MKHUCHLINK_H
			case MKHUCHLINK:
				ProtocolStack<mkhuch_message_t>::instance().add_link(layer);
				break;
#endif // HAVE_MKHUCHLINK_H
#ifdef HAVE_MKLINK_H
			case MKLINK:
				ProtocolStack<mk_message_t>::instance().add_link(layer);
				break;
#endif // HAVE_MKLINK_H

			default:
				Logger::log("Adding of", *link_iter, "failed, due to unkwnown protocol type", Logger::LOGLEVEL_DEBUG);
				break;
		}
	}
}

void read_link_configuration(LinkFactory::link_construction_plan_t &link_plan, Setting &settings) {
	settings.value("type", link_plan.link_type);
	settings.value("name", link_plan.dev_name);
	settings.value("port", link_plan.port);
	settings.value("baudrate", link_plan.baudrate);
	settings.value("protocol", link_plan.protocol_type);
	settings.value("members", link_plan.groupmember_list);
}

void add_apps(const std::list<std::string> &app_list, Setting &settings) {

	map<string, string> arg_map;
	for(list<string>::const_iterator app_iter = app_list.begin(); app_iter != app_list.end(); ++app_iter) {
		//read from global config file first (because map wouldn't overwrite existing entries)
		if(settings.begin_group(*app_iter) == 0) {  //app group available
			settings.values(arg_map);
			settings.end_group();
		}
		try {   //next read from sub config file
			Setting app_settings(settings.path() + string("/") + *app_iter);
			app_settings.values(arg_map);
		}
		catch(const invalid_argument &ia) {
			Logger::log(ia.what(), Logger::LOGLEVEL_DEBUG);
		}
		AppStore::order(*app_iter, arg_map);
		arg_map.clear();
	}
}

void add_sensors(const std::list<std::string> &sensors_list, Setting &settings) {

	map<string, string> arg_map;
	for(list<string>::const_iterator sensors_iter = sensors_list.begin(); sensors_iter != sensors_list.end(); ++sensors_iter) {
		//read from global config file first (because map wouldn't overwrite existing entries)
		if(settings.begin_group(*sensors_iter) == 0) {  //sensor group available
			settings.values(arg_map);
			settings.end_group();
		}
		try {   //next read from sub config file
			Setting sensors_settings(settings.path() + string("/") + *sensors_iter);
			sensors_settings.values(arg_map);
		}
		catch(const invalid_argument &ia) {
			Logger::log(ia.what(), Logger::LOGLEVEL_DEBUG);
		}

		SensorFactory::build(*sensors_iter, arg_map);
		arg_map.clear();
	}
}

void parse_argv(int argc, char **argv) {
	for(int i = 1; i < argc; i++) {
		if( !strcmp(argv[i], "-c") || !strcmp(argv[i], "--config") ) {
			if(++i < argc) {
				cfg_filename = argv[i];
			} else {
				cout << "ERROR: file argument missing" << endl;
				exit(-1);
			}
		} else if( !strcmp(argv[i], "-h") || !strcmp(argv[i], "--help") ) {
			print_help();
			exit(0);
		} else if( !strcmp(argv[i], "-p") || !strcmp(argv[i], "--port") ) {
			if(++i < argc) {
				tcp_port = atoi(argv[i]);
			} else {
				cout << "ERROR: port argument missing" << endl;
				exit(-1);
			}
		} else {
			cout << "ERROR: " << argv[i] << " is no valid argument" << endl;
		}
	}
}

void print_help() {
	cout << "NAME" << endl;
	cout << "\t" << "MAVHUB - Micro Air Vehicle HUB" << endl << endl;
	cout << "SYNOPSIS" << endl;
	cout << "\t" << "mavhub [-h]" << endl << endl;
	cout << "OPTIONS" << endl;
	cout << "\t-c <file> or --config <file>" << endl;
	cout << "\t\t" << "open config file <file>" << endl;
	cout << "\t-h or --help" << endl;
	cout << "\t\t" << "print usage summary" << endl;
	cout << "\t-p <port> or --port <port>" << endl;
	cout << "\t\t" << "set port of management console" << endl;
}
