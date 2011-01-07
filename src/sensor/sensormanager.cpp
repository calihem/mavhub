#include "sensormanager.h"
#include "core/logger.h"
#include <sstream> //stringstream

using namespace std;

namespace mavhub {

SensorManager::SensorManager() {
	}

SensorManager::~SensorManager() {}

SensorManager& SensorManager::instance() {
	// instantiated on first use and will be guaranteed destroyed
	static SensorManager instance;

	return instance;
}

void SensorManager::add_sensor(Sensor* S, unsigned short id) {
	sensor_map.insert(pair<unsigned int,Sensor*>(id,S));
}

void SensorManager::start_all_sensors() {
	stringstream send_stream;
	send_stream << "SensorManager: starting " << sensor_map.size() << " sensors";
	Logger::debug(send_stream.str());

	for (map<unsigned int,Sensor*>::iterator iter = sensor_map.begin(); iter != sensor_map.end(); ++iter) {
		iter->second->start();
	}
}

void SensorManager::restart_sensor(unsigned int id) throw(const char *) {
	try {
		stop_sensor(id);
		start_sensor(id);
	}
	catch (const char *message) {
		throw message;
	}
}

void SensorManager::stop_sensor(unsigned int id) throw(const char *) {
	std::map<unsigned int, Sensor*>::iterator iter;
	if ((iter = sensor_map.find((0x0000FFFF & id))) != sensor_map.end()) {
		if (iter->second->get_status() == Sensor::RUNNING) iter->second->stop();
		else throw "stop_sensor(): sensor isn't running";
	} else throw "stop_sensor(): no such sensor";
}

void SensorManager::start_sensor(unsigned int id) throw(const char *) {
	std::map<unsigned int, Sensor*>::iterator iter;
	if ((iter = sensor_map.find((0x0000FFFF & id))) != sensor_map.end()) {
		if (iter->second->get_status() == Sensor::STOPPED) iter->second->start();
		else throw "start_sensor(): sensor isn't stopped";
	} else throw "start_sensor(): no such sensor";
}

void SensorManager::remove_sensor(unsigned int id) throw(const char *) {
	std::map<unsigned int, Sensor*>::iterator iter;
	if ((iter = sensor_map.find((0x0000FFFF & id))) != sensor_map.end()) {
		sensor_map.erase(iter);
		delete(iter->second);
	} else throw "remove_sensor(): no such sensor";
}

} // namespace mavhub
