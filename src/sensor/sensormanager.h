#ifndef _SENSOR_MANAGER_H_
#define _SENSOR_MANAGER_H_

#include <exception>
#include <inttypes.h>
#include <map>
#include <string>

#include "sensor.h"

namespace mavhub {

	class SensorManager {
		public:
			SensorManager();
			virtual ~SensorManager();
			static SensorManager& instance();
			
			void start_all_sensors();
			void add_sensor(Sensor* S, unsigned short id);
			void start_sensor(unsigned int id) throw(const char *);
			void stop_sensor(unsigned int id) throw(const char *);
			void restart_sensor(unsigned int id) throw(const char *);
			void remove_sensor(unsigned int id) throw(const char *);
			template <typename T>
			void get_data(T &data, unsigned int id) throw(const char *);
		protected:
		private:
			std::map<unsigned int, Sensor*> sensor_map;
	};
			
	template <typename T>
	void SensorManager::get_data(T &data, unsigned int id) throw(const char *) {
		void* data_pointer = NULL;

		std::map<unsigned int, Sensor*>::iterator iter;
		if ((iter = sensor_map.find((0x0000FFFF & id))) != sensor_map.end()) {
			try {
				data_pointer = iter->second->get_data_pointer(id);
				if (data_pointer != NULL) {
					hub::Lock ri_lock(iter->second->get_data_mutex());
					data = *(reinterpret_cast<T*>(data_pointer));
				}
			}
			catch (const char *message) {
				std::string s(message);
				throw ("sensor manager: " + s).c_str();
			}
		} else throw "sensor manager: requesting non present sensor";
	}

} // namespace mavhub

#endif
