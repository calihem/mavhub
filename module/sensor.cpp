#include "sensor.h"

using namespace std;

namespace mavhub {


Sensor::Sensor(): status(UNINITIALIZED) {
	pthread_mutex_init(&data_mutex, NULL);
}

Sensor::~Sensor() {
	stop();
}

void Sensor::stop() {
	status = STOPPED;
	join();
}

pthread_mutex_t &Sensor::get_data_mutex() {
	return data_mutex;
}

Sensor::statusT Sensor::get_status() {
	return status;
}
} // namespace mavhub
