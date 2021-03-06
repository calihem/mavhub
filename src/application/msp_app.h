#ifndef _MSP_APP_H_
#define _MSP_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#ifdef HAVE_MSPLINK_H
#include <msplink.h>

#include <stdlib.h>

#include "protocol/protocollayer.h"
#include "../module/PID.h"

#include <inttypes.h> //uint8_t
#include <bitset>

namespace mavhub {

  class MSPApp : public AppLayer<mavlink_message_t>, public AppLayer<msp_message_t> {
  public:
    static const int component_id = 25;

    MSPApp(const std::map<std::string, std::string>,
           const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
    virtual ~MSPApp();

    virtual void handle_input(const mavlink_message_t &msg);
    virtual void handle_input(const msp_message_t &msg);

  protected:
    virtual void print(std::ostream &os) const;
    virtual void run();

  private:
    static const int parameter_count = 104;
    static const int8_t parameter_ids[parameter_count][15];

    /// tx buffer for mavlink messages
    // 			mavlink_message_t tx_mav_msg;
    /// Mutex to protect tx_mav_msg
    // 			pthread_mutex_t tx_mav_mutex;
    uint8_t parameters[parameter_count];
    /// Serial port to MSP
    // 			UART msp_dev;
    /// tx buffer for MSP messages
    // 			msp_message_t tx_msp_msg;
    /// Mutex to protect tx_msp_msg
    // 			pthread_mutex_t tx_msp_mutex;
    /// Time of last received msp message
    // 			uint64_t message_time;
    /// Last received attitude
    // 			msp_attitude_t attitude;
    /// Time of last received attitude
    // 			uint64_t attitude_time;
    /// Last requested parameter from MSP
    // 			uint8_t parameter_request;
    /// Time of last parameter answer from MSP
    // 			uint64_t parameter_time;
    /// pre-packed heartbeat message
    mavlink_message_t heartbeat_msg;
    /// msp2mavlink message
    mavlink_message_t mavmsg;
    /// app params
    int run_cnt;
    float u_i, v_i;
    float u, v;
    float gyw_x, gyw_y;
    float Kc_x, Ti_x, Td_x;
    float Kc_y, Ti_y, Td_y;
    float squal;
    float gx, gy, gz;

    // rc control
    int aux2, aux2tm1; /// controls extctrl
    // params
    /// request
    int param_request_list;
    /// param container
    std::map<std::string, double> params;
    
    PID* pid_pitch;
    PID* pid_roll;
    PID* pid_alt;
    // 			using AppLayer::send;
    // 			void send_heartbeat();
    // 			void send_mavlink_param_value(const msp_param_type_t param_type);
    // 			const uint8_t get_parameter(const msp_param_type_t param_type) const;
    // 			const int8_t* get_parameter_id(const msp_param_type_t param_type) const;
    // 			const int parameter_id_to_index(const int8_t *parameter_id);
    /// Send msp message over MSP device
    // 			size_t send(const msp_message_t& msg);
    /// Send msp data over MSP device
    // 			size_t send(const msp_msg_type_t type, const void *data, const uint8_t size);
    /// Method to handle input from MSP
    // 			void handle_input(const msp_message_t& msg);
    virtual void read_conf(const std::map<std::string, std::string> args);
    virtual void send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value);
  };
  // ----------------------------------------------------------------------------
  // MSPApp
  // ----------------------------------------------------------------------------
  // 	inline const uint8_t MSPApp::get_parameter(const msp_param_type_t param_type) const {
  // 		return parameters[param_type];
  // 	}
  // 	inline const int8_t* MSPApp::get_parameter_id(const msp_param_type_t param_type) const {
  // 		return parameter_ids[param_type];
  // 	}
} // namespace mavhub

#endif // HAVE_MSPLINK_H
#endif // HAVE_MAVLINK_H
#endif // _MSP_APP_H_
