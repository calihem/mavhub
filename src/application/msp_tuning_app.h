#ifndef _MSP_TUNING_APP_H_
#define _MSP_TUNING_APP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#ifdef HAVE_MSPLINK_H
#include <msplink.h>

#include <stdlib.h>

#include "protocol/protocollayer.h"
// #include "../module/PID.h"
#include "../module/exec_timing.h"

#include <inttypes.h> //uint8_t
#include <bitset>

namespace mavhub {

  class MSPTuningApp : public AppLayer<mavlink_message_t>, public AppLayer<msp_message_t> {
  public:
    static const int component_id = 46;

    MSPTuningApp(const std::map<std::string, std::string>,
           const Logger::log_level_t loglevel = Logger::LOGLEVEL_WARN);
    virtual ~MSPTuningApp();

    virtual void handle_input(const mavlink_message_t &msg);
    virtual void handle_input(const msp_message_t &msg);

  protected:
    virtual void print(std::ostream &os) const;
    virtual void run();

  private:
    /// execution timing
    Exec_Timing* exec_tmr;
    
    static const int parameter_count = 104;
    static const int pid_count = 30; // baseflight
    static const int8_t parameter_ids[parameter_count][15];
    static const int8_t pid_ids[pid_count][15];
    // uint8_t parameters[parameter_count];
    /// pre-packed heartbeat message
    mavlink_message_t heartbeat_msg;
    /// msp2mavlink message
    mavlink_message_t mavmsg;
    /// msp2mavlink param message
    mavlink_message_t mavmsgparam;
    /// msp send message
    msp_message_t msp_send_msg;
    // mavlink structures
    /// ident
    /// sys_status
    mavlink_sys_status_t sys_status;
    /// attitude
    mavlink_attitude_t attitude;
    /// position
    mavlink_global_position_int_t position;
    /// raw_imu
    mavlink_raw_imu_t raw_imu;
    /// motor output
    mavlink_servo_output_raw_t servo_output;
    /// rc channels raw
    mavlink_rc_channels_raw_t rc_channels;
    /// app params
    int run_cnt;
    /// update rate integer variable
    int update_rate;

    // params
    /// request
    int param_request_list;
    /// param container
    std::map<std::string, double> params;
    
    virtual void read_conf(const std::map<std::string, std::string> args);
    virtual void send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value);
  };
} // namespace mavhub

#endif // HAVE_MSPLINK_H
#endif // HAVE_MAVLINK_H
#endif // _MSP_TUNING_APP_H_
