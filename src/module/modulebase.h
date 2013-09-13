#ifndef _MODULEBASE_H_
#define _MODULEBASE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_MAVLINK_H
#include <mavlink.h>

#include <inttypes.h> //uint8_t
#include "protocol/protocollayer.h"

// controller modes
enum ctl_mode_t {
  CTL_MODE_NULL, // do nothing
  CTL_MODE_BUMP, // do system bumping
  CTL_MODE_AC,   // dpo altitude control
  CTL_MODE_DIRECT, // directly provide thrust value
  CTL_MODE_NMODES // number of items in enum
};

// channel types
enum huch_generic_channel_types_t {
  CHAN_THRUST, // Thrust value
  CHAN_ROLL, // Roll value
  CHAN_PITCH,   // Pitch value
  CHAN_YAW, // Yaw value
  CHAN_TRIG_BUMP_ROLL, // trigger roll bumping
  CHAN_LC_ACTIVE, // lateral control active
  CHAN_NUM // number of items in enum
};

// channel types
enum huch_action_types_t {
  ACTION_BUMP_ROLL, // trigger roll bumping
  ACTION_TOGGLE_AC, // toggle altitude control on/off
  ACTION_TOGGLE_LC, // toggle lateral control on/off
  ACTION_TOGGLE_LAT_PID_OPT, // toggle lateral PID optimization
  ACTION_NUM // number of items in enum
};

namespace mavhub {
  class ModuleBase : public AppLayer<mavlink_message_t> {
  public:
    ModuleBase(const std::map<std::string, std::string> args, std::string name);
    virtual ~ModuleBase();
    // virtual void handle_input(const mavlink_message_t &msg) = 0;

  protected:
    // virtual void run() = 0;

    /// component id
    uint16_t component_id;
    /// params requested for transmission
    bool param_request_list;
    /// parameter dict
    std::map<std::string, double>	params;
    /// output enable
    uint8_t output_enable;

  private:
    /// read configuration into params map
    virtual void read_conf(const std::map<std::string, std::string> args) = 0;
    /* /// handle parameter list request */
    /* virtual void param_request_respond(); */

  protected:
    /// base message
    mavlink_message_t msg;
    /// base message
    mavlink_debug_t dbg;
    /// parameter setting
    mavlink_param_set_t param_set;
    /// mavlink action
    mavlink_huch_action_t action;
    /// mavlink action
    mavlink_huch_generic_channel_t chan;

    /// respond to parameters request
    inline void param_request_respond() {
      if(param_request_list) {
        Logger::log(name(), "param_request_respond", Logger::LOGLEVEL_INFO);
        param_request_list = false;
        typedef std::map<std::string, double>::const_iterator ci;
        for(ci p = params.begin(); p!=params.end(); ++p) {
          Logger::log(name(), "param test", p->first, p->second, Logger::LOGLEVEL_INFO);
          mavlink_msg_param_value_pack(system_id(), component_id, &msg, (const char*) p->first.data(), p->second, MAVLINK_TYPE_FLOAT, 1, 0);
          AppLayer<mavlink_message_t>::send(msg);
        }
      }
    }

    /// send debug message
    inline void send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value) {
      dbg->ind = index;
      dbg->value = value;
      mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), msg, dbg);
      AppLayer<mavlink_message_t>::send(*msg);
    }

    /// request data stream
    inline void send_stream_request(mavlink_message_t* msg,
                                    uint8_t req_stream_id,
                                    uint16_t req_message_rate) {

      mavlink_request_data_stream_t req_datastream;
      req_datastream.target_system = system_id();
      req_datastream.target_component = component_id;
      req_datastream.req_stream_id = req_stream_id;
      req_datastream.req_message_rate = req_message_rate;
      req_datastream.start_stop = 1;
      mavlink_msg_request_data_stream_encode(system_id(),
                                             component_id,
                                             msg,
                                             &req_datastream);
      // Logger::log(name(), "sending stream request", Logger::LOGLEVEL_DEBUG);
      AppLayer<mavlink_message_t>::send(*msg);
      return;
    }

    /// set mavlink parameter
    inline void param_request_set(int target_system,
                                  int target_component,
                                  std::string param_id,
                                  float value) {
      param_set.param_id[0] = '\0';

      param_set.target_system = target_system;
      param_set.target_component = target_component;
      strncpy((char *)param_set.param_id, param_id.c_str(), param_id.length() + 1);
      param_set.param_value = value;
      mavlink_msg_param_set_pack(system_id(), component_id,
                                 &msg,
                                 param_set.target_system, 
                                 param_set.target_component, 
                                 param_set.param_id,
                                 param_set.param_value,
                                 MAVLINK_TYPE_FLOAT);
      AppLayer<mavlink_message_t>::send(msg);

      Logger::log(name(), "sent param_set", (int)param_set.target_system, param_set.param_id, param_set.param_value, Logger::LOGLEVEL_DEBUG);

    }

    inline void send_action(int target_system,
                            int target_component,
                            int action_id) {

      action.target = target_system;
      action.target_component = target_component; // roll bump
      action.action = action_id;
      mavlink_msg_huch_action_encode(system_id(), component_id,
                                     &msg,
                                     &action);
      AppLayer<mavlink_message_t>::send(msg);

      Logger::log(name(), "sent action", (int)action.target, (int)action.target_component, Logger::LOGLEVEL_DEBUG);
			
    }

  };
}

#endif // HAVE_MAVLINK_H

#endif
