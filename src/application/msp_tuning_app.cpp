#include "msp_tuning_app.h"

#ifdef HAVE_MAVLINK_H
#ifdef HAVE_MSPLINK_H

#include "core/logger.h"
#include "core/datacenter.h"

using namespace std;

#define DEG2RAD 0.0017453292519943295

namespace mavhub {

// macros for state_vector
#define STATE_PARAM_REQUEST_LIST	1

  const int8_t MSPTuningApp::parameter_ids[parameter_count][15] = {
    "REVISION",
    "CHANNEL_0",
    "CHANNEL_1",
    "CHANNEL_2",
    "CHANNEL_3",
    "CHANNEL_4",
    "CHANNEL_5",
    "CHANNEL_6",
    "CHANNEL_7",
    "CHANNEL_8",
    "CHANNEL_9",
    "CHANNEL_10",
    "CHANNEL_11",
    "GLOBALCONFIG",
    "HEIGHT_MIN_GAS",
    "PRESSURE_PID_D",
    "HEIGHT_MAX",
    "HEIGHT_PID_P",
    "HEIGHT_GAIN",
    "HEIGHT_ACC_IMP",
    "HEIGHT_HOVER",
    "HEIGHT_GPS_Z",
    "HEIGHT_NEUTRAL",
    "STICK_PID_P",
    "STICK_PID_D",
    "PID_YAW_P",
    "GAS_MIN",
    "GAS_MAX",
    "GYRO_ACC_GAIN",
    "COMPASS_IMP",
    "GYRO_PID_P",
    "GYRO_PID_I",
    "GYRP_PID_D",
    "GYRO_YAW_P",
    "GYRO_YAW_I",
    "GYRO_STABILITY",
    "LOW_VOL_WARN",
    "GAS_EMERGENCY",
    "GAS_EMER_TIME",
    "RECEIVER",
    "PID_I",
    "USERPARAM_1",
    "USERPARAM_2",
    "USERPARAM_3",
    "USERPARAM_4",
    "SERVO_NICK_CTL",
    "SERVO_NICK_COM",
    "SERVO_NICK_MIN",
    "SERVO_NICK_MAX",
    "SERVO_ROLL_CTL",
    "SERVO_ROLL_COM",
    "SERVO_ROLL_MIN",
    "SERVO_ROLL_MAX",
    "SERVO_NICK_REF",
    "SERVO_CTL_SPEE",
    "CAM_ORIENT",
    "SERVO_3",
    "SERVO_4",
    "SERVO_5",
    "LOOP_GAS_LIMIT",
    "LOOP_THRESHOLD",
    "LOOP_HYSTERESE",
    "AXIS_LINK_1",
    "AXIS_LINK_2",
    "AXIS_YAW_CORR",
    "ANGLE_TUO_NICK",
    "ANGLE_TUO_ROLL",
    "GYRO_ACC_ALIGN",
    "DRIFT_COMP",
    "STAB_DYNAMIC",
    "USERPARAM_5",
    "USERPARAM_6",
    "USERPARAM_7",
    "USERPARAM_8",
    "J16_BITMASK",
    "J16_TIMING",
    "J17_BITMASK",
    "J17_TIMING",
    "J16_BITM_WARN",
    "J17_BITM_WARN",
    "NAVI_GPS_MCTRL",
    "NAVI_GPS_GAIN",
    "NAVI_GPS_P",
    "NAVI_GPS_I",
    "NAVI_GPS_D",
    "NAVI_GPS_P_LIM",
    "NAVI_GPS_I_LIM",
    "NAVI_GPS_D_LIM",
    "NAVI_GSP_ACC",
    "NAVI_GPS_MISAT",
    "NAVI_STITHRESH",
    "NAVI_WINDCORR",
    "NAVI_SPEEDCOMP",
    "NAVI_OPRADIUS",
    "NAVI_ANGLELIM",
    "NAVI_LOGINTIME",
    "EXTERNCONTROL",
    "ORIENT_ANGLE",
    "ORIENT_MODCTRL",
    "MOTORSAFSWITCH",
    "BITCONFIG",
    "SERVO_COMP_INV",
    "EXTRACONFIG",
    "NAME_0"
  };

  const int8_t MSPTuningApp::pid_ids[pid_count][15] = {
    "P_ROLL",
    "I_ROLL",
    "D_ROLL",
    "P_PITCH",
    "I_PITCH",
    "D_PITCH",
    "P_YAW",
    "I_YAW",
    "D_YAW",
    "P_ALT",
    "I_ALT",
    "D_ALT",
    "P_Pos",
    "I_Pos",
    "D_Pos",
    "P_PosR",
    "I_PosR",
    "D_PosR",
    "P_NavR",
    "I_NavR",
    "D_NavR",
    "P_LEVEL",
    "I_LEVEL",
    "D_LEVEL",
    "P_MAG",
    "I_MAG",
    "D_MAG",
    "P_VEL",
    "I_VEL",
    "D_VEL"
  };

  MSPTuningApp::MSPTuningApp(const map<string, string> args,
                 const Logger::log_level_t loglevel) :
    AppInterface("msp_tuning_app", loglevel),
    AppLayer<mavlink_message_t>("msp_tuning_app", loglevel),
    AppLayer<msp_message_t>("msp_tuning_app", loglevel),
    run_cnt(0)
    // 	msp_dev(serial_port, UART::baudrate_to_speed(baudrate) | CS8 | CLOCAL | CREAD),
    // 	message_time( get_time_us() ),
    // 	attitude_time( get_time_us() ),
    // 	parameter_request(255),
    // 	parameter_time(0) {
  {
    // 	pthread_mutex_init(&tx_mav_mutex, NULL);
    // 	pthread_mutex_init(&tx_msp_mutex, NULL);
    // 	msplink_msg_init(&tx_msp_msg);
    int mav_type = MAV_TYPE_GENERIC;
    int component_id = 46;
    int autopilot = MAV_AUTOPILOT_GENERIC;
    //system_id()
    mavlink_msg_heartbeat_pack(system_id(),
                               component_id,
                               &heartbeat_msg,
                               mav_type,
                               autopilot,
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,	//base mode
                               0,	//custom mode
                               MAV_STATE_ACTIVE);	//system status
    read_conf(args);
    assign_variable_from_args(update_rate);

    // init execution timer
    exec_tmr = new Exec_Timing(update_rate);

    //
    msplink_msg_init_rq(&msp_send_msg);
  }

  MSPTuningApp::~MSPTuningApp() {}

  void MSPTuningApp::handle_input(const mavlink_message_t &msg) {
    static char param_id[16];
    // log("MSPTuningApp got mavlink_message", static_cast<int>(msg.msgid), Logger::LOGLEVEL_DEBUG);
    switch(msg.msgid) {
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
      Logger::log("MSPTuningApp::handle_input: PARAM_REQUEST_LIST", Logger::LOGLEVEL_INFO);
      if(mavlink_msg_param_request_list_get_target_system (&msg) == system_id()) {
        param_request_list = 1;
      }
      break;
    case MAVLINK_MSG_ID_PARAM_SET:
      if(mavlink_msg_param_set_get_target_system(&msg) == system_id()) {
        Logger::log("MSPTuningApp::handle_input: PARAM_SET for this system", (int)system_id(), Logger::LOGLEVEL_INFO);
        if(mavlink_msg_param_set_get_target_component(&msg) == component_id) {
          Logger::log("MSPTuningApp::handle_input: PARAM_SET for this component", (int)component_id, Logger::LOGLEVEL_INFO);
          mavlink_msg_param_set_get_param_id(&msg, param_id);
          Logger::log("MSPTuningApp::handle_input: PARAM_SET for param_id", param_id, Logger::LOGLEVEL_INFO);
          
          typedef map<string, double>::const_iterator ci;
          for(ci p = params.begin(); p!=params.end(); ++p) {
            // Logger::log("ctrl_zrate param test", p->first, p->second, Logger::LOGLEVEL_INFO);
            if(!strcmp(p->first.data(), (const char *)param_id)) {
              params[p->first] = mavlink_msg_param_set_get_param_value(&msg);
              Logger::log("x MSPTuningApp::handle_input: PARAM_SET request for", p->first, params[p->first], Logger::LOGLEVEL_INFO);
            }
          }

          // assemble msp send data
          uint8_t pid_vals[pid_count];
          std::ostringstream logstream;
          logstream << "sending pid values: ";
          for (int i = 0; i < pid_count; i++) {
            pid_vals[i] = (uint8_t)params[(const char*)pid_ids[i]];
            logstream << "pid[" << (int)i << "] = " << (int)pid_vals[i] << ", ";
          }
          msplink_msg_encode(&msp_send_msg, MSP_SET_PID, pid_vals, sizeof(pid_vals));
          Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
          AppLayer<msp_message_t>::send(msp_send_msg);
        }
      }
      break;
    default:
      break;
    }
  }

  void MSPTuningApp::handle_input(const msp_message_t& msg) {
    // log("MSPTuningApp got msp_message", static_cast<int>(msg.type), Logger::LOGLEVEL_DEBUG);
    std::ostringstream logstream;
    const msp_ident_t *msp_ident;
    const msp_status_t *msp_status;
    const msp_raw_imu_t *msp_raw_imu;
    const msp_motor_t *msp_motor;
    const msp_rc_t *msp_rc;
    const msp_attitude_t *msp_attitude;
    const msp_altitude_t *msp_altitude;
    const msp_analog_t *msp_analog;
    const uint8_t *pid_vals;
    
    // // checksum
    // int i;
    // uint8_t checksum = 0;
    // checksum ^= msg.len;
    // checksum ^= msg.type;
    // for(i = 0; i < msg.len; i++)
    //   checksum ^= msg.data[i];

    // if (checksum != msg.hash) {
    //   logstream << "checksum err? " << (int)checksum << ", " << (int)msg.hash;
    //   Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
    //   return;
    // }      
        
    switch(msg.type) {
    case MSP_IDENT: {
      msp_ident = reinterpret_cast<const msp_ident_t*>(msg.data);
      logstream << "Got MSP_IDENT, MW Version: " << (int)msp_ident->version << ", Multitype:  " << (int)msp_ident->multitype;
      Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
      // mavlink send heartbeat
      AppLayer<mavlink_message_t>::send(heartbeat_msg);
      break;
    }
    case MSP_STATUS:
      msp_status = reinterpret_cast<const msp_status_t*>(msg.data);
      // logstream << "STATUS cycletime: " << msp_status->cycletime;
      // logstream << ", STATUS batt: " << msp_status->;
      // Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
      // mavlink_msg_sys_status_pack(system_id(), 0, &mavmsg,
      //                             0, 0, 0,
      //                             msp_status->cycletime/1000, // load
      //                             0, //
      //                             0, 0, 0, 0, 0, 0, 0, 0);
      sys_status.load = msp_status->cycletime/1000;
      // AppLayer<mavlink_message_t>::send(mavmsg);
      break;
    case MSP_ANALOG:
      msp_analog = reinterpret_cast<const msp_analog_t*>(msg.data);
      // logstream << "ANALOG vbat: " << (int)msp_analog->vbat;
      // Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
      sys_status.voltage_battery = msp_analog->vbat * 100;
      // AppLayer<mavlink_message_t>::send(mavmsg);
      break;
    case MSP_RAW_IMU:
      msp_raw_imu = reinterpret_cast<const msp_raw_imu_t*>(msg.data);
      // std::ostringstream logstream;
      // logstream << "ACC x: " << msp_raw_imu->ax << ", y: " << msp_raw_imu->ay << ", z: " << msp_raw_imu->az;
      // Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
      // logstream.str("");
      // logstream << "GYR x: " << msp_raw_imu->gx << ", y: " << msp_raw_imu->gy << ", z: " << msp_raw_imu->gz;
      // Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
      // convert to mavlink
      // mavlink_msg_raw_imu_pack(system_id(), 0, &mavmsg, 0,
      //                          msp_raw_imu->ax, msp_raw_imu->ay, msp_raw_imu->az,
      //                          msp_raw_imu->gx, msp_raw_imu->gy, msp_raw_imu->gz,
      //                          msp_raw_imu->magx, msp_raw_imu->magy, msp_raw_imu->magz);
      raw_imu.xacc = msp_raw_imu->ax;
      raw_imu.yacc = msp_raw_imu->ay;
      raw_imu.zacc = msp_raw_imu->az;
      raw_imu.xgyro = msp_raw_imu->gx;
      raw_imu.ygyro = msp_raw_imu->gy;
      raw_imu.zgyro = msp_raw_imu->gz;
      raw_imu.xmag = msp_raw_imu->magx;
      raw_imu.ymag = msp_raw_imu->magy;
      raw_imu.zmag = msp_raw_imu->magz;
      // AppLayer<mavlink_message_t>::send(mavmsg);
      break;

    case MSP_MOTOR:
      msp_motor = reinterpret_cast<const msp_motor_t*>(msg.data);
      // logstream << "MOTOR 1-4: [" << msp_motor->mot[0] << ", " << msp_motor->mot[1] << ", "
      //           << msp_motor->mot[2] << ", " << msp_motor->mot[3] << "]";
      // Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
      // mavlink_msg_servo_output_raw_pack(system_id(), 0, &mavmsg, (uint32_t)0, (uint8_t)0,
      //                                   msp_motor->mot[0], msp_motor->mot[1],
      //                                   msp_motor->mot[2], msp_motor->mot[3],
      //                                   msp_motor->mot[4], msp_motor->mot[5],
      //                                   msp_motor->mot[6], msp_motor->mot[7]);
      servo_output.servo1_raw = msp_motor->mot[0];
      servo_output.servo2_raw = msp_motor->mot[1];
      servo_output.servo3_raw = msp_motor->mot[2];
      servo_output.servo4_raw = msp_motor->mot[3];
      servo_output.servo5_raw = msp_motor->mot[4];
      servo_output.servo6_raw = msp_motor->mot[5];
      servo_output.servo7_raw = msp_motor->mot[6];
      servo_output.servo8_raw = msp_motor->mot[7];
      // AppLayer<mavlink_message_t>::send(mavmsg);
      break;

    case MSP_RC:
      msp_rc = reinterpret_cast<const msp_rc_t*>(msg.data);
      // logstream << "RC 1-4: [" << msp_rc->roll << ", " << msp_rc->pitch << ", "
      //           << msp_rc->yaw << ", " << msp_rc->throttle << "]";
      // Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
      rc_channels.chan1_raw = msp_rc->roll;
      rc_channels.chan2_raw = msp_rc->pitch;
      rc_channels.chan3_raw = msp_rc->yaw;
      rc_channels.chan4_raw = msp_rc->throttle;
      rc_channels.chan5_raw = msp_rc->aux1;
      rc_channels.chan6_raw = msp_rc->aux2;
      rc_channels.chan7_raw = msp_rc->aux3;
      rc_channels.chan8_raw = msp_rc->aux4;
      rc_channels.port = 0;
      rc_channels.rssi = 0;
      break;
      
    case MSP_ATTITUDE:
      msp_attitude = reinterpret_cast<const msp_attitude_t*>(msg.data);
      attitude.roll = (float)((int16_t)msp_attitude->roll) * DEG2RAD;
      attitude.pitch = (float)((int16_t)msp_attitude->pitch) * DEG2RAD;
      attitude.yaw = (float)((int16_t)msp_attitude->yaw); // * DEG2RAD;
      attitude.rollspeed = 0.;
      attitude.pitchspeed = 0.;
      attitude.yawspeed = 0.;
      break;
    case MSP_ALTITUDE:
      msp_altitude = reinterpret_cast<const msp_altitude_t*>(msg.data);
      // logstream << "alt: " << msp_altitude->alt << ", vario: " << msp_altitude->vario;
      // Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
      position.lat = 0;
      position.lon = 0;
      position.alt          = (int)(msp_altitude->alt);
      position.relative_alt = (int)(msp_altitude->alt);
      position.vx  = 0;
      position.vy  = 0;
      position.vz           = (int)(msp_altitude->vario);
      position.hdg = 0;
      break;
    case MSP_PID:
      pid_vals = reinterpret_cast<const uint8_t*>(msg.data);
      logstream << "received ";
      for(int i = 0; i < pid_count; i++) {
        logstream << "pid[" << (int)i << "] = " << (int)pid_vals[i] << ", ";
        params[(const char*)pid_ids[i]] = (float)pid_vals[i];
        mavlink_msg_param_value_pack(system_id(), component_id, &mavmsgparam,
                                     (const char*) pid_ids[i], (float)pid_vals[i],
                                     MAVLINK_TYPE_FLOAT, 1, 0);
        AppLayer<mavlink_message_t>::send(mavmsgparam);
      }
      Logger::log(logstream.str(), Logger::LOGLEVEL_DEBUG);
      
      // case MSP_MSG_TYPE_MSP_IMU: {
      //   const msp_msp_imu_t *msp_imu = reinterpret_cast<const msp_msp_imu_t*>(msg.data);
      //   // std::ostringstream logstream;
      //   // logstream << message_time << " " 
      //   // 	<< msp_imu->x_adc_acc << " "
      //   // 	<< msp_imu->y_adc_acc << " "
      //   // 	<< msp_imu->z_adc_acc << " "
      //   // 	<< msp_imu->x_adc_gyro << " "
      //   // 	<< msp_imu->y_adc_gyro << " "
      //   // 	<< msp_imu->z_adc_gyro;
      //   // Logger::log(logstream.str(), Logger::LOGLEVEL_ALL, Logger::LOGLEVEL_ALL);
      //   mavlink_huch_imu_raw_adc_t imu_raw_adc;
      //   imu_raw_adc.xacc = msp_imu->x_adc_acc;
      //   imu_raw_adc.yacc = msp_imu->y_adc_acc;
      //   imu_raw_adc.zacc = msp_imu->z_adc_acc;
      //   imu_raw_adc.xgyro = msp_imu->x_adc_gyro;
      //   imu_raw_adc.ygyro = msp_imu->y_adc_gyro;
      //   imu_raw_adc.zgyro = msp_imu->z_adc_gyro;
      //   DataCenter::set_huch_imu_raw_adc(imu_raw_adc);
      //   Lock tx_lock(tx_mav_mutex);
      //   //forward raw ADC
      //   mavlink_msg_huch_imu_raw_adc_encode(owner()->system_id(),
      //                                       component_id,
      //                                       &tx_mav_msg,
      //                                       &imu_raw_adc
      //                                       );
      //   send(tx_mav_msg);
      //   //forward MSP IMU
      //   //TODO: add compass value and baro
      //   mavlink_huch_msp_imu_t huch_msp_imu;
      //   huch_msp_imu.usec = message_time;
      //   huch_msp_imu.xacc = (2500*msp_imu->x_acc)/1024; //convert normalized analog to mg
      //   huch_msp_imu.yacc = (2500*msp_imu->y_acc)/1024;
      //   huch_msp_imu.zacc = (2500*msp_imu->z_acc)/1024;
      //   huch_msp_imu.xgyro = (6700*msp_imu->x_adc_gyro)/(3*1024); //convert analog to 0.1 deg./sec.
      //   huch_msp_imu.ygyro = (6700*msp_imu->y_adc_gyro)/(3*1024);
      //   huch_msp_imu.zgyro = (6700*msp_imu->z_adc_gyro)/(3*1024);
      //   DataCenter::set_huch_msp_imu(huch_msp_imu);
      //   mavlink_msg_huch_msp_imu_encode( owner()->system_id(),
      //                                    component_id,
      //                                    &tx_mav_msg,
      //                                    &huch_msp_imu
      //                                    );
      //   send(tx_mav_msg);
      //   //forward pressure
      //   mavlink_raw_pressure_t raw_pressure;
      //   raw_pressure.usec = message_time;
      //   raw_pressure.press_abs = msp_imu->press_abs;
      //   raw_pressure.press_diff1 = 0;	//press_diff1
      //   raw_pressure.press_diff2 = 0;	//press_diff2
      //   raw_pressure.temperature = 0;	//temperature
      //   DataCenter::set_raw_pressure(raw_pressure);
      //   mavlink_msg_raw_pressure_encode(owner()->system_id(),
      //                                   component_id,
      //                                   &tx_mav_msg,
      //                                   &raw_pressure
      //                                   );
      //   send(tx_mav_msg);
      //   //TODO: forward magneto
      //   break;
      // }
      // case MSP_MSG_TYPE_PARAM_VALUE: {
      //   const msp_param_t *param = reinterpret_cast<const msp_param_t*>(msg.data);
      //   //set parameter
      //   uint8_t index;
      //   if(param->index >= parameter_count)
      //     index = parameter_count-1;
      //   else
      //     index = param->index;
      //   parameters[index] = param->value;
      //   //ask for next parameter
      //   if(index < parameter_count - 1) {
      //     parameter_request = index + 1;
      //     msp_param_type_t param_type= static_cast<msp_param_type_t>(parameter_request);
      //     send(MSP_MSG_TYPE_PARAM_REQUEST, &param_type, sizeof(msp_param_type_t));
      //     parameter_time = message_time;
      //   } else { //got all parameters
      //     parameter_request = 255;
      //   }
      //   //inform others
      //   send_mavlink_param_value( static_cast<msp_param_type_t>(index) );
      //   break;
      // }
      // case MSP_MSG_TYPE_ACTION_ACK: {

      //   Lock tx_lock(tx_mav_mutex);
      //   mavlink_msg_action_ack_pack(owner()->system_id(), component_id, &tx_mav_msg, msg.data[0], msg.data[1]);
      //   send(tx_mav_msg);
      //   break;
      // }
      // case MSP_MSG_TYPE_SYSTEM_STATUS: {
      //   const msp_system_status_t *sys_status = reinterpret_cast<const msp_system_status_t*>(msg.data);
      //   Lock tx_lock(tx_mav_mutex);
      //   mavlink_msg_sys_status_pack(owner()->system_id(),
      //                               component_id,
      //                               &tx_mav_msg,
      //                               sys_status->mode,
      //                               sys_status->nav_mode,
      //                               sys_status->state,
      //                               1000, //FIXME: use glibtop to get load of linux system
      //                               sys_status->vbat*100, //convert dV to mV
      //                               0,//motor block (unsupported)
      //                               sys_status->packet_drop);
      //   send(tx_mav_msg);
      //   break;
      // }
      // case MSP_MSG_TYPE_BOOT:
      //   //TODO
      //   break;
    default:
      break;
    }
  }

  void MSPTuningApp::print(std::ostream &os) const {
    // 	AppLayer<mavlink_message_t>::print(os);

    // 	os << "* device: " << msp_dev;
  }

  void MSPTuningApp::run() {
    // fd_set read_fds;
    // int fds_ready;
    // timeval timeout;
    // uint8_t input;
    // msp_message_t rx_msg;
    // msplink_status_t link_status;
    // uint64_t delta_time = 0, tmp64;
    static mavlink_message_t mavlink_msg;
    static mavlink_debug_t dbg;
    msp_message_t msp_msg;
    msp_ident_t ident;
    // msp_status_t status;
    // msp_raw_imu_t raw_imu;
    // msp_motor_t motor;
    msp_rc_t rc;
    // msp_rc_t rc;
    static int n=0;
    int sleeptime, i;
    uint64_t dt;

    msplink_msg_init_rq(&msp_msg);

    // msp_msg.sync = MSP_MESSAGE_START;
    log("MSPTuningApp running", Logger::LOGLEVEL_DEBUG);

    // msplink_status_initialize(&link_status);
    // loop forever
    while( !interrupted() ) {
      // handle parameter requests
      if(param_request_list) {
        Logger::log("MSPTuningApp::run: param request", Logger::LOGLEVEL_INFO);
        param_request_list = 0;

        typedef map<string, double>::const_iterator ci;
        for(ci p = params.begin(); p!=params.end(); ++p) {
          // Logger::log("ctrl_zrate param test", p->first, p->second, Logger::LOGLEVEL_INFO);
          mavlink_msg_param_value_pack(system_id(), component_id, &mavlink_msg, (const char*) p->first.data(), p->second, MAVLINK_TYPE_FLOAT, 1, 0);
          AppLayer<mavlink_message_t>::send(mavlink_msg);


          // send MSP_PID
          msp_msg.len = 0;
          msp_msg.type = MSP_PID;
          AppLayer<msp_message_t>::send(msp_msg);
        }

        // mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"prm_test_pitch", prm_test_pitch, 1, 0);
        // send(msg);
        // mavlink_msg_param_value_pack(owner()->system_id(), component_id, &msg, (int8_t *)"prm_yaw_P", prm_yaw_P, 1, 0);
        // send(msg);
      }

      sleeptime = exec_tmr->calcSleeptime();
      //Logger::log("plat_link_car.cpp: sleeptime: ", sleeptime, Logger::LOGLEVEL_INFO);
      usleep(sleeptime);
      dt = exec_tmr->updateExecStats();

      // send out mavlink data
      // sys status
      mavlink_msg_sys_status_encode(system_id(), component_id, &mavmsg, &sys_status);
      AppLayer<mavlink_message_t>::send(mavmsg);
      // raw imu
      mavlink_msg_raw_imu_encode(system_id(), component_id, &mavmsg, &raw_imu);
      AppLayer<mavlink_message_t>::send(mavmsg);
      // motors
      mavlink_msg_servo_output_raw_encode(system_id(), component_id, &mavmsg, &servo_output);
      AppLayer<mavlink_message_t>::send(mavmsg);
      // rc channels
      mavlink_msg_rc_channels_raw_encode(system_id(), component_id, &mavmsg, &rc_channels);
      AppLayer<mavlink_message_t>::send(mavmsg);
      // attitude
      mavlink_msg_attitude_encode(system_id(), component_id, &mavmsg, &attitude);
      AppLayer<mavlink_message_t>::send(mavmsg);
      // altitude
      mavlink_msg_global_position_int_encode(system_id(), component_id, &mavmsg, &position);
      AppLayer<mavlink_message_t>::send(mavmsg);
      
      
      // send MSP requests
      if(run_cnt % update_rate == 0) {
        msp_msg.len = 0;
        msp_msg.type = MSP_IDENT;
        msplink_msg_encode(&msp_msg, MSP_IDENT, &ident, sizeof(ident));
        AppLayer<msp_message_t>::send(msp_msg);
        // usleep(10000);
        Logger::log(name(), "sent ident", Logger::LOGLEVEL_DEBUG);
      }
      
      msp_msg.len = 0;
      msp_msg.type = MSP_STATUS;
      AppLayer<msp_message_t>::send(msp_msg);
      // usleep(10000);

      msp_msg.len = 0;
      msp_msg.type = MSP_ANALOG;
      AppLayer<msp_message_t>::send(msp_msg);
      
      msp_msg.len = 0;
      msp_msg.type = MSP_RAW_IMU;
      AppLayer<msp_message_t>::send(msp_msg);
      // usleep(10000);

      msp_msg.len = 0;
      msp_msg.type = MSP_MOTOR;
      AppLayer<msp_message_t>::send(msp_msg);
      // usleep(10000);

      msp_msg.len = 0;
      msp_msg.type = MSP_RC;
      AppLayer<msp_message_t>::send(msp_msg);
      // usleep(10000);

      msp_msg.len = 0;
      msp_msg.type = MSP_ATTITUDE;
      AppLayer<msp_message_t>::send(msp_msg);
      // usleep(10000);

      msp_msg.len = 0;
      msp_msg.type = MSP_ALTITUDE;
      AppLayer<msp_message_t>::send(msp_msg);
      // usleep(10000);

      // Logger::log(name(), "run", Logger::LOGLEVEL_DEBUG);
      run_cnt++;
      // usleep(10000);
    }
    log("MSPTuningApp stopped", Logger::LOGLEVEL_DEBUG);
  }

  // size_t MSPTuningApp::send(const msp_message_t& msg) {
  // 	size_t sent = 0;

  // 	sent += msp_dev.write(&(msg.sync), 3);
  // 	sent += msp_dev.write(msg.data, msg.len);
  // 	sent += msp_dev.write(&(msg.hash), 1);

  // 	return sent;
  // }

  // size_t MSPTuningApp::send(const msp_msg_type_t type, const void *data, const uint8_t size) {
  // 	size_t sent = 0;
  // 	char buf = MSP_MESSAGE_START;
  // 	Lock tx_lock(tx_msp_mutex);
  // 
  // 	//send header and data
  // 	sent += msp_dev.write(&buf, 1);
  // 	sent += msp_dev.write(&type, 1);
  // 	sent += msp_dev.write(&size, 1);
  // 	sent += msp_dev.write(data, size);
  // 
  // 	//calc and send hash value
  // 	uint8_t hash;
  // 	msplink_hash_init(&hash);
  // 	hash = msplink_hash_update(type, hash);
  // 	hash = msplink_hash_update(size, hash);
  // 	const uint8_t *data_ptr = static_cast<const uint8_t*>(data);
  // 	for(uint8_t i=0; i<size; i++) {
  // 		hash = msplink_hash_update(*data_ptr++, hash);
  // 	}
  // 	hash = ~hash;
  // 	sent += msp_dev.write(&hash, 1);

  // 	return sent;
  // }

  // void MSPTuningApp::send_heartbeat() {
  /*	Lock tx_lock(tx_mav_mutex);
	mavlink_msg_heartbeat_pack(owner()->system_id(), component_id, &tx_mav_msg, MAV_QUADROTOR, MAV_AUTOPILOT_HUCH);
	send(tx_mav_msg);*/
  // }

  // void MSPTuningApp::send_mavlink_param_value(const msp_param_type_t param_type) {
  /*	Lock tx_lock(tx_mav_mutex);

	mavlink_msg_param_value_pack(owner()->system_id(),
        component_id,
        &tx_mav_msg,
        get_parameter_id(param_type),
        static_cast<float>( get_parameter(param_type) ),
        parameter_count,
        param_type);
	send(tx_mav_msg);*/
  // }

  // const int MSPTuningApp::parameter_id_to_index(const int8_t *parameter_id) {
  //TODO: use efficient search algorithm
  /*	for(int i=0; i<parameter_count; i++) {
        if(strncmp( (const char*)parameter_id, (const char*)parameter_ids[i], 15) == 0) {
        return i;
        }
	}*/
  // 	return -1;
  // }
  void MSPTuningApp::read_conf(const map<string, string> args) {
    map<string,string>::const_iterator iter;

    // iter = args.find("component_id");
    // if( iter != args.end() ) {
    //   istringstream s(iter->second);
    //   s >> component_id;
    // }

    // PID x component
    iter = args.find("pid_Kc_x");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["pid_Kc_x"];
    }
    iter = args.find("pid_Ti_x");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["pid_Ti_x"];
    }
    iter = args.find("pid_Td_x");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["pid_Td_x"];
    }
    iter = args.find("of_gyw_y");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["of_gyw_y"];
    }

    // PID y component
    iter = args.find("pid_Kc_y");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["pid_Kc_y"];
    }
    iter = args.find("pid_Ti_y");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["pid_Ti_y"];
    }
    iter = args.find("pid_Td_y");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["pid_Td_y"];
    }
    // PID z component
    iter = args.find("pid_Kc_z");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["pid_Kc_z"];
    }
    iter = args.find("pid_Ti_z");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["pid_Ti_z"];
    }
    iter = args.find("pid_Td_z");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["pid_Td_z"];
    }

    iter = args.find("of_gyw_x");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["of_gyw_x"];
    }

    // PID limit
    iter = args.find("pitch_limit");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["pitch_limit"];
    }
    iter = args.find("roll_limit");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["roll_limit"];
    }
    iter = args.find("throttle_limit");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["throttle_limit"];
    }

    // reset integral
    iter = args.find("reset_i");
    if( iter != args.end() ) {
      istringstream s(iter->second);
      s >> params["reset_i"];
    }
    else {
      params["reset_i"] = 0.0;
    }

  }

  void MSPTuningApp::send_debug(mavlink_message_t* msg, mavlink_debug_t* dbg, int index, double value) {
    dbg->ind = index;
    dbg->value = value;
    mavlink_msg_debug_encode(system_id(), static_cast<uint8_t>(component_id), msg, dbg);
    AppLayer<mavlink_message_t>::send(*msg);
  }


} // namespace mavhub

#endif // HAVE_MSPLINK_H
#endif // HAVE_MAVLINK_H

