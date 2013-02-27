/** @file
 *	@brief MAVLink comm protocol testsuite generated from huch.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef HUCH_TESTSUITE_H
#define HUCH_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_huch(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_huch(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_mk_extern_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mk_extern_control_t packet_in = {
		17235,
	17339,
	17,
	84,
	151,
	218,
	29,
	96,
	163,
	};
	mavlink_mk_extern_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.gas = packet_in.gas;
        	packet1.height = packet_in.height;
        	packet1.remote_buttons = packet_in.remote_buttons;
        	packet1.nick = packet_in.nick;
        	packet1.roll = packet_in.roll;
        	packet1.yaw = packet_in.yaw;
        	packet1.AP_flags = packet_in.AP_flags;
        	packet1.frame = packet_in.frame;
        	packet1.config = packet_in.config;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mk_extern_control_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mk_extern_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mk_extern_control_pack(system_id, component_id, &msg , packet1.remote_buttons , packet1.nick , packet1.roll , packet1.yaw , packet1.gas , packet1.height , packet1.AP_flags , packet1.frame , packet1.config );
	mavlink_msg_mk_extern_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mk_extern_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.remote_buttons , packet1.nick , packet1.roll , packet1.yaw , packet1.gas , packet1.height , packet1.AP_flags , packet1.frame , packet1.config );
	mavlink_msg_mk_extern_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mk_extern_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mk_extern_control_send(MAVLINK_COMM_1 , packet1.remote_buttons , packet1.nick , packet1.roll , packet1.yaw , packet1.gas , packet1.height , packet1.AP_flags , packet1.frame , packet1.config );
	mavlink_msg_mk_extern_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_exp_ctrl(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_exp_ctrl_t packet_in = {
		5,
	{ 72, 73, 74, 75, 76, 77, 78, 79, 80 },
	{ 163, 164 },
	};
	mavlink_huch_exp_ctrl_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.status = packet_in.status;
        
        	mav_array_memcpy(packet1.rx, packet_in.rx, sizeof(int8_t)*9);
        	mav_array_memcpy(packet1.tx, packet_in.tx, sizeof(int8_t)*2);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_exp_ctrl_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_exp_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_exp_ctrl_pack(system_id, component_id, &msg , packet1.status , packet1.rx , packet1.tx );
	mavlink_msg_huch_exp_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_exp_ctrl_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.status , packet1.rx , packet1.tx );
	mavlink_msg_huch_exp_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_exp_ctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_exp_ctrl_send(MAVLINK_COMM_1 , packet1.status , packet1.rx , packet1.tx );
	mavlink_msg_huch_exp_ctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_attitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_attitude_t packet_in = {
		963497464,
	963497672,
	963497880,
	17859,
	17963,
	18067,
	18171,
	18275,
	18379,
	18483,
	18587,
	18691,
	18795,
	18899,
	19003,
	19107,
	};
	mavlink_huch_attitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.xgyroint = packet_in.xgyroint;
        	packet1.ygyroint = packet_in.ygyroint;
        	packet1.zgyroint = packet_in.zgyroint;
        	packet1.xacc = packet_in.xacc;
        	packet1.yacc = packet_in.yacc;
        	packet1.zacc = packet_in.zacc;
        	packet1.zaccraw = packet_in.zaccraw;
        	packet1.xaccmean = packet_in.xaccmean;
        	packet1.yaccmean = packet_in.yaccmean;
        	packet1.zaccmean = packet_in.zaccmean;
        	packet1.xgyro = packet_in.xgyro;
        	packet1.ygyro = packet_in.ygyro;
        	packet1.zgyro = packet_in.zgyro;
        	packet1.xmag = packet_in.xmag;
        	packet1.ymag = packet_in.ymag;
        	packet1.zmag = packet_in.zmag;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_attitude_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_attitude_pack(system_id, component_id, &msg , packet1.xacc , packet1.yacc , packet1.zacc , packet1.zaccraw , packet1.xaccmean , packet1.yaccmean , packet1.zaccmean , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xgyroint , packet1.ygyroint , packet1.zgyroint , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_huch_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_attitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.xacc , packet1.yacc , packet1.zacc , packet1.zaccraw , packet1.xaccmean , packet1.yaccmean , packet1.zaccmean , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xgyroint , packet1.ygyroint , packet1.zgyroint , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_huch_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_attitude_send(MAVLINK_COMM_1 , packet1.xacc , packet1.yacc , packet1.zacc , packet1.zaccraw , packet1.xaccmean , packet1.yaccmean , packet1.zaccmean , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xgyroint , packet1.ygyroint , packet1.zgyroint , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_huch_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_fc_altitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_fc_altitude_t packet_in = {
		17235,
	17339,
	};
	mavlink_huch_fc_altitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.baro = packet_in.baro;
        	packet1.baroref = packet_in.baroref;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_fc_altitude_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_fc_altitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_fc_altitude_pack(system_id, component_id, &msg , packet1.baro , packet1.baroref );
	mavlink_msg_huch_fc_altitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_fc_altitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.baro , packet1.baroref );
	mavlink_msg_huch_fc_altitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_fc_altitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_fc_altitude_send(MAVLINK_COMM_1 , packet1.baro , packet1.baroref );
	mavlink_msg_huch_fc_altitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_ranger(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_ranger_t packet_in = {
		17235,
	17339,
	17443,
	};
	mavlink_huch_ranger_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.ranger1 = packet_in.ranger1;
        	packet1.ranger2 = packet_in.ranger2;
        	packet1.ranger3 = packet_in.ranger3;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_ranger_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_ranger_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_ranger_pack(system_id, component_id, &msg , packet1.ranger1 , packet1.ranger2 , packet1.ranger3 );
	mavlink_msg_huch_ranger_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_ranger_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ranger1 , packet1.ranger2 , packet1.ranger3 );
	mavlink_msg_huch_ranger_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_ranger_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_ranger_send(MAVLINK_COMM_1 , packet1.ranger1 , packet1.ranger2 , packet1.ranger3 );
	mavlink_msg_huch_ranger_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_exp_ctrl_rx(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_exp_ctrl_rx_t packet_in = {
		17235,
	17339,
	17443,
	17547,
	29,
	};
	mavlink_huch_exp_ctrl_rx_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.value0 = packet_in.value0;
        	packet1.value1 = packet_in.value1;
        	packet1.value2 = packet_in.value2;
        	packet1.value3 = packet_in.value3;
        	packet1.version = packet_in.version;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_exp_ctrl_rx_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_exp_ctrl_rx_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_exp_ctrl_rx_pack(system_id, component_id, &msg , packet1.version , packet1.value0 , packet1.value1 , packet1.value2 , packet1.value3 );
	mavlink_msg_huch_exp_ctrl_rx_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_exp_ctrl_rx_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.version , packet1.value0 , packet1.value1 , packet1.value2 , packet1.value3 );
	mavlink_msg_huch_exp_ctrl_rx_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_exp_ctrl_rx_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_exp_ctrl_rx_send(MAVLINK_COMM_1 , packet1.version , packet1.value0 , packet1.value1 , packet1.value2 , packet1.value3 );
	mavlink_msg_huch_exp_ctrl_rx_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mk_fc_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mk_fc_status_t packet_in = {
		17235,
	17339,
	17443,
	17547,
	17651,
	17755,
	};
	mavlink_mk_fc_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.rssi = packet_in.rssi;
        	packet1.batt = packet_in.batt;
        	packet1.nick = packet_in.nick;
        	packet1.roll = packet_in.roll;
        	packet1.yaw = packet_in.yaw;
        	packet1.gas = packet_in.gas;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mk_fc_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mk_fc_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mk_fc_status_pack(system_id, component_id, &msg , packet1.rssi , packet1.batt , packet1.nick , packet1.roll , packet1.yaw , packet1.gas );
	mavlink_msg_mk_fc_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mk_fc_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rssi , packet1.batt , packet1.nick , packet1.roll , packet1.yaw , packet1.gas );
	mavlink_msg_mk_fc_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mk_fc_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mk_fc_status_send(MAVLINK_COMM_1 , packet1.rssi , packet1.batt , packet1.nick , packet1.roll , packet1.yaw , packet1.gas );
	mavlink_msg_mk_fc_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_ctrl_hover_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_ctrl_hover_state_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
	};
	mavlink_huch_ctrl_hover_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.uss = packet_in.uss;
        	packet1.baro = packet_in.baro;
        	packet1.accz = packet_in.accz;
        	packet1.ir1 = packet_in.ir1;
        	packet1.ir2 = packet_in.ir2;
        	packet1.kal_s0 = packet_in.kal_s0;
        	packet1.kal_s1 = packet_in.kal_s1;
        	packet1.kal_s2 = packet_in.kal_s2;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_ctrl_hover_state_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_ctrl_hover_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_ctrl_hover_state_pack(system_id, component_id, &msg , packet1.uss , packet1.baro , packet1.accz , packet1.ir1 , packet1.ir2 , packet1.kal_s0 , packet1.kal_s1 , packet1.kal_s2 );
	mavlink_msg_huch_ctrl_hover_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_ctrl_hover_state_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.uss , packet1.baro , packet1.accz , packet1.ir1 , packet1.ir2 , packet1.kal_s0 , packet1.kal_s1 , packet1.kal_s2 );
	mavlink_msg_huch_ctrl_hover_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_ctrl_hover_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_ctrl_hover_state_send(MAVLINK_COMM_1 , packet1.uss , packet1.baro , packet1.accz , packet1.ir1 , packet1.ir2 , packet1.kal_s0 , packet1.kal_s1 , packet1.kal_s2 );
	mavlink_msg_huch_ctrl_hover_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_imu_raw_adc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_imu_raw_adc_t packet_in = {
		17235,
	17339,
	17443,
	17547,
	17651,
	17755,
	};
	mavlink_huch_imu_raw_adc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.xacc = packet_in.xacc;
        	packet1.yacc = packet_in.yacc;
        	packet1.zacc = packet_in.zacc;
        	packet1.xgyro = packet_in.xgyro;
        	packet1.ygyro = packet_in.ygyro;
        	packet1.zgyro = packet_in.zgyro;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_imu_raw_adc_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_imu_raw_adc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_imu_raw_adc_pack(system_id, component_id, &msg , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro );
	mavlink_msg_huch_imu_raw_adc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_imu_raw_adc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro );
	mavlink_msg_huch_imu_raw_adc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_imu_raw_adc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_imu_raw_adc_send(MAVLINK_COMM_1 , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro );
	mavlink_msg_huch_imu_raw_adc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_mk_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_mk_imu_t packet_in = {
		93372036854775807ULL,
	17651,
	17755,
	17859,
	17963,
	18067,
	18171,
	};
	mavlink_huch_mk_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.xacc = packet_in.xacc;
        	packet1.yacc = packet_in.yacc;
        	packet1.zacc = packet_in.zacc;
        	packet1.xgyro = packet_in.xgyro;
        	packet1.ygyro = packet_in.ygyro;
        	packet1.zgyro = packet_in.zgyro;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_mk_imu_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_mk_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_mk_imu_pack(system_id, component_id, &msg , packet1.usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro );
	mavlink_msg_huch_mk_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_mk_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro );
	mavlink_msg_huch_mk_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_mk_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_mk_imu_send(MAVLINK_COMM_1 , packet1.usec , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro );
	mavlink_msg_huch_mk_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_raw_pressure(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_raw_pressure_t packet_in = {
		93372036854775807ULL,
	963497880,
	};
	mavlink_huch_raw_pressure_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.pressure = packet_in.pressure;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_raw_pressure_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_raw_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_raw_pressure_pack(system_id, component_id, &msg , packet1.usec , packet1.pressure );
	mavlink_msg_huch_raw_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_raw_pressure_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.pressure );
	mavlink_msg_huch_raw_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_raw_pressure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_raw_pressure_send(MAVLINK_COMM_1 , packet1.usec , packet1.pressure );
	mavlink_msg_huch_raw_pressure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_altitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_altitude_t packet_in = {
		93372036854775807ULL,
	73.0,
	};
	mavlink_huch_altitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.altitude = packet_in.altitude;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_altitude_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_altitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_altitude_pack(system_id, component_id, &msg , packet1.usec , packet1.altitude );
	mavlink_msg_huch_altitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_altitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.altitude );
	mavlink_msg_huch_altitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_altitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_altitude_send(MAVLINK_COMM_1 , packet1.usec , packet1.altitude );
	mavlink_msg_huch_altitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_temperature(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_temperature_t packet_in = {
		93372036854775807ULL,
	963497880,
	};
	mavlink_huch_temperature_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.temperature = packet_in.temperature;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_temperature_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_temperature_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_temperature_pack(system_id, component_id, &msg , packet1.usec , packet1.temperature );
	mavlink_msg_huch_temperature_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_temperature_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.temperature );
	mavlink_msg_huch_temperature_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_temperature_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_temperature_send(MAVLINK_COMM_1 , packet1.usec , packet1.temperature );
	mavlink_msg_huch_temperature_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_magnetic_kompass(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_magnetic_kompass_t packet_in = {
		93372036854775807ULL,
	73.0,
	101.0,
	129.0,
	};
	mavlink_huch_magnetic_kompass_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.data_x = packet_in.data_x;
        	packet1.data_y = packet_in.data_y;
        	packet1.data_z = packet_in.data_z;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_magnetic_kompass_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_magnetic_kompass_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_magnetic_kompass_pack(system_id, component_id, &msg , packet1.usec , packet1.data_x , packet1.data_y , packet1.data_z );
	mavlink_msg_huch_magnetic_kompass_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_magnetic_kompass_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.data_x , packet1.data_y , packet1.data_z );
	mavlink_msg_huch_magnetic_kompass_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_magnetic_kompass_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_magnetic_kompass_send(MAVLINK_COMM_1 , packet1.usec , packet1.data_x , packet1.data_y , packet1.data_z );
	mavlink_msg_huch_magnetic_kompass_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_distance(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_distance_t packet_in = {
		93372036854775807ULL,
	73.0,
	};
	mavlink_huch_distance_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.distance = packet_in.distance;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_distance_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_distance_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_distance_pack(system_id, component_id, &msg , packet1.usec , packet1.distance );
	mavlink_msg_huch_distance_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_distance_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.distance );
	mavlink_msg_huch_distance_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_distance_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_distance_send(MAVLINK_COMM_1 , packet1.usec , packet1.distance );
	mavlink_msg_huch_distance_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_visual_navigation(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_visual_navigation_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
	18899,
	19003,
	};
	mavlink_huch_visual_navigation_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.alt_velocity = packet_in.alt_velocity;
        	packet1.alt_absolute = packet_in.alt_absolute;
        	packet1.home_beta = packet_in.home_beta;
        	packet1.home_distance = packet_in.home_distance;
        	packet1.visual_compass = packet_in.visual_compass;
        	packet1.ego_beta = packet_in.ego_beta;
        	packet1.ego_speed = packet_in.ego_speed;
        	packet1.debug = packet_in.debug;
        	packet1.keypoints = packet_in.keypoints;
        	packet1.error = packet_in.error;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_visual_navigation_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_visual_navigation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_visual_navigation_pack(system_id, component_id, &msg , packet1.alt_velocity , packet1.alt_absolute , packet1.home_beta , packet1.home_distance , packet1.visual_compass , packet1.ego_beta , packet1.ego_speed , packet1.keypoints , packet1.error , packet1.debug );
	mavlink_msg_huch_visual_navigation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_visual_navigation_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.alt_velocity , packet1.alt_absolute , packet1.home_beta , packet1.home_distance , packet1.visual_compass , packet1.ego_beta , packet1.ego_speed , packet1.keypoints , packet1.error , packet1.debug );
	mavlink_msg_huch_visual_navigation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_visual_navigation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_visual_navigation_send(MAVLINK_COMM_1 , packet1.alt_velocity , packet1.alt_absolute , packet1.home_beta , packet1.home_distance , packet1.visual_compass , packet1.ego_beta , packet1.ego_speed , packet1.keypoints , packet1.error , packet1.debug );
	mavlink_msg_huch_visual_navigation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_analog(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_analog_t packet_in = {
		93372036854775807ULL,
	963497880,
	};
	mavlink_huch_analog_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.analog = packet_in.analog;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_analog_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_analog_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_analog_pack(system_id, component_id, &msg , packet1.usec , packet1.analog );
	mavlink_msg_huch_analog_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_analog_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.analog );
	mavlink_msg_huch_analog_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_analog_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_analog_send(MAVLINK_COMM_1 , packet1.usec , packet1.analog );
	mavlink_msg_huch_analog_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_hc_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_hc_raw_t packet_in = {
		93372036854775807ULL,
	963497880,
	963498088,
	963498296,
	963498504,
	963498712,
	};
	mavlink_huch_hc_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.raw0 = packet_in.raw0;
        	packet1.raw1 = packet_in.raw1;
        	packet1.raw2 = packet_in.raw2;
        	packet1.raw3 = packet_in.raw3;
        	packet1.raw4 = packet_in.raw4;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_hc_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_hc_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_hc_raw_pack(system_id, component_id, &msg , packet1.usec , packet1.raw0 , packet1.raw1 , packet1.raw2 , packet1.raw3 , packet1.raw4 );
	mavlink_msg_huch_hc_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_hc_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.raw0 , packet1.raw1 , packet1.raw2 , packet1.raw3 , packet1.raw4 );
	mavlink_msg_huch_hc_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_hc_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_hc_raw_send(MAVLINK_COMM_1 , packet1.usec , packet1.raw0 , packet1.raw1 , packet1.raw2 , packet1.raw3 , packet1.raw4 );
	mavlink_msg_huch_hc_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_sensor_array(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_sensor_array_t packet_in = {
		93372036854775807ULL,
	{ 179.0, 180.0, 181.0, 182.0, 183.0, 184.0, 185.0, 186.0, 187.0, 188.0, 189.0, 190.0, 191.0, 192.0, 193.0, 194.0 },
	};
	mavlink_huch_sensor_array_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        
        	mav_array_memcpy(packet1.data, packet_in.data, sizeof(double)*16);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_sensor_array_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_sensor_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_sensor_array_pack(system_id, component_id, &msg , packet1.usec , packet1.data );
	mavlink_msg_huch_sensor_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_sensor_array_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.data );
	mavlink_msg_huch_sensor_array_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_sensor_array_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_sensor_array_send(MAVLINK_COMM_1 , packet1.usec , packet1.data );
	mavlink_msg_huch_sensor_array_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_sim_ctrl(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_sim_ctrl_t packet_in = {
		93372036854775807ULL,
	73.0,
	17859,
	};
	mavlink_huch_sim_ctrl_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.arg = packet_in.arg;
        	packet1.cmd = packet_in.cmd;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_sim_ctrl_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_sim_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_sim_ctrl_pack(system_id, component_id, &msg , packet1.usec , packet1.cmd , packet1.arg );
	mavlink_msg_huch_sim_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_sim_ctrl_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.cmd , packet1.arg );
	mavlink_msg_huch_sim_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_sim_ctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_sim_ctrl_send(MAVLINK_COMM_1 , packet1.usec , packet1.cmd , packet1.arg );
	mavlink_msg_huch_sim_ctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_ext_ctrl(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_ext_ctrl_t packet_in = {
		17235,
	17339,
	17443,
	17547,
	29,
	96,
	163,
	};
	mavlink_huch_ext_ctrl_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.thrust = packet_in.thrust;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        	packet1.mask = packet_in.mask;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_ext_ctrl_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_ext_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_ext_ctrl_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.mask , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_huch_ext_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_ext_ctrl_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.mask , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_huch_ext_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_ext_ctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_ext_ctrl_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.mask , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust );
	mavlink_msg_huch_ext_ctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_lin_sen_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_lin_sen_raw_t packet_in = {
		963497464,
	{ 17443, 17444, 17445, 17446, 17447, 17448, 17449, 17450, 17451, 17452, 17453, 17454, 17455, 17456, 17457, 17458, 17459, 17460, 17461, 17462, 17463, 17464, 17465, 17466, 17467, 17468, 17469, 17470, 17471, 17472, 17473, 17474, 17475, 17476, 17477, 17478, 17479, 17480, 17481, 17482, 17483, 17484, 17485, 17486, 17487, 17488, 17489, 17490, 17491, 17492, 17493, 17494, 17495, 17496, 17497, 17498, 17499, 17500, 17501, 17502, 17503, 17504, 17505, 17506, 17507, 17508, 17509, 17510, 17511, 17512, 17513, 17514, 17515, 17516, 17517, 17518, 17519, 17520, 17521, 17522, 17523, 17524, 17525, 17526, 17527, 17528, 17529, 17530, 17531, 17532, 17533, 17534, 17535, 17536, 17537, 17538 },
	};
	mavlink_huch_lin_sen_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.msec = packet_in.msec;
        
        	mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint16_t)*96);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_lin_sen_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_lin_sen_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_lin_sen_raw_pack(system_id, component_id, &msg , packet1.msec , packet1.data );
	mavlink_msg_huch_lin_sen_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_lin_sen_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.msec , packet1.data );
	mavlink_msg_huch_lin_sen_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_lin_sen_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_lin_sen_raw_send(MAVLINK_COMM_1 , packet1.msec , packet1.data );
	mavlink_msg_huch_lin_sen_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_lin_sen_ctrl(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_lin_sen_ctrl_t packet_in = {
		963497464,
	17443,
	151,
	};
	mavlink_huch_lin_sen_ctrl_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.msec = packet_in.msec;
        	packet1.arg = packet_in.arg;
        	packet1.cmd = packet_in.cmd;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_lin_sen_ctrl_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_lin_sen_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_lin_sen_ctrl_pack(system_id, component_id, &msg , packet1.msec , packet1.cmd , packet1.arg );
	mavlink_msg_huch_lin_sen_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_lin_sen_ctrl_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.msec , packet1.cmd , packet1.arg );
	mavlink_msg_huch_lin_sen_ctrl_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_lin_sen_ctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_lin_sen_ctrl_send(MAVLINK_COMM_1 , packet1.msec , packet1.cmd , packet1.arg );
	mavlink_msg_huch_lin_sen_ctrl_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_visual_flow(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_visual_flow_t packet_in = {
		93372036854775807ULL,
	73.0,
	101.0,
	129.0,
	157.0,
	};
	mavlink_huch_visual_flow_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.u = packet_in.u;
        	packet1.v = packet_in.v;
        	packet1.u_i = packet_in.u_i;
        	packet1.v_i = packet_in.v_i;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_visual_flow_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_visual_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_visual_flow_pack(system_id, component_id, &msg , packet1.usec , packet1.u , packet1.v , packet1.u_i , packet1.v_i );
	mavlink_msg_huch_visual_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_visual_flow_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.u , packet1.v , packet1.u_i , packet1.v_i );
	mavlink_msg_huch_visual_flow_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_visual_flow_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_visual_flow_send(MAVLINK_COMM_1 , packet1.usec , packet1.u , packet1.v , packet1.u_i , packet1.v_i );
	mavlink_msg_huch_visual_flow_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_potibox(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_potibox_t packet_in = {
		93372036854775807ULL,
	{ 17651, 17652, 17653, 17654, 17655, 17656 },
	{ 18275, 18276, 18277, 18278 },
	};
	mavlink_huch_potibox_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        
        	mav_array_memcpy(packet1.a, packet_in.a, sizeof(int16_t)*6);
        	mav_array_memcpy(packet1.d, packet_in.d, sizeof(int16_t)*4);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_potibox_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_potibox_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_potibox_pack(system_id, component_id, &msg , packet1.usec , packet1.a , packet1.d );
	mavlink_msg_huch_potibox_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_potibox_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.a , packet1.d );
	mavlink_msg_huch_potibox_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_potibox_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_potibox_send(MAVLINK_COMM_1 , packet1.usec , packet1.a , packet1.d );
	mavlink_msg_huch_potibox_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_attitude_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_attitude_control_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	53,
	120,
	};
	mavlink_huch_attitude_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.thrust = packet_in.thrust;
        	packet1.target = packet_in.target;
        	packet1.mask = packet_in.mask;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_attitude_control_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_attitude_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_attitude_control_pack(system_id, component_id, &msg , packet1.target , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust , packet1.mask );
	mavlink_msg_huch_attitude_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_attitude_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust , packet1.mask );
	mavlink_msg_huch_attitude_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_attitude_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_attitude_control_send(MAVLINK_COMM_1 , packet1.target , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust , packet1.mask );
	mavlink_msg_huch_attitude_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_generic_channel(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_generic_channel_t packet_in = {
		93372036854775807ULL,
	73.0,
	17859,
	};
	mavlink_huch_generic_channel_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.usec = packet_in.usec;
        	packet1.value = packet_in.value;
        	packet1.index = packet_in.index;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_generic_channel_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_generic_channel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_generic_channel_pack(system_id, component_id, &msg , packet1.usec , packet1.index , packet1.value );
	mavlink_msg_huch_generic_channel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_generic_channel_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.usec , packet1.index , packet1.value );
	mavlink_msg_huch_generic_channel_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_generic_channel_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_generic_channel_send(MAVLINK_COMM_1 , packet1.usec , packet1.index , packet1.value );
	mavlink_msg_huch_generic_channel_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_action(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_action_t packet_in = {
		5,
	72,
	139,
	};
	mavlink_huch_action_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.target = packet_in.target;
        	packet1.target_component = packet_in.target_component;
        	packet1.action = packet_in.action;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_action_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_action_pack(system_id, component_id, &msg , packet1.target , packet1.target_component , packet1.action );
	mavlink_msg_huch_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_action_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.target_component , packet1.action );
	mavlink_msg_huch_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_action_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_action_send(MAVLINK_COMM_1 , packet1.target , packet1.target_component , packet1.action );
	mavlink_msg_huch_action_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_cam_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_cam_state_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
	101,
	168,
	235,
	46,
	};
	mavlink_huch_cam_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.hist1 = packet_in.hist1;
        	packet1.hist2 = packet_in.hist2;
        	packet1.hist3 = packet_in.hist3;
        	packet1.hist4 = packet_in.hist4;
        	packet1.hist5 = packet_in.hist5;
        	packet1.hist6 = packet_in.hist6;
        	packet1.hist7 = packet_in.hist7;
        	packet1.hist8 = packet_in.hist8;
        	packet1.cam_index = packet_in.cam_index;
        	packet1.exposure = packet_in.exposure;
        	packet1.contrast = packet_in.contrast;
        	packet1.gain = packet_in.gain;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_cam_state_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_cam_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_cam_state_pack(system_id, component_id, &msg , packet1.cam_index , packet1.exposure , packet1.contrast , packet1.gain , packet1.hist1 , packet1.hist2 , packet1.hist3 , packet1.hist4 , packet1.hist5 , packet1.hist6 , packet1.hist7 , packet1.hist8 );
	mavlink_msg_huch_cam_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_cam_state_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cam_index , packet1.exposure , packet1.contrast , packet1.gain , packet1.hist1 , packet1.hist2 , packet1.hist3 , packet1.hist4 , packet1.hist5 , packet1.hist6 , packet1.hist7 , packet1.hist8 );
	mavlink_msg_huch_cam_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_cam_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_cam_state_send(MAVLINK_COMM_1 , packet1.cam_index , packet1.exposure , packet1.contrast , packet1.gain , packet1.hist1 , packet1.hist2 , packet1.hist3 , packet1.hist4 , packet1.hist5 , packet1.hist6 , packet1.hist7 , packet1.hist8 );
	mavlink_msg_huch_cam_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch_cam_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_huch_cam_cmd_t packet_in = {
		5,
	72,
	139,
	206,
	};
	mavlink_huch_cam_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.cam_index = packet_in.cam_index;
        	packet1.exposure = packet_in.exposure;
        	packet1.contrast = packet_in.contrast;
        	packet1.gain = packet_in.gain;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_cam_cmd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_huch_cam_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_cam_cmd_pack(system_id, component_id, &msg , packet1.cam_index , packet1.exposure , packet1.contrast , packet1.gain );
	mavlink_msg_huch_cam_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_cam_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cam_index , packet1.exposure , packet1.contrast , packet1.gain );
	mavlink_msg_huch_cam_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_huch_cam_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_huch_cam_cmd_send(MAVLINK_COMM_1 , packet1.cam_index , packet1.exposure , packet1.contrast , packet1.gain );
	mavlink_msg_huch_cam_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_huch(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_mk_extern_control(system_id, component_id, last_msg);
	mavlink_test_huch_exp_ctrl(system_id, component_id, last_msg);
	mavlink_test_huch_attitude(system_id, component_id, last_msg);
	mavlink_test_huch_fc_altitude(system_id, component_id, last_msg);
	mavlink_test_huch_ranger(system_id, component_id, last_msg);
	mavlink_test_huch_exp_ctrl_rx(system_id, component_id, last_msg);
	mavlink_test_mk_fc_status(system_id, component_id, last_msg);
	mavlink_test_huch_ctrl_hover_state(system_id, component_id, last_msg);
	mavlink_test_huch_imu_raw_adc(system_id, component_id, last_msg);
	mavlink_test_huch_mk_imu(system_id, component_id, last_msg);
	mavlink_test_huch_raw_pressure(system_id, component_id, last_msg);
	mavlink_test_huch_altitude(system_id, component_id, last_msg);
	mavlink_test_huch_temperature(system_id, component_id, last_msg);
	mavlink_test_huch_magnetic_kompass(system_id, component_id, last_msg);
	mavlink_test_huch_distance(system_id, component_id, last_msg);
	mavlink_test_huch_visual_navigation(system_id, component_id, last_msg);
	mavlink_test_huch_analog(system_id, component_id, last_msg);
	mavlink_test_huch_hc_raw(system_id, component_id, last_msg);
	mavlink_test_huch_sensor_array(system_id, component_id, last_msg);
	mavlink_test_huch_sim_ctrl(system_id, component_id, last_msg);
	mavlink_test_huch_ext_ctrl(system_id, component_id, last_msg);
	mavlink_test_huch_lin_sen_raw(system_id, component_id, last_msg);
	mavlink_test_huch_lin_sen_ctrl(system_id, component_id, last_msg);
	mavlink_test_huch_visual_flow(system_id, component_id, last_msg);
	mavlink_test_huch_potibox(system_id, component_id, last_msg);
	mavlink_test_huch_attitude_control(system_id, component_id, last_msg);
	mavlink_test_huch_generic_channel(system_id, component_id, last_msg);
	mavlink_test_huch_action(system_id, component_id, last_msg);
	mavlink_test_huch_cam_state(system_id, component_id, last_msg);
	mavlink_test_huch_cam_cmd(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // HUCH_TESTSUITE_H
