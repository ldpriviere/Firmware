/** @file
 *	@brief MAVLink comm protocol testsuite generated from sunbirds.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef SUNBIRDS_TESTSUITE_H
#define SUNBIRDS_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_sunbirds(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_sunbirds(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_mppt_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mppt_status_t packet_in = {
		17235,17339,17443,17547,29,96
    };
	mavlink_mppt_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.mpptTemperature = packet_in.mpptTemperature;
        	packet1.solarCurrent = packet_in.solarCurrent;
        	packet1.totalCurrent = packet_in.totalCurrent;
        	packet1.batteryVoltage = packet_in.batteryVoltage;
        	packet1.dutyCycleMin = packet_in.dutyCycleMin;
        	packet1.dutyCycleMax = packet_in.dutyCycleMax;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mppt_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mppt_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mppt_status_pack(system_id, component_id, &msg , packet1.mpptTemperature , packet1.solarCurrent , packet1.totalCurrent , packet1.batteryVoltage , packet1.dutyCycleMin , packet1.dutyCycleMax );
	mavlink_msg_mppt_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mppt_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mpptTemperature , packet1.solarCurrent , packet1.totalCurrent , packet1.batteryVoltage , packet1.dutyCycleMin , packet1.dutyCycleMax );
	mavlink_msg_mppt_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mppt_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mppt_status_send(MAVLINK_COMM_1 , packet1.mpptTemperature , packet1.solarCurrent , packet1.totalCurrent , packet1.batteryVoltage , packet1.dutyCycleMin , packet1.dutyCycleMax );
	mavlink_msg_mppt_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sb_cam_footprint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_sb_cam_footprint_t packet_in = {
		93372036854775807ULL,963497880,963498088,963498296,963498504
    };
	mavlink_sb_cam_footprint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.alt = packet_in.alt;
        	packet1.yaw = packet_in.yaw;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sb_cam_footprint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_sb_cam_footprint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sb_cam_footprint_pack(system_id, component_id, &msg , packet1.timestamp , packet1.lat , packet1.lon , packet1.alt , packet1.yaw );
	mavlink_msg_sb_cam_footprint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sb_cam_footprint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.lat , packet1.lon , packet1.alt , packet1.yaw );
	mavlink_msg_sb_cam_footprint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_sb_cam_footprint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sb_cam_footprint_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.lat , packet1.lon , packet1.alt , packet1.yaw );
	mavlink_msg_sb_cam_footprint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sunbirds(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_mppt_status(system_id, component_id, last_msg);
	mavlink_test_sb_cam_footprint(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SUNBIRDS_TESTSUITE_H
