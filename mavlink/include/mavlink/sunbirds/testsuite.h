/** @file
 *	@brief MAVLink comm protocol testsuite generated from mppt.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef MPPT_TESTSUITE_H
#define MPPT_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_mppt(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_mppt(system_id, component_id, last_msg);
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

static void mavlink_test_mppt(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_mppt_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MPPT_TESTSUITE_H
