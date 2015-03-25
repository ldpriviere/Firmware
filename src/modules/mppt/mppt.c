/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
 
/**
 * @file mppt.c
 * Maximum power point tracking application for PX4 autopilot.
 */
 
 
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <string.h>
#include <poll.h>
#include <stdbool.h>
#include <arch/board/board.h>
#include <systemlib/systemlib.h>
#include <visibility.h>
#include <drivers/drv_hrt.h>
#include <mavlink/mavlink_log.h>

#include "../../NuttX/nuttx/include/fcntl.h"
#include "../../NuttX/nuttx/include/sys/types.h"

#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>

#include "mpptLink.h"

#include <math.h>
#include <float.h>


#define MAVLINK_OPEN_INTERVAL 50000

__EXPORT int mppt_main(int argc, char *argv[]);

void handleEndianness(MpptFrame *frame);
int mppt_app_thread_main(int argc, char *argv[]);
static void usage(const char *reason);


/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static int mavlink_fd = 0;

void handleEndianness(MpptFrame *frame)
{
	unsigned char buffer[4];
	unsigned char temp;

	memcpy(buffer, &frame->batteryVoltage, sizeof(float));
	temp = buffer[3];
	buffer[3] = buffer[2];
	buffer[2] = buffer[1];
	buffer[1] = buffer[0];
	buffer[0] = temp;
	memcpy(&frame->batteryVoltage, buffer, sizeof(float));

	memcpy(buffer, &frame->solarCurrent, sizeof(float));
	temp = buffer[3];
	buffer[3] = buffer[2];
	buffer[2] = buffer[1];
	buffer[1] = buffer[0];
	buffer[0] = temp;
	memcpy(&frame->solarCurrent, buffer, sizeof(float));
}

/* Main Thread */
int mppt_app_thread_main(int argc, char *argv[])
{
	const int timeout = 3000;
	uint8_t buf[20];
//	uint8_t buf2[10];
	MpptFrame frame;
	
	/* ======================  UART4 Configuration ======================== */

	/* CAUTION : UART4 is TTYS6 device */
	int uart_fd = open("/dev/ttyS6", O_RDWR | O_NOCTTY);
	
	if (uart_fd < 0) {
		printf("ERROR opening UART4, aborting..\n");
		return uart_fd;
	}
	
	struct termios uart_config;
	int termios_state = 0;
	
	if ((termios_state = tcgetattr(uart_fd, &uart_config)) < 0) {
		printf("ERROR getting termios config for UART4: %d\n", termios_state);
		exit(termios_state);
	}
	
	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* Set baud rate */
	if (cfsetispeed(&uart_config, B9600) < 0 || cfsetospeed(&uart_config, B9600) < 0) {
		printf("ERROR setting termios config for UART4: %d\n", termios_state);
		exit(ERROR);
	}
	
	if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
		printf("ERROR setting termios config for UART4\n");
		exit(termios_state);
	}
	
	/* ==================== MAV LOG initialization ======================*/

	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	/* ==================== uORB messages initialization ======================*/

	struct battery_status_s _battery_status;
	orb_advert_t _battery_pub = orb_advertise(ORB_ID(battery_status), &_battery_status);

	/* ==================== POLL initialization ======================*/
	struct pollfd fds[1];
	fds[0].fd = uart_fd;
	fds[0].events = POLLIN;

	ssize_t nread = 0;
	
	/* ==================== MPPT frame messages initialization ======================*/

	initMpptFrame(&frame);
	

	/* ==================== MAIN loop ======================*/

	unsigned counter = 0;
	unsigned int counter2 = 0;
	char bufferPrintf[50];
	while (!thread_should_exit) {
	
		if (mavlink_fd < 0 && counter % (1000000 / MAVLINK_OPEN_INTERVAL) == 0) {
			/* try to open the mavlink log device every once in a while */
			mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
		}

		int pollResult = 0;
		if ((pollResult = poll(fds, 1, timeout)) > 0) {

			/* non-blocking read. read may return negative values */
			if ((nread = read(uart_fd, buf, sizeof(buf))) < (ssize_t)sizeof(buf)) {
				/* to avoid reading very small chunks wait for data before reading */
				usleep(1000);
			}

			for(int i=0; i < nread; i++)
			{
//				buf2[counter2] = buf[i];

				if(parseMpptFrame(&buf[i], &frame))
				{
					
					hrt_abstime timestamp = hrt_absolute_time();

					/* A complete message is available */
					
					/*Handle PIC Endianness*/
					handleEndianness(&frame);

					//mavlink_log_info(mavlink_fd, "Message complete \n");

					//sprintf(bufferPrintf, "Buffer : %x %x %x %x %x %x %x %x %x %x\n", buf2[0],buf2[1],buf2[2],buf2[3],buf2[4],buf2[5],buf2[6],buf2[7],buf2[8],buf2[9]);
					//mavlink_log_info(mavlink_fd, bufferPrintf);

					//sprintf(bufferPrintf, "voltage : %f\n", (double)frame.batteryVoltage);
					//mavlink_log_info(mavlink_fd, bufferPrintf);

					//sprintf(bufferPrintf, "current : %f\n", (double)frame.solarCurrent);
					//mavlink_log_info(mavlink_fd, bufferPrintf);

					_battery_status.timestamp = timestamp;
					_battery_status.voltage_v = frame.batteryVoltage;
					_battery_status.voltage_filtered_v = frame.batteryVoltage;
					_battery_status.current_a = frame.solarCurrent;
					_battery_status.discharged_mah = -1;

//					counter2 = 0;
//					for(int j=0; j<10; j++)
//					{
//						buf2[j] = 0;
//					}

					/* Publish message */
					orb_publish(ORB_ID(battery_status), _battery_pub, &_battery_status);
				}
				else
				{
					counter2++;
				}
			}
		
		}
		else
		{
			sprintf(bufferPrintf, "MPPT data lost. Error code : %d \n", pollResult);
			mavlink_log_critical(mavlink_fd, bufferPrintf);
		}

		counter++;
	
	}
	
	close(uart_fd);
	
	printf("[mppt_app] exiting.\n");
	thread_running = false;
	
	fflush(stdout);

	return 0;
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: mppt {start|stop|status}\n\n");
	exit(1);
}

int mppt_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("mppt_app already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("mppt_app",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 20,
					 2048,
					 mppt_app_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		thread_running = true;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("mppt_app is running\n");

		} else {
			printf("mppt_app not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
