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
 * @file sdlogcam.c
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
#include <sys/stat.h>

#include "../../NuttX/nuttx/include/fcntl.h"
#include "../../NuttX/nuttx/include/sys/types.h"

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/sb_cam_footprint.h>

#include <math.h>
#include <float.h>

static const unsigned MAX_NO_LOGFILE = 999;		/**< Maximum number of log files */

__EXPORT int sdlogcam_main(int argc, char *argv[]);

int sdlogcam_app_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
bool file_exist(const char *filename);
int open_log_file();

/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

/* GPS time, used for log files naming */
uint64_t gps_time = 0;
int		_gps_pos_sub;			/**< GPS position subscription */
struct vehicle_gps_position_s 		_gps_pos;				/**< GPS position  */


struct sb_cam_footprint_s _sb_cam_footprint;
int		_sb_cam_footprint_sub;			/**< CAM footprint subscription */

/**
 * @return 0 if file exists
 */
bool file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}

int open_log_file()
{
	/* string to hold the path to the log */
	char log_file_name[32] = "";
	char log_file_path[64] = "";

	if (gps_time != 0) {
		/* use GPS time for log file naming, e.g. /fs/microsd/2014-01-19/CAM_19_37_52.txt */
		time_t gps_time_sec = gps_time / 1000000;
		struct tm t;
		gmtime_r(&gps_time_sec, &t);
		strftime(log_file_name, sizeof(log_file_name), "CAM_%H_%M_%S.txt", &t);
		snprintf(log_file_path, sizeof(log_file_path), "%s/%s", "/fs/microsd/log", log_file_name);

	} else {
		uint16_t file_number = 1; // start with file log001

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {
			/* format log file path: e.g. /fs/microsd/sess001/CAM_001.txt */
			snprintf(log_file_name, sizeof(log_file_name), "CAM_%03u.txt", file_number);
			snprintf(log_file_path, sizeof(log_file_path), "%s/%s", "/fs/microsd/log", log_file_name);

			if (!file_exist(log_file_path)) {
				break;
			}

			file_number++;
		}
	}

	int fd = open(log_file_path, O_CREAT | O_WRONLY | O_DSYNC);

	if (fd < 0) {
		warn("failed opening log: %s", log_file_name);

	} else {
		warnx("log file: %s", log_file_name);
	}

	return fd;
}

/* Main Thread */
int sdlogcam_app_thread_main(int argc, char *argv[])
{

	int log_fd = -1;

	_gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_sb_cam_footprint_sub = orb_subscribe(ORB_ID(sb_cam_footprint));

	if (!orb_copy(ORB_ID(vehicle_gps_position), _gps_pos_sub, &_gps_pos)) {
		gps_time = _gps_pos.time_gps_usec;
	}


	/* ==================== POLL initialization ======================*/
	struct pollfd fds[2];
	fds[0].fd = _sb_cam_footprint_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _gps_pos_sub;
	fds[1].events = POLLIN;

	/* ==================== MAIN loop ======================*/

	log_fd = open_log_file();

	while (!thread_should_exit) {
	
		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);
	
		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("[sb_cam_trigger] poll error %d", pret);
			continue;
		}

		/* Get cam footprint */
		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(sb_cam_footprint), _sb_cam_footprint_sub, &_sb_cam_footprint);

			if(log_fd >= 0)
			{
				dprintf(log_fd, "%llu;%.8f;%.8f;%.8f;%.8f\n",
						_sb_cam_footprint.timestamp,
						(double)_sb_cam_footprint.alt,
						_sb_cam_footprint.lat,
						_sb_cam_footprint.lon,
						(double)_sb_cam_footprint.yaw);

				fsync(log_fd);
			}
		}

		if (fds[1].revents & POLLIN) {
			orb_copy(ORB_ID(vehicle_gps_position), _gps_pos_sub, &_gps_pos);
			//warnx("LOG GPS POS %d", _gps_pos.alt);
		}
	}
	
	fsync(log_fd);
	close(log_fd);

	printf("[sdlogcam_app] exiting.\n");
	thread_running = false;
	
	fflush(stdout);

	return 0;
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: sdlogcam {start|stop|status}\n\n");
	exit(1);
}

int sdlogcam_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("sdlogcam_app already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("sdlogcam_app",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 20,
					 2048,
					 sdlogcam_app_thread_main,
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
			printf("sdlogcam_app is running\n");

		} else {
			printf("sdlogcam_app not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
