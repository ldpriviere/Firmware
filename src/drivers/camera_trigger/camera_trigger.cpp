/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file camera_trigger.cpp
 *
 * External camera-IMU synchronisation and triggering via FMU auxiliary pins.
 *
 * Support for camera manipulation via PWM signal over servo pins.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 * @author Kelly Steich <kelly.steich@wingtra.com>
 * @author Andreas Bircher <andreas@wingtra.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <mathlib/mathlib.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <sys/stat.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/sb_cam_footprint.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <board_config.h>

#include "interfaces/src/pwm.h"
#include "interfaces/src/relay.h"

#define TRIGGER_PIN_DEFAULT 1
static const unsigned MAX_NO_LOGFILE = 999;		/**< Maximum number of log files */

extern "C" __EXPORT int camera_trigger_main(int argc, char *argv[]);

typedef enum {
	CAMERA_INTERFACE_MODE_NONE = 0,
	CAMERA_INTERFACE_MODE_RELAY,
	CAMERA_INTERFACE_MODE_PWM
} camera_interface_mode_t;

class CameraTrigger
{
public:
	/**
	 * Constructor
	 */
	CameraTrigger(camera_interface_mode_t camera_interface_mode);

	/**
	 * Destructor, also kills task.
	 */
	~CameraTrigger();

	/**
	 * Set the trigger on / off
	 */
	void		control(bool on);

	/**
	 * Trigger just once
	 */
	void		shootOnce();

	/**
	 * Start the task.
	 */
	void		start();

	/**
	 * Stop the task.
	 */
	void		stop();

	/**
	 * Display info.
	 */
	void		info();

	/**
	 * Update parameters
	 */
	void		updateParameters();

	/**
	 * Open Log file
	 */
	int		open_log_file();

	/**
	 * File exist
	 */
	bool file_exist(const char *filename);

private:

	struct hrt_call		_engagecall;
	struct hrt_call		_disengagecall;
	static struct work_s	_work;

	int 			_gpio_fd;

	int				_mode;
	float 			_activation_time;
	float  			_interval;

	int				_mode_shadow = -1;

	float  			_distance;
	uint32_t 		_trigger_seq;
	bool	 		_trigger_enabled;
	math::Vector<2>	_last_shoot_position;
	bool			_valid_position;

	int			_vcommand_sub;
	int			_vlposition_sub;

	orb_advert_t		_trigger_pub;

	param_t _p_mode;
	param_t _p_activation_time;
	param_t _p_interval;
	param_t _p_distance;
	param_t _p_pin;

	struct sb_cam_footprint_s _sb_cam_footprint;
	orb_advert_t _sb_cam_footprint_pub;

	int 	_params_sub = -1; 		/**< notification of parameter updates */
	int		_home_pos_sub;			/**< home position subscription */
	int		_global_pos_sub;		/**< global position subscription */
	int		_gps_pos_sub;			/**< GPS position subscription */

	struct home_position_s				_home_pos;			/**< home position*/
	struct vehicle_global_position_s	_global_pos;		/**< global vehicle position */
	struct vehicle_gps_position_s 		_gps_pos;				/**< GPS position  */

	/* GPS time, used for log files naming */
	uint64_t gps_time = 0;

	/*Log file descriptor*/
	int log_fd = -1;

	uint64_t last_engage_time = 0;

	camera_interface_mode_t _camera_interface_mode;
	CameraInterface *_camera_interface;  ///< instance of camera interface

	/**
	 * Vehicle command handler
	 */
	static void	cycle_trampoline(void *arg);
	/**
	 * Fires trigger
	 */
	static void	engage(void *arg);
	/**
	 * Resets trigger
	 */
	static void	disengage(void *arg);

};

struct work_s CameraTrigger::_work;

namespace camera_trigger
{

CameraTrigger	*g_camera_trigger;
}

CameraTrigger::CameraTrigger(camera_interface_mode_t camera_interface_mode) :
	_engagecall {},
	_disengagecall {},
	_gpio_fd(-1),
	_mode(0),
	_activation_time(0.5f /* ms */),
	_interval(100.0f /* ms */),
	_distance(25.0f /* m */),
	_trigger_seq(0),
	_trigger_enabled(false),
	_last_shoot_position(0.0f, 0.0f),
	_valid_position(false),
	_vcommand_sub(-1),
	_vlposition_sub(-1),
	_trigger_pub(-1),
	_camera_interface_mode(camera_interface_mode),
	_camera_interface(nullptr)
{
	//Initiate Camera interface basedon camera_interface_mode
	if (_camera_interface != nullptr) {
		delete(_camera_interface);
		/* set to zero to ensure parser is not used while not instantiated */
		_camera_interface = nullptr;
	}

	switch (_camera_interface_mode) {
	case CAMERA_INTERFACE_MODE_RELAY:
		_camera_interface = new CameraInterfaceRelay;
		break;

	case CAMERA_INTERFACE_MODE_PWM:
		_camera_interface = new CameraInterfacePWM;
		break;

	default:
		break;
	}

	memset(&_work, 0, sizeof(_work));

	// Parameters
	_p_interval = param_find("TRIG_INTERVAL");
	_p_distance = param_find("TRIG_DISTANCE");
	_p_activation_time = param_find("TRIG_ACT_TIME");
	_p_mode = param_find("TRIG_MODE");


	this->updateParameters();

	_sb_cam_footprint_pub = orb_advertise(ORB_ID(sb_cam_footprint), &_sb_cam_footprint);

	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));
	_gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

	if (!orb_copy(ORB_ID(vehicle_gps_position), _gps_pos_sub, &_gps_pos)) {
		gps_time = _gps_pos.time_gps_usec;
	}

	log_fd = open_log_file();
}

CameraTrigger::~CameraTrigger()
{
	delete(_camera_interface);

	camera_trigger::g_camera_trigger = nullptr;
}

void CameraTrigger::updateParameters()
{
	param_get(_p_activation_time, &_activation_time);
	param_get(_p_interval, &_interval);
	param_get(_p_distance, &_distance);
	param_get(_p_mode, &_mode);
}

/**
 * @return 0 if file exists
 */
bool CameraTrigger::file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}


int CameraTrigger::open_log_file()
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

void
CameraTrigger::control(bool on)
{
	// always execute, even if already on
	// to reset timings if necessary

	warnx("Enter control mode : %i", on);

	if (on) {
		// schedule trigger on and off calls
		hrt_call_every(&_engagecall, 0, (_interval * 1000),
			       (hrt_callout)&CameraTrigger::engage, this);

		// schedule trigger on and off calls
		hrt_call_every(&_disengagecall, 0 + (_activation_time * 1000), (_interval * 1000),
			       (hrt_callout)&CameraTrigger::disengage, this);

	} else {
		// cancel all calls
		hrt_cancel(&_engagecall);
		hrt_cancel(&_disengagecall);
		// ensure that the pin is off
		hrt_call_after(&_disengagecall, 0,
			       (hrt_callout)&CameraTrigger::disengage, this);
	}

	_trigger_enabled = on;
}

void
CameraTrigger::shootOnce()
{
	// schedule trigger on and off calls
	hrt_call_after(&_engagecall, 0,
		       (hrt_callout)&CameraTrigger::engage, this);

	// schedule trigger on and off calls
	hrt_call_after(&_disengagecall, 0 + (_activation_time * 1000),
		       (hrt_callout)&CameraTrigger::disengage, this);
}

void
CameraTrigger::start()
{
	// enable immediate if configured that way
	if (_mode == 2) {
		control(true);
	}

	// start to monitor at high rate for trigger enable command
	work_queue(LPWORK, &_work, (worker_t)&CameraTrigger::cycle_trampoline, this, USEC2TICK(1));

}

void
CameraTrigger::stop()
{
	work_cancel(LPWORK, &_work);
	hrt_cancel(&_engagecall);
	hrt_cancel(&_disengagecall);

	fsync(log_fd);
	close(log_fd);

	if (camera_trigger::g_camera_trigger != nullptr) {
		delete(camera_trigger::g_camera_trigger);
	}
}

void
CameraTrigger::cycle_trampoline(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	/* ================================== CHECK PARAMs ============================= */

	/* only update parameters if they changed */
	bool param_updated;
	orb_check(trig->_params_sub, &param_updated);
	if (param_updated) {
		/* read from param to clear updated flag */
		struct parameter_update_s update;
		orb_copy(ORB_ID(parameter_update), trig->_params_sub, &update);

		/* update parameters from storage */
		trig->updateParameters();
	}

	/* global position updated */
	bool global_position_updated;
	orb_check(trig->_global_pos_sub, &global_position_updated);
	if (global_position_updated) {
		orb_copy(ORB_ID(vehicle_global_position), trig->_global_pos_sub, &(trig->_global_pos));
	}

	/* home position updated */
	bool home_position_updated;
	orb_check(trig->_home_pos_sub, &home_position_updated);
	if (home_position_updated) {
		orb_copy(ORB_ID(home_position), trig->_home_pos_sub, &(trig->_home_pos));
	}

	/* GPS position updated */
	bool gps_position_updated;
	orb_check(trig->_gps_pos_sub, &gps_position_updated);
	if (gps_position_updated) {
		orb_copy(ORB_ID(vehicle_gps_position), trig->_gps_pos_sub, &(trig->_gps_pos));
		trig->gps_time = trig->_gps_pos.time_gps_usec;
	}

	/* ================================== CHECK PARAMs ============================= */

	// while the trigger is inactive it has to be ready
	// to become active instantaneously
	int poll_interval_usec = 5000;

	if(trig->_mode != trig->_mode_shadow)
	{
		if (trig->_mode < 3) {

			if(trig->_mode == 0)
			{
				trig->control(false);
			}
			else if(trig->_mode == 2)
			{
				trig->control(true);
				// while the trigger is active there is no
				// need to poll at a very high rate
				poll_interval_usec = 5000;
			}

		}
		else if(trig->_mode == 3)
		{

			// Set trigger based on covered distance
			if (trig->_vlposition_sub < 0) {
				trig->_vlposition_sub = orb_subscribe(ORB_ID(vehicle_local_position));
			}

			struct vehicle_local_position_s pos;

			orb_copy(ORB_ID(vehicle_local_position), trig->_vlposition_sub, &pos);

			if (pos.xy_valid) {
				// Initialize position if not done yet
				math::Vector<2> current_position(pos.x, pos.y);

				if (!trig->_valid_position) {
					// First time valid position, take first shot
					trig->_last_shoot_position = current_position;
					trig->_valid_position = pos.xy_valid;
					trig->shootOnce();
				}

				// Check that distance threshold is exceeded and the time between last shot is large enough
				if ((trig->_last_shoot_position - current_position).length() >= trig->_distance) {
					trig->shootOnce();
					trig->_last_shoot_position = current_position;
				}
			} else {
				poll_interval_usec = 5000;
			}
		}

		trig->_mode_shadow = trig->_mode;
	}

	work_queue(LPWORK, &_work, (worker_t)&CameraTrigger::cycle_trampoline,
		   camera_trigger::g_camera_trigger, USEC2TICK(poll_interval_usec));
}

void
CameraTrigger::engage(void *arg)
{
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);
//	int n;
//	char log_line_buffer[100];

	trig->last_engage_time = hrt_absolute_time();

	trig->_camera_interface->trigger(true);

	trig->_sb_cam_footprint.timestamp = trig->last_engage_time;
	trig->_sb_cam_footprint.alt = trig->_global_pos.alt - trig->_home_pos.alt;
	trig->_sb_cam_footprint.lat = trig->_global_pos.lat;
	trig->_sb_cam_footprint.lon = trig->_global_pos.lon;
	trig->_sb_cam_footprint.yaw = trig->_global_pos.yaw;

	orb_publish(ORB_ID(sb_cam_footprint), trig->_sb_cam_footprint_pub, &(trig->_sb_cam_footprint));

//	if(trig->log_fd >= 0)
//	{
//		snprintf(log_line_buffer, sizeof(log_line_buffer), "%d;%f;%f;%f;%f\n",
//				trig->last_engage_time,
//				(double)trig->_sb_cam_footprint.alt,
//				trig->_sb_cam_footprint.lat,
//				trig->_sb_cam_footprint.lon,
//				(double)trig->_sb_cam_footprint.yaw);
//
//		fsync(trig->log_fd);
//		n = write(trig->log_fd, log_line_buffer, strlen(log_line_buffer));
//
//		if (n < 0) {
//			err(1, "error writing log file");
//		}
//	}
}

void
CameraTrigger::disengage(void *arg)
{
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	trig->_camera_interface->trigger(false);
}

void
CameraTrigger::info()
{
	warnx("state : %s", _trigger_enabled ? "enabled" : "disabled");
	warnx("mode : %i", _mode);
	warnx("interval : %.2f [ms]", (double)_interval);
	warnx("distance : %.2f [m]", (double)_distance);
	warnx("activation time : %.2f [ms]", (double)_activation_time);
	_camera_interface->info();
}

static void usage()
{
	errx(1, "usage: camera_trigger {start {--relay|--pwm}|stop|info}\n");
}

int camera_trigger_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (camera_trigger::g_camera_trigger != nullptr) {
			errx(0, "already running");
		}

		if (argc >= 3) {
			if (!strcmp(argv[2], "--relay")) {
				camera_trigger::g_camera_trigger = new CameraTrigger(CAMERA_INTERFACE_MODE_RELAY);
				warnx("started with camera interface mode : relay");

			} else if (!strcmp(argv[2], "--pwm")) {
				camera_trigger::g_camera_trigger = new CameraTrigger(CAMERA_INTERFACE_MODE_PWM);
				warnx("started with camera interface mode : pwm");

			} else {
				usage();
				return 0;
			}

		} else {
			camera_trigger::g_camera_trigger = new CameraTrigger(CAMERA_INTERFACE_MODE_RELAY);
			warnx("started with default camera interface mode : relay");
		}

		if (camera_trigger::g_camera_trigger == nullptr) {
			errx(1, "alloc failed");
		}

		camera_trigger::g_camera_trigger->start();

		return 0;
	}

	if (camera_trigger::g_camera_trigger == nullptr) {
		errx(1, "not running");

	} else if (!strcmp(argv[1], "stop")) {
		camera_trigger::g_camera_trigger->stop();

	} else if (!strcmp(argv[1], "info")) {
		camera_trigger::g_camera_trigger->info();

	} else if (!strcmp(argv[1], "enable")) {
		camera_trigger::g_camera_trigger->control(true);

	} else if (!strcmp(argv[1], "disable")) {
		camera_trigger::g_camera_trigger->control(false);

	} else {
		usage();
	}

	return 0;
}

