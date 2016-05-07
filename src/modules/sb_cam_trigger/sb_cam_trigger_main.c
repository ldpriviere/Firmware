/**
 * @file sb_cam_trigger_main.c
 * Sunbirds camera triggering application
 */


/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/prctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <mathlib/mathlib.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/sb_cam_footprint.h>
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <board_config.h>

#define TRIGGER_PIN_DEFAULT 1

__EXPORT int sb_cam_trigger_main(int argc, char *argv[]);

int sb_cam_trigger_thread_main(int argc, char *argv[]);
void camera_engage();
void camera_disengage();
void camera_trigger(int trigger);
void doTrigger();
void parameters_update();
int open_log_file();
bool file_exist(const char *filename);

static const unsigned MAX_NO_LOGFILE = 999;		/**< Maximum number of log files */
static bool thread_should_exit = false;			/**< Daemon exit flag */
static bool thread_running = false;				/**< Daemon status flag */
static int deamon_task;							/**< Handle of deamon task / thread */

int trigger_ono = 1;
param_t _p_trigger_ono;

float trigger_period = 1.0f;
param_t _p_trigger_period;

int trigger_pin = 1;
param_t _p_trigger_pin;


int trigger_polarity = 0;
param_t _p_trigger_polarity;

float activation_time = 0;
param_t _p_activation_time;

uint64_t last_engage_time = 0;
int isTriggering = 0;

int 	_params_sub = -1; 		/**< notification of parameter updates */
int		_home_pos_sub;			/**< home position subscription */
int		_global_pos_sub;		/**< global position subscription */
int		_gps_pos_sub;			/**< GPS position subscription */

struct home_position_s				_home_pos;			/**< home position*/
struct vehicle_global_position_s	_global_pos;		/**< global vehicle position */
struct vehicle_gps_position_s 		_gps_pos;				/**< GPS position  */

struct sb_cam_footprint_s _sb_cam_footprint;
orb_advert_t _sb_cam_footprint_pub;

/* GPS time, used for log files naming */
static uint64_t gps_time = 0;

/*Log file descriptor*/
int log_fd = -1;

static uint32_t _gpios[6] = {
	GPIO_GPIO0_OUTPUT,
	GPIO_GPIO1_OUTPUT,
	GPIO_GPIO2_OUTPUT,
	GPIO_GPIO3_OUTPUT,
	GPIO_GPIO4_OUTPUT,
	GPIO_GPIO5_OUTPUT
};

void doTrigger()
{
	uint64_t time = hrt_absolute_time();
	if(isTriggering)
	{
		if(time > (last_engage_time + activation_time*1000000))
		{
			camera_disengage();
		}
	}
	else
	{
		if(time > (last_engage_time + trigger_period*1000000))
		{
			camera_engage();
		}
	}
}

void camera_engage()
{
	int n;
	char log_line_buffer[100];

	last_engage_time = hrt_absolute_time();
	camera_trigger(trigger_polarity);
	isTriggering = 1;

	_sb_cam_footprint.timestamp = last_engage_time;
	_sb_cam_footprint.alt = _global_pos.alt - _home_pos.alt;
	_sb_cam_footprint.lat = _global_pos.lat;
	_sb_cam_footprint.lon = _global_pos.lon;
	_sb_cam_footprint.yaw = _global_pos.yaw;

	orb_publish(ORB_ID(sb_cam_footprint), _sb_cam_footprint_pub, &_sb_cam_footprint);

	if(log_fd >= 0)
	{
		snprintf(log_line_buffer, sizeof(log_line_buffer), "%d;%f;%f;%f;%f\n",
				last_engage_time,
				(double)_sb_cam_footprint.alt,
				_sb_cam_footprint.lat,
				_sb_cam_footprint.lon,
				(double)_sb_cam_footprint.yaw);

		fsync(log_fd);
		n = write(log_fd, log_line_buffer, strlen(log_line_buffer));

		if (n < 0) {
			err(1, "error writing log file");
		}
	}
}

void camera_disengage()
{
	camera_trigger(!trigger_polarity);
	isTriggering = 0;
}

void camera_trigger(int trigger)
{
	stm32_gpiowrite(_gpios[trigger_pin - 1], trigger);
}

void parameters_update()
{
	param_get(_p_trigger_ono, &trigger_ono);
	param_get(_p_trigger_period, &trigger_period);
	param_get(_p_trigger_pin, &trigger_pin);
	param_get(_p_trigger_polarity, &trigger_polarity);
	param_get(_p_activation_time, &activation_time);
}

/* Main Thread */
int sb_cam_trigger_thread_main(int argc, char *argv[])
{

	_p_trigger_ono = param_find("SB_CAM_ONOFF");
	_p_trigger_period = param_find("SB_CAM_PERIOD");
	_p_trigger_pin = param_find("SB_CAM_PIN");
	_p_trigger_polarity = param_find("SB_CAM_POL");
	_p_activation_time = param_find("SB_CAM_ATIME");

	parameters_update();

	stm32_configgpio(_gpios[trigger_pin-1]);
	stm32_gpiowrite(_gpios[trigger_pin-1], !trigger_polarity);

	_sb_cam_footprint_pub = orb_advertise(ORB_ID(sb_cam_footprint), &_sb_cam_footprint);

	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));
	_gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));


	/* wakeup source(s) */
	struct pollfd fds[4];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _global_pos_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _home_pos_sub;
	fds[2].events = POLLIN;
	fds[3].fd = _gps_pos_sub;
	fds[3].events = POLLIN;

	if (!orb_copy(ORB_ID(vehicle_gps_position), _gps_pos_sub, &_gps_pos)) {
		gps_time = _gps_pos.time_gps_usec;
	}

	log_fd = open_log_file();

	/* ==================== MAIN loop ======================*/
	while (!thread_should_exit) {

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("[sb_cam_trigger] poll error %d", pret);
			continue;
		}

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* global position updated */
		if (fds[1].revents & POLLIN) {
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
		}

		/* home position updated */
		if (fds[2].revents & POLLIN) {
			orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
		}

		/* GPS position updated */
		if (fds[3].revents & POLLIN) {
			orb_copy(ORB_ID(vehicle_gps_position), _gps_pos_sub, &_gps_pos);
			gps_time = _gps_pos.time_gps_usec;
		}

		if(trigger_ono)
		{
			doTrigger();
		}

	}

	printf("[sb_cam_trigger] exiting.\n");
	thread_running = false;

	fflush(stdout);
	fsync(log_fd);
	close(log_fd);

	return 0;
}

static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: sb_cam_trigger {start|stop|status}\n\n");
	exit(1);
}

int open_log_file()
{
	/* string to hold the path to the log */
	char log_file_name[32] = "";
	char log_file_path[64] = "";

	if (gps_time != 0) {
		/* use GPS time for log file naming, e.g. /fs/microsd/2014-01-19/19_37_52.bin */
		time_t gps_time_sec = gps_time / 1000000;
		struct tm t;
		gmtime_r(&gps_time_sec, &t);
		strftime(log_file_name, sizeof(log_file_name), "CAM_%H_%M_%S.txt", &t);
		snprintf(log_file_path, sizeof(log_file_path), "%s/%s", "/fs/microsd/log", log_file_name);

	} else {
		uint16_t file_number = 1; // start with file log001

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {
			/* format log file path: e.g. /fs/microsd/sess001/log001.bin */
			snprintf(log_file_name, sizeof(log_file_name), "CAM_%03u.bin", file_number);
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

/**
 * @return 0 if file exists
 */
bool file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}


int sb_cam_trigger_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("sb_cam_trigger already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("sb_cam_trigger",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 20,
					 2048,
					 sb_cam_trigger_thread_main,
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
			printf("sb_cam_trigger is running\n");

		} else {
			printf("sb_cam_trigger not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
