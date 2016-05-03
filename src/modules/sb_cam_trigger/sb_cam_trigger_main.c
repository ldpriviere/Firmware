/**
 * @file sb_cam_trigger_main.c
 * Sunbirds camera triggering application
 */


/****************************************************************************
 * Included Files
 ****************************************************************************/
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

static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

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

int 		_params_sub = -1; /**< notification of parameter updates */

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
	last_engage_time = hrt_absolute_time();
	camera_trigger(trigger_polarity);
	isTriggering = 1;
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


	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	/* wakeup source(s) */
	struct pollfd fds[1];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;

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

		if(trigger_ono)
		{
			doTrigger();
		}

	}

	printf("[sb_cam_trigger] exiting.\n");
	thread_running = false;

	fflush(stdout);

	return 0;
}

static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: sb_cam_trigger {start|stop|status}\n\n");
	exit(1);
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
