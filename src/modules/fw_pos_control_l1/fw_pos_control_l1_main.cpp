/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: 	Lorenz Meier
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
 * @file fw_pos_control_l1_main.c
 * Implementation of a generic position controller based on the L1 norm. Outputs a bank / roll
 * angle, equivalent to a lateral motion (for copters and rovers).
 *
 * Original publication for horizontal control class:
 *    S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
 *    Proceedings of the AIAA Guidance, Navigation and Control
 *    Conference, Aug 2004. AIAA-2004-4900.
 *
 * Original implementation for total energy control class:
 *    Paul Riseborough and Andrew Tridgell, 2013 (code in lib/external_lgpl)
 *
 * More details and acknowledgements in the referenced library headers.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/mission_item_triplet.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/navigation_capabilities.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>

#include <ecl/l1/ecl_l1_pos_controller.h>
#include <external_lgpl/tecs/tecs.h>

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_pos_control_l1_main(int argc, char *argv[]);

static int mavlink_fd;

class FixedwingPositionControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingPositionControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~FixedwingPositionControl();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	int		_control_task;			/**< task handle for sensor task */

	int		_global_pos_sub;
	int		_mission_item_triplet_sub;
	int		_att_sub;			/**< vehicle attitude subscription */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_control_mode_sub;			/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_control_sub;		/**< notification of manual control updates */
	int		_accel_sub;			/**< body frame accelerations */

	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint */
	orb_advert_t	_nav_capabilities_pub;		/**< navigation capabilities publication */

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct navigation_capabilities_s		_nav_capabilities;	/**< navigation capabilities */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct airspeed_s				_airspeed;		/**< airspeed */
	struct vehicle_control_mode_s				_control_mode;		/**< vehicle status */
	struct vehicle_global_position_s		_global_pos;		/**< global vehicle position */
	struct mission_item_triplet_s			_mission_item_triplet;		/**< triplet of mission items */
	struct accel_report				_accel;			/**< body frame accelerations */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */

	/** manual control states */
	float		_seatbelt_hold_heading;		/**< heading the system should hold in seatbelt mode */
	float		_loiter_hold_lat;
	float		_loiter_hold_lon;
	float		_loiter_hold_alt;
	bool		_loiter_hold;

	float		_launch_lat;
	float		_launch_lon;
	float		_launch_alt;
	bool		_launch_valid;

	/* land states */
	/* not in non-abort mode for landing yet */
	bool land_noreturn_horizontal;
	bool land_noreturn_vertical;
	bool land_stayonground;
	bool land_motor_lim;
	bool land_onslope;

	float flare_curve_alt_last;
	/* heading hold */
	float target_bearing;

	/* throttle and airspeed states */
	float _airspeed_error;				///< airspeed error to setpoint in m/s
	bool _airspeed_valid;				///< flag if a valid airspeed estimate exists
	uint64_t _airspeed_last_valid;			///< last time airspeed was valid. Used to detect sensor failures
	float _groundspeed_undershoot;			///< ground speed error to min. speed in m/s
	bool _global_pos_valid;				///< global position is valid
	math::Dcm _R_nb;				///< current attitude

	ECL_L1_Pos_Controller				_l1_control;
	TECS						_tecs;

	struct {
		float l1_period;
		float l1_damping;

		float time_const;
		float min_sink_rate;
		float max_sink_rate;
		float max_climb_rate;
		float throttle_damp;
		float integrator_gain;
		float vertical_accel_limit;
		float height_comp_filter_omega;
		float speed_comp_filter_omega;
		float roll_throttle_compensation;
		float speed_weight;
		float pitch_damping;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float pitch_limit_min;
		float pitch_limit_max;
		float roll_limit;
		float throttle_min;
		float throttle_max;
		float throttle_cruise;

		float throttle_land_max;

		float loiter_hold_radius;

		float heightrate_p;
		float speedrate_p;

		float land_slope_angle;
		float land_slope_length;
		float land_H1_virt;
		float land_flare_alt_relative;
		float land_thrust_lim_horizontal_distance;

	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t l1_period;
		param_t l1_damping;

		param_t time_const;
		param_t min_sink_rate;
		param_t max_sink_rate;
		param_t max_climb_rate;
		param_t throttle_damp;
		param_t integrator_gain;
		param_t vertical_accel_limit;
		param_t height_comp_filter_omega;
		param_t speed_comp_filter_omega;
		param_t roll_throttle_compensation;
		param_t speed_weight;
		param_t pitch_damping;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t pitch_limit_min;
		param_t pitch_limit_max;
		param_t roll_limit;
		param_t throttle_min;
		param_t throttle_max;
		param_t throttle_cruise;

		param_t throttle_land_max;

		param_t loiter_hold_radius;

		param_t heightrate_p;
		param_t speedrate_p;

		param_t land_slope_angle;
		param_t land_slope_length;
		param_t land_H1_virt;
		param_t land_flare_alt_relative;
		param_t land_thrust_lim_horizontal_distance;

	}		_parameter_handles;		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle status.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for airspeed updates.
	 */
	bool		vehicle_airspeed_poll();

	/**
	 * Check for position updates.
	 */
	void		vehicle_attitude_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Get Altitude on the landing glide slope
	 */
	float getLandingSlopeAbsoluteAltitude(float wp_distance, float wp_altitude, float landing_slope_angle_rad, float horizontal_displacement);

	/**
	 * Control position.
	 */
	bool		control_position(const math::Vector2f &global_pos, const math::Vector2f &ground_speed,
					 const struct mission_item_triplet_s &_mission_item_triplet);

	float calculate_target_airspeed(float airspeed_demand);
	void calculate_gndspeed_undershoot(const math::Vector2f &current_position, const math::Vector2f &ground_speed, const struct mission_item_triplet_s &mission_item_triplet);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));
};

namespace l1_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

FixedwingPositionControl	*g_control;
}

FixedwingPositionControl::FixedwingPositionControl() :

	_task_should_exit(false),
	_control_task(-1),

/* subscriptions */
	_global_pos_sub(-1),
	_mission_item_triplet_sub(-1),
	_att_sub(-1),
	_airspeed_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),

/* publications */
	_attitude_sp_pub(-1),
	_nav_capabilities_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw l1 control")),
/* states */
	_setpoint_valid(false),
	_loiter_hold(false),
	_airspeed_error(0.0f),
	_airspeed_valid(false),
	_groundspeed_undershoot(0.0f),
	_global_pos_valid(false),
	land_noreturn_horizontal(false),
	land_noreturn_vertical(false),
	land_stayonground(false),
	land_motor_lim(false),
	land_onslope(false),
	flare_curve_alt_last(0.0f)
{

	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	/* safely initialize structs */
	vehicle_attitude_s			_att = {0};
	vehicle_attitude_setpoint_s		_att_sp = {0};
	navigation_capabilities_s		_nav_capabilities = {0};
	manual_control_setpoint_s		_manual = {0};
	airspeed_s				_airspeed = {0};
	vehicle_control_mode_s			_control_mode = {0};
	vehicle_global_position_s		_global_pos = {0};
	mission_item_triplet_s			_mission_item_triplet = {0};
	accel_report				_accel = {0};




	_nav_capabilities.turn_distance = 0.0f;

	_parameter_handles.l1_period = param_find("FW_L1_PERIOD");
	_parameter_handles.l1_damping = param_find("FW_L1_DAMPING");
	_parameter_handles.loiter_hold_radius = param_find("FW_LOITER_R");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.pitch_limit_min = param_find("FW_P_LIM_MIN");
	_parameter_handles.pitch_limit_max = param_find("FW_P_LIM_MAX");
	_parameter_handles.roll_limit = param_find("FW_R_LIM");
	_parameter_handles.throttle_min = param_find("FW_THR_MIN");
	_parameter_handles.throttle_max = param_find("FW_THR_MAX");
	_parameter_handles.throttle_cruise = param_find("FW_THR_CRUISE");
	_parameter_handles.throttle_land_max = param_find("FW_THR_LND_MAX");

	_parameter_handles.land_slope_angle = param_find("FW_LND_ANG");
	_parameter_handles.land_slope_length = param_find("FW_LND_SLLR");
	_parameter_handles.land_H1_virt = param_find("FW_LND_HVIRT");
	_parameter_handles.land_flare_alt_relative = param_find("FW_LND_FLALT");
	_parameter_handles.land_thrust_lim_horizontal_distance = param_find("FW_LND_TLDIST");

	_parameter_handles.time_const = 			param_find("FW_T_TIME_CONST");
	_parameter_handles.min_sink_rate = 			param_find("FW_T_SINK_MIN");
	_parameter_handles.max_sink_rate =			param_find("FW_T_SINK_MAX");
	_parameter_handles.max_climb_rate =			param_find("FW_T_CLMB_MAX");
	_parameter_handles.throttle_damp = 			param_find("FW_T_THR_DAMP");
	_parameter_handles.integrator_gain =			param_find("FW_T_INTEG_GAIN");
	_parameter_handles.vertical_accel_limit =		param_find("FW_T_VERT_ACC");
	_parameter_handles.height_comp_filter_omega =		param_find("FW_T_HGT_OMEGA");
	_parameter_handles.speed_comp_filter_omega =		param_find("FW_T_SPD_OMEGA");
	_parameter_handles.roll_throttle_compensation = 	param_find("FW_T_RLL2THR");
	_parameter_handles.speed_weight = 			param_find("FW_T_SPDWEIGHT");
	_parameter_handles.pitch_damping = 			param_find("FW_T_PTCH_DAMP");
	_parameter_handles.heightrate_p =			param_find("FW_T_HRATE_P");
	_parameter_handles.speedrate_p =			param_find("FW_T_SRATE_P");

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingPositionControl::~FixedwingPositionControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	l1_control::g_control = nullptr;
}

int
FixedwingPositionControl::parameters_update()
{

	/* L1 control parameters */
	param_get(_parameter_handles.l1_damping, &(_parameters.l1_damping));
	param_get(_parameter_handles.l1_period, &(_parameters.l1_period));
	param_get(_parameter_handles.loiter_hold_radius, &(_parameters.loiter_hold_radius));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.pitch_limit_min, &(_parameters.pitch_limit_min));
	param_get(_parameter_handles.pitch_limit_max, &(_parameters.pitch_limit_max));
	param_get(_parameter_handles.roll_limit, &(_parameters.roll_limit));
	param_get(_parameter_handles.throttle_min, &(_parameters.throttle_min));
	param_get(_parameter_handles.throttle_max, &(_parameters.throttle_max));
	param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));

	param_get(_parameter_handles.throttle_land_max, &(_parameters.throttle_land_max));

	param_get(_parameter_handles.time_const, &(_parameters.time_const));
	param_get(_parameter_handles.min_sink_rate, &(_parameters.min_sink_rate));
	param_get(_parameter_handles.max_sink_rate, &(_parameters.max_sink_rate));
	param_get(_parameter_handles.throttle_damp, &(_parameters.throttle_damp));
	param_get(_parameter_handles.integrator_gain, &(_parameters.integrator_gain));
	param_get(_parameter_handles.vertical_accel_limit, &(_parameters.vertical_accel_limit));
	param_get(_parameter_handles.height_comp_filter_omega, &(_parameters.height_comp_filter_omega));
	param_get(_parameter_handles.speed_comp_filter_omega, &(_parameters.speed_comp_filter_omega));
	param_get(_parameter_handles.roll_throttle_compensation, &(_parameters.roll_throttle_compensation));
	param_get(_parameter_handles.speed_weight, &(_parameters.speed_weight));
	param_get(_parameter_handles.pitch_damping, &(_parameters.pitch_damping));
	param_get(_parameter_handles.max_climb_rate, &(_parameters.max_climb_rate));

	param_get(_parameter_handles.heightrate_p, &(_parameters.heightrate_p));
	param_get(_parameter_handles.speedrate_p, &(_parameters.speedrate_p));

	param_get(_parameter_handles.land_slope_angle, &(_parameters.land_slope_angle));
	param_get(_parameter_handles.land_slope_length, &(_parameters.land_slope_length));
	param_get(_parameter_handles.land_H1_virt, &(_parameters.land_H1_virt));
	param_get(_parameter_handles.land_flare_alt_relative, &(_parameters.land_flare_alt_relative));
	param_get(_parameter_handles.land_thrust_lim_horizontal_distance, &(_parameters.land_thrust_lim_horizontal_distance));

	_l1_control.set_l1_damping(_parameters.l1_damping);
	_l1_control.set_l1_period(_parameters.l1_period);
	_l1_control.set_l1_roll_limit(math::radians(_parameters.roll_limit));

	_tecs.set_time_const(_parameters.time_const);
	_tecs.set_min_sink_rate(_parameters.min_sink_rate);
	_tecs.set_max_sink_rate(_parameters.max_sink_rate);
	_tecs.set_throttle_damp(_parameters.throttle_damp);
	_tecs.set_integrator_gain(_parameters.integrator_gain);
	_tecs.set_vertical_accel_limit(_parameters.vertical_accel_limit);
	_tecs.set_height_comp_filter_omega(_parameters.height_comp_filter_omega);
	_tecs.set_speed_comp_filter_omega(_parameters.speed_comp_filter_omega);
	_tecs.set_roll_throttle_compensation(_parameters.roll_throttle_compensation);
	_tecs.set_speed_weight(_parameters.speed_weight);
	_tecs.set_pitch_damping(_parameters.pitch_damping);
	_tecs.set_indicated_airspeed_min(_parameters.airspeed_min);
	_tecs.set_indicated_airspeed_max(_parameters.airspeed_max);
	_tecs.set_max_climb_rate(_parameters.max_climb_rate);
	_tecs.set_heightrate_p(_parameters.heightrate_p);
	_tecs.set_speedrate_p(_parameters.speedrate_p);

	/* sanity check parameters */
	if (_parameters.airspeed_max < _parameters.airspeed_min ||
	    _parameters.airspeed_max < 5.0f ||
	    _parameters.airspeed_min > 100.0f ||
	    _parameters.airspeed_trim < _parameters.airspeed_min ||
	    _parameters.airspeed_trim > _parameters.airspeed_max) {
		warnx("error: airspeed parameters invalid");
		return 1;
	}

	return OK;
}

void
FixedwingPositionControl::vehicle_control_mode_poll()
{
	bool vstatus_updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_control_mode_sub, &vstatus_updated);

	if (vstatus_updated) {

		bool was_armed = _control_mode.flag_armed;

		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);

		if (!was_armed && _control_mode.flag_armed) {
			_launch_lat = _global_pos.lat / 1e7f;
			_launch_lon = _global_pos.lon / 1e7f;
			_launch_alt = _global_pos.alt;
			_launch_valid = true;
		}
	}
}

bool
FixedwingPositionControl::vehicle_airspeed_poll()
{
	/* check if there is an airspeed update or if it timed out */
	bool airspeed_updated;
	orb_check(_airspeed_sub, &airspeed_updated);

	if (airspeed_updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
		_airspeed_valid = true;
		_airspeed_last_valid = hrt_absolute_time();

	} else {

		/* no airspeed updates for one second */
		if (_airspeed_valid && (hrt_absolute_time() - _airspeed_last_valid) > 1e6) {
			_airspeed_valid = false;
		}
	}

	/* update TECS state */
	_tecs.enable_airspeed(_airspeed_valid);

	return airspeed_updated;
}

void
FixedwingPositionControl::vehicle_attitude_poll()
{
	/* check if there is a new position */
	bool att_updated;
	orb_check(_att_sub, &att_updated);

	if (att_updated) {
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

		/* set rotation matrix */
		for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
				_R_nb(i, j) = _att.R[i][j];
	}
}

void
FixedwingPositionControl::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
FixedwingPositionControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool mission_item_triplet_updated;
	orb_check(_mission_item_triplet_sub, &mission_item_triplet_updated);

	if (mission_item_triplet_updated) {
		orb_copy(ORB_ID(mission_item_triplet), _mission_item_triplet_sub, &_mission_item_triplet);
		_setpoint_valid = true;
	}
}

void
FixedwingPositionControl::task_main_trampoline(int argc, char *argv[])
{
	l1_control::g_control->task_main();
}

float
FixedwingPositionControl::calculate_target_airspeed(float airspeed_demand)
{
	float airspeed;

	if (_airspeed_valid) {
		airspeed = _airspeed.true_airspeed_m_s;

	} else {
		airspeed = _parameters.airspeed_min + (_parameters.airspeed_max - _parameters.airspeed_min) / 2.0f;
	}

	/* cruise airspeed for all modes unless modified below */
	float target_airspeed = airspeed_demand;

	/* add minimum ground speed undershoot (only non-zero in presence of sufficient wind) */
	target_airspeed += _groundspeed_undershoot;

	if (0/* throttle nudging enabled */) {
		//target_airspeed += nudge term.
	}

	/* sanity check: limit to range */
	target_airspeed = math::constrain(target_airspeed, _parameters.airspeed_min, _parameters.airspeed_max);

	/* plain airspeed error */
	_airspeed_error = target_airspeed - airspeed;

	return target_airspeed;
}

void
FixedwingPositionControl::calculate_gndspeed_undershoot(const math::Vector2f &current_position, const math::Vector2f &ground_speed, const struct mission_item_triplet_s &mission_item_triplet)
{

	if (_global_pos_valid) {

		/* rotate ground speed vector with current attitude */
		math::Vector2f yaw_vector(_R_nb(0, 0), _R_nb(1, 0));
		yaw_vector.normalize();
		float ground_speed_body = yaw_vector * ground_speed;

		/* The minimum desired ground speed is the minimum airspeed projected on to the ground using the altitude and horizontal difference between the waypoints if available*/
		float distance = 0.0f;
		float delta_altitude = 0.0f;
		if (mission_item_triplet.previous_valid) {
			distance = get_distance_to_next_waypoint(mission_item_triplet.previous.lat, mission_item_triplet.previous.lon, mission_item_triplet.current.lat, mission_item_triplet.current.lon);
			delta_altitude = mission_item_triplet.current.altitude - mission_item_triplet.previous.altitude;
		} else {
			distance = get_distance_to_next_waypoint(current_position.getX(), current_position.getY(), mission_item_triplet.current.lat, mission_item_triplet.current.lon);
			delta_altitude = mission_item_triplet.current.altitude -  _global_pos.alt;
		}

		float ground_speed_desired = _parameters.airspeed_min * cosf(atan2f(delta_altitude, distance));


		/*
		 * Ground speed undershoot is the amount of ground velocity not reached
		 * by the plane. Consequently it is zero if airspeed is >= min ground speed
		 * and positive if airspeed < min ground speed.
		 *
		 * This error value ensures that a plane (as long as its throttle capability is
		 * not exceeded) travels towards a waypoint (and is not pushed more and more away
		 * by wind). Not countering this would lead to a fly-away.
		 */
		_groundspeed_undershoot = math::max(ground_speed_desired - ground_speed_body, 0.0f);

	} else {
		_groundspeed_undershoot = 0;
	}
}

float FixedwingPositionControl::getLandingSlopeAbsoluteAltitude(float wp_distance, float wp_altitude, float landing_slope_angle_rad, float horizontal_displacement)
{
	return (wp_distance - horizontal_displacement) * tanf(landing_slope_angle_rad) + wp_altitude; //flare_relative_alt is negative
}

bool
FixedwingPositionControl::control_position(const math::Vector2f &current_position, const math::Vector2f &ground_speed,
		const struct mission_item_triplet_s &mission_item_triplet)
{
	bool setpoint = true;

	calculate_gndspeed_undershoot(current_position, ground_speed, mission_item_triplet);

	float eas2tas = 1.0f; // XXX calculate actual number based on current measurements

	// XXX re-visit
	float baro_altitude = _global_pos.alt;

	/* filter speed and altitude for controller */
	math::Vector3 accel_body(_accel.x, _accel.y, _accel.z);
	math::Vector3 accel_earth = _R_nb.transpose() * accel_body;

	_tecs.update_50hz(baro_altitude, _airspeed.indicated_airspeed_m_s, _R_nb, accel_body, accel_earth);
	float altitude_error = _mission_item_triplet.current.altitude - _global_pos.alt;

	/* no throttle limit as default */
	float throttle_max = 1.0f;

	/* AUTONOMOUS FLIGHT */

	// XXX this should only execute if auto AND safety off (actuators active),
	// else integrators should be constantly reset.
	if (_control_mode.flag_control_position_enabled) {

		/* get circle mode */
		bool was_circle_mode = _l1_control.circle_mode();

		/* restore speed weight, in case changed intermittently (e.g. in landing handling) */
		_tecs.set_speed_weight(_parameters.speed_weight);

		/* current waypoint (the one currently heading for) */
		math::Vector2f next_wp(mission_item_triplet.current.lat, mission_item_triplet.current.lon);

		/* current waypoint (the one currently heading for) */
		math::Vector2f curr_wp(mission_item_triplet.current.lat, mission_item_triplet.current.lon);


		/* previous waypoint */
		math::Vector2f prev_wp;

		if (mission_item_triplet.previous_valid) {
			prev_wp.setX(mission_item_triplet.previous.lat);
			prev_wp.setY(mission_item_triplet.previous.lon);

		} else {
			/*
			 * No valid previous waypoint, go for the current wp.
			 * This is automatically handled by the L1 library.
			 */
			prev_wp.setX(mission_item_triplet.current.lat);
			prev_wp.setY(mission_item_triplet.current.lon);

		}

		if (mission_item_triplet.current.nav_cmd == NAV_CMD_WAYPOINT || mission_item_triplet.current.nav_cmd == NAV_CMD_RETURN_TO_LAUNCH) {
			/* waypoint is a plain navigation waypoint */
			_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed);
			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _mission_item_triplet.current.altitude, calculate_target_airspeed(_parameters.airspeed_trim),
						    _airspeed.indicated_airspeed_m_s, eas2tas,
						    false, math::radians(_parameters.pitch_limit_min),
						    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
						    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));

		} else if (mission_item_triplet.current.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
			   mission_item_triplet.current.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			   mission_item_triplet.current.nav_cmd == NAV_CMD_LOITER_UNLIMITED) {

			/* waypoint is a loiter waypoint */
			_l1_control.navigate_loiter(curr_wp, current_position, mission_item_triplet.current.loiter_radius,
						  mission_item_triplet.current.loiter_direction, ground_speed);
			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _mission_item_triplet.current.altitude, calculate_target_airspeed(_parameters.airspeed_trim),
						    _airspeed.indicated_airspeed_m_s, eas2tas,
						    false, math::radians(_parameters.pitch_limit_min),
						    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
						    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));

		} else if (mission_item_triplet.current.nav_cmd == NAV_CMD_LAND) {

			/* Horizontal landing control */
			/* switch to heading hold for the last meters, continue heading hold after */
			float wp_distance = get_distance_to_next_waypoint(current_position.getX(), current_position.getY(), curr_wp.getX(), curr_wp.getY());
			//warnx("wp dist: %d, alt err: %d, noret: %s", (int)wp_distance, (int)altitude_error, (land_noreturn) ? "YES" : "NO");
			const float heading_hold_distance = 15.0f;
			if (wp_distance < heading_hold_distance || land_noreturn_horizontal) {

				/* heading hold, along the line connecting this and the last waypoint */
				

				// if (mission_item_triplet.previous_valid) {
				// 	target_bearing = get_bearing_to_next_waypoint(prev_wp.getX(), prev_wp.getY(), next_wp.getX(), next_wp.getY());
				// } else {

				if (!land_noreturn_horizontal) //set target_bearing in first occurrence
					target_bearing = _att.yaw;
				//}

//					warnx("NORET: %d, target_bearing: %d, yaw: %d", (int)land_noreturn_horizontal, (int)math::degrees(target_bearing), (int)math::degrees(_att.yaw));

				_l1_control.navigate_heading(target_bearing, _att.yaw, ground_speed);

				/* limit roll motion to prevent wings from touching the ground first */
				_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-10.0f), math::radians(10.0f));

				land_noreturn_horizontal = true;

			} else {

				/* normal navigation */
				_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed);
			}

			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();


			/* Vertical landing control */
			//xxx: using the tecs altitude controller for slope control for now

//				/* do not go down too early */
//				if (wp_distance > 50.0f) {
//					altitude_error = (_global_triplet.current.altitude + 25.0f) - _global_pos.alt;
//				}
			/* apply minimum pitch (flare) and limit roll if close to touch down, altitude error is negative (going down) */
			// XXX this could make a great param

			float flare_angle_rad = -math::radians(5.0f);//math::radians(mission_item_triplet.current.param1)
			float land_pitch_min = math::radians(5.0f);
			float throttle_land = _parameters.throttle_min + (_parameters.throttle_max - _parameters.throttle_min) * 0.1f;
			float airspeed_land = 1.3f * _parameters.airspeed_min;
			float airspeed_approach = 1.3f * _parameters.airspeed_min;

			float landing_slope_angle_rad = math::radians(_parameters.land_slope_angle);
			float flare_relative_alt = _parameters.land_flare_alt_relative;
			float motor_lim_horizontal_distance = _parameters.land_thrust_lim_horizontal_distance;//be generous here as we currently have to rely on the user input for the waypoint
			float L_wp_distance = get_distance_to_next_waypoint(prev_wp.getX(), prev_wp.getY(), curr_wp.getX(), curr_wp.getY()) * _parameters.land_slope_length;
			float H1 = _parameters.land_H1_virt;
			float H0 = flare_relative_alt + H1;
			float d1 = flare_relative_alt/tanf(landing_slope_angle_rad);
			float flare_constant = (H0 * d1)/flare_relative_alt;//-flare_length/(logf(H1/H0));
			float flare_length = - logf(H1/H0) * flare_constant;//d1+20.0f;//-logf(0.01f/flare_relative_alt);
			float horizontal_slope_displacement = (flare_length - d1);
			float L_altitude = getLandingSlopeAbsoluteAltitude(L_wp_distance, _mission_item_triplet.current.altitude, landing_slope_angle_rad, horizontal_slope_displacement);
			float landing_slope_alt_desired = getLandingSlopeAbsoluteAltitude(wp_distance, _mission_item_triplet.current.altitude, landing_slope_angle_rad, horizontal_slope_displacement);

			if ( (_global_pos.alt < _mission_item_triplet.current.altitude + flare_relative_alt) || land_noreturn_vertical) {  //checking for land_noreturn to avoid unwanted climb out

				/* land with minimal speed */

//					/* force TECS to only control speed with pitch, altitude is only implicitely controlled now */
//					_tecs.set_speed_weight(2.0f);

				/* kill the throttle if param requests it */
				throttle_max = _parameters.throttle_max;

				 if (wp_distance < motor_lim_horizontal_distance || land_motor_lim) {
					throttle_max = math::min(throttle_max, _parameters.throttle_land_max);
					if (!land_motor_lim) {
						land_motor_lim  = true;
						mavlink_log_info(mavlink_fd, "[POSCTRL] Landing, limit throttle");
					}

				 }

				float flare_curve_alt =   _mission_item_triplet.current.altitude + H0 * expf(-math::max(0.0f, flare_length - wp_distance)/flare_constant) - H1;

				/* avoid climbout */
				if (flare_curve_alt_last < flare_curve_alt && land_noreturn_vertical || land_stayonground)
				{
					flare_curve_alt = mission_item_triplet.current.altitude;
					land_stayonground = true;
				}

				_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, flare_curve_alt, calculate_target_airspeed(airspeed_land),
												    _airspeed.indicated_airspeed_m_s, eas2tas,
												    false, flare_angle_rad,
												    0.0f, throttle_max, throttle_land,
												    flare_angle_rad,  math::radians(15.0f));

				if (!land_noreturn_vertical) {
					mavlink_log_info(mavlink_fd, "[POSCTRL] Landing, flare");
					land_noreturn_vertical = true;
				}
				//warnx("Landing:  flare, _global_pos.alt  %.1f, flare_curve_alt %.1f, flare_curve_alt_last %.1f, flare_length %.1f, wp_distance %.1f", _global_pos.alt, flare_curve_alt, flare_curve_alt_last, flare_length, wp_distance);

				flare_curve_alt_last = flare_curve_alt;

			} else if (wp_distance < L_wp_distance) {

				/* minimize speed to approach speed, stay on landing slope */
				_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, landing_slope_alt_desired, calculate_target_airspeed(airspeed_approach),
							    _airspeed.indicated_airspeed_m_s, eas2tas,
							    false, flare_angle_rad,
							    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
							    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));
				//warnx("Landing: stay on slope, alt_desired: %.1f (wp_distance: %.1f), calculate_target_airspeed(airspeed_land) %.1f, horizontal_slope_displacement %.1f, d1 %.1f, flare_length %.1f", landing_slope_alt_desired, wp_distance, calculate_target_airspeed(airspeed_land), horizontal_slope_displacement, d1, flare_length);

				if (!land_onslope) {

					mavlink_log_info(mavlink_fd, "[POSCTRL] Landing, on slope");
					land_onslope = true;
				}

			} else {

				 /* intersect glide slope:
				 * if current position is higher or within 10m of slope follow the glide slope
				 * if current position is below slope -10m continue on maximum of previous wp altitude or L_altitude until the intersection with the slope
				 * */
				float altitude_desired = _global_pos.alt;
				if (_global_pos.alt > landing_slope_alt_desired - 10.0f) {
					/* stay on slope */
					altitude_desired = landing_slope_alt_desired;
					//warnx("Landing: before L, stay on landing slope, alt_desired: %.1f (wp_distance: %.1f, L_wp_distance %.1f), calculate_target_airspeed(airspeed_land) %.1f, horizontal_slope_displacement %.1f", altitude_desired, wp_distance, L_wp_distance, calculate_target_airspeed(airspeed_land), horizontal_slope_displacement);
				} else {
					/* continue horizontally */
					altitude_desired =  math::max(_global_pos.alt, L_altitude);
					//warnx("Landing: before L,continue at: %.4f, (landing_slope_alt_desired %.4f, wp_distance: %.4f, L_altitude: %.4f L_wp_distance: %.4f)", altitude_desired, landing_slope_alt_desired, wp_distance, L_altitude, L_wp_distance);
				}

				_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, altitude_desired, calculate_target_airspeed(airspeed_approach),
							    _airspeed.indicated_airspeed_m_s, eas2tas,
							    false, math::radians(_parameters.pitch_limit_min),
							    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
							    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));
			}

		} else if (mission_item_triplet.current.nav_cmd == NAV_CMD_TAKEOFF) {

			_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed);
			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			/* apply minimum pitch and limit roll if target altitude is not within 10 meters */
			if (altitude_error > 15.0f) {

				/* enforce a minimum of 10 degrees pitch up on takeoff, or take parameter */
				_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _mission_item_triplet.current.altitude, calculate_target_airspeed(1.3f * _parameters.airspeed_min),
							    _airspeed.indicated_airspeed_m_s, eas2tas,
							    true, math::max(math::radians(mission_item_triplet.current.radius), math::radians(10.0f)),
							    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
							    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));

				/* limit roll motion to ensure enough lift */
				_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-15.0f), math::radians(15.0f));

			} else {

				_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _mission_item_triplet.current.altitude, calculate_target_airspeed(_parameters.airspeed_trim),
							    _airspeed.indicated_airspeed_m_s, eas2tas,
							    false, math::radians(_parameters.pitch_limit_min),
							    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
							    math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max));
			}
		}

		// warnx("nav bearing: %8.4f bearing err: %8.4f target bearing: %8.4f", (double)_l1_control.nav_bearing(),
		//       (double)_l1_control.bearing_error(), (double)_l1_control.target_bearing());
		// warnx("prev wp: %8.4f/%8.4f, next wp: %8.4f/%8.4f prev:%s", (double)prev_wp.getX(), (double)prev_wp.getY(),
		//       (double)next_wp.getX(), (double)next_wp.getY(), (mission_item_triplet.previous_valid) ? "valid" : "invalid");

		// XXX at this point we always want no loiter hold if a
		// mission is active
		_loiter_hold = false;

		/* reset land state */
		if (mission_item_triplet.current.nav_cmd != NAV_CMD_LAND) {
			land_noreturn_horizontal = false;
			land_noreturn_vertical = false;
			land_stayonground = false;
			land_motor_lim = false;
			land_onslope = false;
		}

		if (was_circle_mode && !_l1_control.circle_mode()) {
			/* just kicked out of loiter, reset roll integrals */
			_att_sp.roll_reset_integral = true;
		}

	} else if (0/* easy mode enabled */) {

		/** EASY FLIGHT **/

		if (0/* switched from another mode to easy */) {
			_seatbelt_hold_heading = _att.yaw;
		}

		if (0/* easy on and manual control yaw non-zero */) {
			_seatbelt_hold_heading = _att.yaw + _manual.yaw;
		}

		/* climb out control */
		bool climb_out = false;

		/* user wants to climb out */
		if (_manual.pitch > 0.3f && _manual.throttle > 0.8f) {
			climb_out = true;
		}

		/* if in seatbelt mode, set airspeed based on manual control */

		// XXX check if ground speed undershoot should be applied here
		float seatbelt_airspeed = _parameters.airspeed_min +
					  (_parameters.airspeed_max - _parameters.airspeed_min) *
					  _manual.throttle;

		_l1_control.navigate_heading(_seatbelt_hold_heading, _att.yaw, ground_speed);
		_att_sp.roll_body = _l1_control.nav_roll();
		_att_sp.yaw_body = _l1_control.nav_bearing();
		_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _global_pos.alt + _manual.pitch * 2.0f,
					    seatbelt_airspeed,
					    _airspeed.indicated_airspeed_m_s, eas2tas,
					    false, _parameters.pitch_limit_min,
					    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
					    _parameters.pitch_limit_min, _parameters.pitch_limit_max);

	} else if (0/* seatbelt mode enabled */) {

		/** SEATBELT FLIGHT **/

		if (0/* switched from another mode to seatbelt */) {
			_seatbelt_hold_heading = _att.yaw;
		}

		if (0/* seatbelt on and manual control yaw non-zero */) {
			_seatbelt_hold_heading = _att.yaw + _manual.yaw;
		}

		/* if in seatbelt mode, set airspeed based on manual control */

		// XXX check if ground speed undershoot should be applied here
		float seatbelt_airspeed = _parameters.airspeed_min +
					  (_parameters.airspeed_max - _parameters.airspeed_min) *
					  _manual.throttle;

		/* user switched off throttle */
		if (_manual.throttle < 0.1f) {
			throttle_max = 0.0f;
			/* switch to pure pitch based altitude control, give up speed */
			_tecs.set_speed_weight(0.0f);
		}

		/* climb out control */
		bool climb_out = false;

		/* user wants to climb out */
		if (_manual.pitch > 0.3f && _manual.throttle > 0.8f) {
			climb_out = true;
		}

		_l1_control.navigate_heading(_seatbelt_hold_heading, _att.yaw, ground_speed);
		_att_sp.roll_body =	_manual.roll;
		_att_sp.yaw_body =	_manual.yaw;
		_tecs.update_pitch_throttle(_R_nb, _att.pitch, _global_pos.alt, _global_pos.alt + _manual.pitch * 2.0f,
					    seatbelt_airspeed,
					    _airspeed.indicated_airspeed_m_s, eas2tas,
					    climb_out, _parameters.pitch_limit_min,
					    _parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
					    _parameters.pitch_limit_min, _parameters.pitch_limit_max);

	} else {

		/** MANUAL FLIGHT **/

		/* no flight mode applies, do not publish an attitude setpoint */
		setpoint = false;
	}

	_att_sp.pitch_body = _tecs.get_pitch_demand();
	_att_sp.thrust = math::min(_tecs.get_throttle_demand(), throttle_max);

	return setpoint;
}

void
FixedwingPositionControl::task_main()
{

	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_mission_item_triplet_sub = orb_subscribe(ORB_ID(mission_item_triplet));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_control_mode_sub, 200);
	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);

	/* abort on a nonzero return value from the parameter init */
	if (parameters_update()) {
		/* parameter setup went wrong, abort */
		warnx("aborting startup due to errors.");
		_task_should_exit = true;
	}

	/* wakeup source(s) */
	struct pollfd fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _global_pos_sub;
	fds[1].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* check vehicle status for changes to publication state */
		vehicle_control_mode_poll();

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if position changed */
		if (fds[1].revents & POLLIN) {


			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f)
				deltaT = 0.01f;

			/* load local copies */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);

			// XXX add timestamp check
			_global_pos_valid = true;

			vehicle_attitude_poll();
			vehicle_setpoint_poll();
			vehicle_accel_poll();
			vehicle_airspeed_poll();
			// vehicle_baro_poll();

			math::Vector2f ground_speed(_global_pos.vx, _global_pos.vy);
			math::Vector2f current_position(_global_pos.lat / 1e7f, _global_pos.lon / 1e7f);

			/*
			 * Attempt to control position, on success (= sensors present and not in manual mode),
			 * publish setpoint.
			 */
			if (control_position(current_position, ground_speed, _mission_item_triplet)) {
				_att_sp.timestamp = hrt_absolute_time();

				/* lazily publish the setpoint only once available */
				if (_attitude_sp_pub > 0) {
					/* publish the attitude setpoint */
					orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_pub, &_att_sp);

				} else {
					/* advertise and publish */
					_attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
				}

				/* XXX check if radius makes sense here */
				float turn_distance = _l1_control.switch_distance(_mission_item_triplet.current.radius);

				/* lazily publish navigation capabilities */
				if (turn_distance != _nav_capabilities.turn_distance && turn_distance > 0) {

					/* set new turn distance */
					_nav_capabilities.turn_distance = turn_distance;

					if (_nav_capabilities_pub > 0) {
						orb_publish(ORB_ID(navigation_capabilities), _nav_capabilities_pub, &_nav_capabilities);
					} else {
						_nav_capabilities_pub = orb_advertise(ORB_ID(navigation_capabilities), &_nav_capabilities);
					}
				}

			}

		}

		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_exit(0);
}

int
FixedwingPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("fw_pos_control_l1",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       4048,
				       (main_t)&FixedwingPositionControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int fw_pos_control_l1_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: fw_pos_control_l1 {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (l1_control::g_control != nullptr)
			errx(1, "already running");

		l1_control::g_control = new FixedwingPositionControl;

		if (l1_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != l1_control::g_control->start()) {
			delete l1_control::g_control;
			l1_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (l1_control::g_control == nullptr)
			errx(1, "not running");

		delete l1_control::g_control;
		l1_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (l1_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}