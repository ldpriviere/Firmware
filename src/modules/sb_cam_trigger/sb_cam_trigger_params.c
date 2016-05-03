/**
 * @file sb_cam_trigger_params.c
 * Parameters for camera triggering for Sunbirds
 *
 * @author Laurent Rivière
 */


#include <systemlib/param/param.h>

/**
 * Camera triggering ON/OFF parameter
 * @min 0
 * @max 1
 * @group Sunbirds parameters
 */
PARAM_DEFINE_FLOAT(SB_CAM_ONOFF, 1);

/**
 * Camera triggering period in seconds
 * @min 1.0
 * @group Sunbirds parameters
 */
PARAM_DEFINE_FLOAT(SB_CAM_PERIOD, 1.0f);


/**
 * Camera triggering pin (aux pins from 1 to 6)
 *
 * @min 1
 * @max 6
 * @group Sunbirds parameters
 */
PARAM_DEFINE_FLOAT(SB_CAM_PIN, 1);


/**
 * Camera triggering polarity
 *
 * @min 0
 * @max 1
 * @group Sunbirds parameters
 */
PARAM_DEFINE_FLOAT(SB_CAM_POL, 0);

/**
 * Camera trigger activation time
 *
 * This parameter sets the time the trigger needs to pulled high or low.
 *
 * @unit ms
 * @min 0.1
 * @max 3
 * @decimal 1
 * @group Camera trigger
 */
PARAM_DEFINE_FLOAT(SB_CAM_ATIME, 0.5f);
