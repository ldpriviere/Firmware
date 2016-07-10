MODULE_COMMAND		= 	camera_trigger
SRCS				= 	camera_trigger.cpp \
						camera_trigger_params.c \
                		interfaces/src/camera_interface.cpp \
                		interfaces/src/pwm.cpp \
                		interfaces/src/relay.cpp \

MAXOPTIMIZATION	 = -Os