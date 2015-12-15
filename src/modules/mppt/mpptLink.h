#ifndef MPPT_LINK_H
#define MPPT_LINK_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MPPT_LINK_START_CHAR 0xFE
#define MPPT_LINK_STOP_CHAR	0x55

#define MPPT_MAX_FRAME_SIZE	50

typedef enum _MpptFrameType
{
	STATUS_FRAME,
	EVENT_FRAME,
	NA_FRAME

}MpptFrameType;

typedef struct _MpptStatusFrame
{
	float mpptTemperature;
	float solarCurrent;
	float totalCurrent;
	float batteryVoltage;
	unsigned char dutyCycleMin;
	unsigned char dutyCycleMax;
}MpptStatusFrame;


typedef enum _MpptEventType
{
	TEMPERATURE_EMERGENCY,
	MPPT_START,
	MPPT_STOP,
	DUCTY_CYCLE_OUT_OF_RANGE

}MpptEventType;

typedef struct _MpptEventFrame
{
	MpptEventType eventType;
	unsigned char param1;
	unsigned char param2;

}MpptEventFrame;

typedef enum _MpptParseResult
{
	MPPT_OK,
	CORRUPTION,
	BUFFER_EMPTY,
	MPPT_ERROR
}MpptParseResult;

typedef enum _MpptStateMachineStatus
{
	NO_FRAME,
	START_FOUND,
	STOP_FOUND,
}MpptStateMachineStatus;

typedef struct _MpptParsingStateMachine
{
	MpptStateMachineStatus status;
	unsigned char buffer[MPPT_MAX_FRAME_SIZE];
	int bufferIndex;

	MpptStatusFrame statusFrameRx;
	MpptEventFrame	eventFrameRx;

	void (*statusFrameFound)(MpptStatusFrame *statusFrame);
	void (*eventFrameFound)(MpptEventFrame *eventFrame);

}MpptParsingStateMachine;

int createMpptStatusFrame(MpptStatusFrame *statusFrame, unsigned char *buffer);
int createMpptEventFrame(MpptEventFrame *eventFrame, unsigned char *buffer);
void initMpptStateMachine(MpptParsingStateMachine* stateMachine);
int parseMpptFrame(MpptParsingStateMachine* stateMachine, unsigned char rxBuffer);
MpptFrameType getFrameType(unsigned char *frameBuffer, int bufferSize);
MpptParseResult buildStatusFrame(MpptStatusFrame *statusFrame, unsigned char *frameBuffer, int bufferSize);
MpptParseResult buildEventFrame(MpptEventFrame *eventFrame, unsigned char *frameBuffer, int bufferSize);
void processRxBuffer(MpptParsingStateMachine *stateMachine, unsigned char *rxBuffer, int size);


#endif /* MPPT_LINK_H */
