#ifndef MPPT_LINK_H
#define MPPT_LINK_H

#define MPPT_LINK_START_CHAR 0xFE
#define MPPT_LINK_STOP_CHAR	0x55
#define MPPT_FRAME_LENGTH 8

typedef enum _MpptFrameState {

	NO_SYNCH,
	SYNCH,
	VOLTAGE,
	CURRENT,
	END,
	COMPLETE

}MpptFrameState;

typedef struct _MttpFrame{

	MpptFrameState frameState;
	int frameIndex;
	int length;
	
	float batteryVoltage;
	float solarCurrent;

}MpptFrame;


void initMpptFrame(MpptFrame *frame);
int parseMpptFrame(unsigned char *buf, MpptFrame *frame);
void buildMpptFrame(unsigned char *buf, MpptFrame *frame);
void createMpptFrame(MpptFrame *frame, unsigned char *buffer);

#endif /* MPPT_LINK_H */