#include "mpptLink.h"
#include <string.h>


void initMpptFrame(MpptFrame *frame)
{
	frame->frameState = NO_SYNCH;
	frame->batteryVoltage = -1.0;
	frame->solarCurrent = -1.0;
	frame->frameIndex = 0;
	frame->length = 0;
}

int parseMpptFrame(unsigned char *buf, MpptFrame *frame)
{
	switch(*buf)
	{
		case MPPT_LINK_START_CHAR : 
		{
			initMpptFrame(frame);
			frame->frameState = SYNCH;
			
			return 0;
		}
		case MPPT_LINK_STOP_CHAR : 
		{
			/* Frame is not valid */
			if(frame->frameState != END)
			{
				initMpptFrame(frame);
				return 0;
			}
			
			/* Frame is valid */
			if(frame->length == MPPT_FRAME_LENGTH)
			{
				frame->frameState = COMPLETE;
				return 1;
			}
			else /* Frame is not valid */
			{
				initMpptFrame(frame);
				return 0;
			}
		}
		default :
		{
			buildMpptFrame(buf, frame);
			return 0;
		}
	}
}

void buildMpptFrame(unsigned char *buf, MpptFrame *frame)
{
	switch(frame->frameState)
	{
		
		case SYNCH :
		{
		
			frame->frameState = VOLTAGE;
				
			unsigned char *pVolt = (unsigned char *)(&frame->batteryVoltage) + frame->frameIndex;
			*pVolt = *buf;
				
			frame->frameIndex++;
			frame->length++;
			
			break;
		}
		
		case VOLTAGE :
		{
			unsigned char *pVolt = (unsigned char *)(&frame->batteryVoltage)  + frame->frameIndex;
			*pVolt = *buf;
			
			frame->length++;
			
			if(frame->frameIndex == sizeof(float) - 1)
			{
				frame->frameIndex = 0;
				frame->frameState = CURRENT;
			}
			else
			{
				frame->frameIndex++;
			}		
			break;
		}
		
		case CURRENT :
		{
			unsigned char *pVolt = (unsigned char *)(&frame->solarCurrent) + frame->frameIndex;
			*pVolt = *buf;
			
			frame->length++;
			
			if(frame->frameIndex == sizeof(float) - 1)
			{
				frame->frameIndex = 0;
				frame->frameState = END;
			}
			else
			{
				frame->frameIndex++;
			}			
			break;
		}
		
		default : break;
	}
}

void createMpptFrame(MpptFrame *frame, unsigned char *buffer)
{
	
	buffer[0] = MPPT_LINK_START_CHAR;

	memcpy(&buffer[1], &(frame->batteryVoltage), sizeof(frame->batteryVoltage));
	memcpy(&buffer[5], &(frame->solarCurrent), sizeof(frame->solarCurrent));
	
	buffer[9] = MPPT_LINK_STOP_CHAR;

}