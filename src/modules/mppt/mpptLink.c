#include "mpptLink.h"

int createMpptStatusFrame(MpptStatusFrame *statusFrame, unsigned char *buffer)
{
	int i= 0;

	/* Setup frame start */
	buffer[i] = MPPT_LINK_START_CHAR;
	i++;

	/* It's a status frame */
	buffer[i] = STATUS_FRAME;
	i++;

	/* Copy temperature */
	memcpy(&buffer[i], &(statusFrame->mpptTemperature), sizeof(statusFrame->mpptTemperature));
	i+=sizeof(statusFrame->mpptTemperature);

	/* Copy solar current*/
	memcpy(&buffer[i], &(statusFrame->solarCurrent), sizeof(statusFrame->solarCurrent));
	i+=sizeof(statusFrame->solarCurrent);

	/* Copy total current*/
	memcpy(&buffer[i], &(statusFrame->totalCurrent), sizeof(statusFrame->totalCurrent));
	i+=sizeof(statusFrame->totalCurrent);

	/* Copy battery voltage*/
	memcpy(&buffer[i], &(statusFrame->batteryVoltage), sizeof(statusFrame->batteryVoltage));
	i+=sizeof(statusFrame->batteryVoltage);

	/* Copy duty cycle min*/
	memcpy(&buffer[i], &(statusFrame->dutyCycleMin), sizeof(statusFrame->dutyCycleMin));
	i+=sizeof(statusFrame->dutyCycleMin);

	/* Copy duty cycle max*/
	memcpy(&buffer[i], &(statusFrame->dutyCycleMax), sizeof(statusFrame->dutyCycleMax));
	i+=sizeof(statusFrame->dutyCycleMax);

	/* End of frame */
	buffer[i] = MPPT_LINK_STOP_CHAR;
	i++;

	return i;
}

int createMpptEventFrame(MpptEventFrame *eventFrame, unsigned char *buffer)
{
	int i=0;

	/* Setup frame start */
	buffer[i] = MPPT_LINK_START_CHAR;
	i++;

	/* It's a status frame */
	buffer[i] = EVENT_FRAME;
	i++;

	/* Copy event type */
	memcpy(&buffer[i], &(eventFrame->eventType), sizeof(eventFrame->eventType));
	i+=sizeof(eventFrame->eventType);

	/* Copy event param 1 */
	memcpy(&buffer[i], &(eventFrame->param1), sizeof(eventFrame->param1));
	i+=sizeof(eventFrame->param1);

	/* Copye event param 2 */
	memcpy(&buffer[i], &(eventFrame->param2), sizeof(eventFrame->param2));
	i+=sizeof(eventFrame->param2);

	/* End of frame */
	buffer[i] = MPPT_LINK_STOP_CHAR;
	i++;

	return i;
}

void initMpptStateMachine(MpptParsingStateMachine* stateMachine)
{
	stateMachine->status = NO_FRAME;
	stateMachine->bufferIndex = 0;

	for(int i=0; i<MPPT_MAX_FRAME_SIZE; i++)
	{
		stateMachine->buffer[i] = 0;
	}
}

void processRxBuffer(MpptParsingStateMachine *stateMachine, unsigned char *rxBuffer, int size)
{
	for(int i = 0; i<size; i++)
	{
		if(parseMpptFrame(stateMachine, rxBuffer[i]))
		{
			
			MpptFrameType frameType = getFrameType(stateMachine->buffer, stateMachine->bufferIndex);
			MpptParseResult result;

			if(frameType == STATUS_FRAME)
			{
				result = buildStatusFrame(&(stateMachine->statusFrameRx), stateMachine->buffer, stateMachine->bufferIndex);
				if(result != BUFFER_EMPTY &&  result != MPPT_ERROR){

					if(result == MPPT_OK)
					{
						stateMachine->statusFrameFound(&(stateMachine->statusFrameRx));
					}
				}
			}
			else if(frameType == EVENT_FRAME)
			{
				result = buildEventFrame(&(stateMachine->eventFrameRx), stateMachine->buffer, stateMachine->bufferIndex);
				if(result != BUFFER_EMPTY &&  result != MPPT_ERROR){

					if(result == MPPT_OK)
					{
						stateMachine->eventFrameFound(&(stateMachine->eventFrameRx));
					}
				}
			}
			else
			{
				initMpptStateMachine(stateMachine);
			}
		}
	}
}

int parseMpptFrame(MpptParsingStateMachine* stateMachine, unsigned char rxBuffer)
{
	int frameFound = 0;

	if(stateMachine->status == STOP_FOUND)
	{
		initMpptStateMachine(stateMachine);
	}

	switch(stateMachine->status)
	{
		case NO_FRAME :
		{
			if(rxBuffer == MPPT_LINK_START_CHAR)
			{
				stateMachine->status = START_FOUND;
				stateMachine->buffer[stateMachine->bufferIndex] = rxBuffer;
				stateMachine->bufferIndex ++;
			}
			else
			{
				stateMachine->bufferIndex = 0;
			}

			frameFound = 0;
			break;
		}
		case START_FOUND :
		{
			if(stateMachine->bufferIndex < MPPT_MAX_FRAME_SIZE)
			{
				if(rxBuffer == MPPT_LINK_START_CHAR)
				{
					initMpptStateMachine(stateMachine);
					stateMachine->status = START_FOUND;
					frameFound = 0;
				}
				else if(rxBuffer == MPPT_LINK_STOP_CHAR)
				{
					stateMachine->status = STOP_FOUND;
					frameFound = 1;
				}

				stateMachine->buffer[stateMachine->bufferIndex] = rxBuffer;
				stateMachine->bufferIndex ++;
				break;
			}
			else
			{
				initMpptStateMachine(stateMachine);
				frameFound = 0;
				break;
			}
		}
	}

	return frameFound;
}

MpptFrameType getFrameType(unsigned char *frameBuffer, int bufferSize)
{
	if(bufferSize>0)
	{
		switch(frameBuffer[1])
		{
			case STATUS_FRAME : return STATUS_FRAME;
			case EVENT_FRAME : return EVENT_FRAME;
		}
	}
	
	return NA_FRAME;
}

MpptParseResult buildStatusFrame(MpptStatusFrame *statusFrame, unsigned char *frameBuffer, int bufferSize)
{
	int i=0;
	int tmpSizeof = 0;

	if((statusFrame==NULL)|| (frameBuffer == NULL))
	{
		return MPPT_ERROR;
	}
	else if(bufferSize<3)
	{
		return BUFFER_EMPTY;
	}
	else if (frameBuffer[0] != MPPT_LINK_START_CHAR)
	{
		return CORRUPTION;
	}

	i++;
	if(frameBuffer[i] != STATUS_FRAME)
	{
		return CORRUPTION;
	}

	i++;

	/* Parse temperature */
	tmpSizeof = sizeof(statusFrame->mpptTemperature);
	if((i + tmpSizeof) <= bufferSize)
	{
		memcpy(&(statusFrame->mpptTemperature), &frameBuffer[i], tmpSizeof);
		i+= tmpSizeof;
	}
	else
	{
		return BUFFER_EMPTY;
	}

	/* Parse solar current*/
	tmpSizeof = sizeof(statusFrame->solarCurrent);
	if((i + tmpSizeof) <= bufferSize)
	{
		memcpy(&(statusFrame->solarCurrent), &frameBuffer[i], tmpSizeof);
		i+= tmpSizeof;
	}
	else
	{
		return BUFFER_EMPTY;
	}

	/* Parse total current*/
	tmpSizeof = sizeof(statusFrame->totalCurrent);
	if((i + tmpSizeof) <= bufferSize)
	{
		memcpy(&(statusFrame->totalCurrent), &frameBuffer[i], tmpSizeof);
		i+= tmpSizeof;
	}
	else
	{
		return BUFFER_EMPTY;
	}

	/* Parse battery voltage*/
	tmpSizeof = sizeof(statusFrame->batteryVoltage);
	if((i + tmpSizeof) <= bufferSize)
	{
		memcpy(&(statusFrame->batteryVoltage), &frameBuffer[i], tmpSizeof);
		i+= tmpSizeof;
	}
	else
	{
		return BUFFER_EMPTY;
	}

	/* Parse duty cycle min*/
	tmpSizeof = sizeof(statusFrame->dutyCycleMin);
	if((i + tmpSizeof) <= bufferSize)
	{
		memcpy(&(statusFrame->dutyCycleMin), &frameBuffer[i], tmpSizeof);
		i+= tmpSizeof;
	}
	else
	{
		return BUFFER_EMPTY;
	}

	/* Parse duty cycle max*/
	tmpSizeof = sizeof(statusFrame->dutyCycleMax);
	if((i + tmpSizeof) <= bufferSize)
	{
		memcpy(&(statusFrame->dutyCycleMax), &frameBuffer[i], tmpSizeof);
		i+= tmpSizeof;
	}
	else
	{
		return BUFFER_EMPTY;
	}

	if(frameBuffer[i] != MPPT_LINK_STOP_CHAR)
	{
		return CORRUPTION;
	}
	
	return MPPT_OK;
}

MpptParseResult buildEventFrame(MpptEventFrame *eventFrame, unsigned char *frameBuffer, int bufferSize)
{
	int i=0;
	int tmpSizeof = 0;

	if((eventFrame==NULL)|| (frameBuffer == NULL))
	{
		return MPPT_ERROR;
	}
	else if(bufferSize<3)
	{
		return BUFFER_EMPTY;
	}
	else if (frameBuffer[0] != MPPT_LINK_START_CHAR)
	{
		return CORRUPTION;
	}

	i++;
	if(frameBuffer[i] != EVENT_FRAME)
	{
		return CORRUPTION;
	}

	i++;


	/* Parse event type */
	tmpSizeof = sizeof(eventFrame->eventType);
	if((i + tmpSizeof) <= bufferSize)
	{
		memcpy(&(eventFrame->eventType), &frameBuffer[i], tmpSizeof);
		i+= tmpSizeof;
	}
	else
	{
		return BUFFER_EMPTY;
	}

	/* Parse param 1 */
	tmpSizeof = sizeof(eventFrame->param1);
	if((i + tmpSizeof) <= bufferSize)
	{
		memcpy(&(eventFrame->param1), &frameBuffer[i], tmpSizeof);
		i+= tmpSizeof;
	}
	else
	{
		return BUFFER_EMPTY;
	}

	/* Parse param 2 */
	tmpSizeof = sizeof(eventFrame->param2);
	if((i + tmpSizeof) <= bufferSize)
	{
		memcpy(&(eventFrame->param2), &frameBuffer[i], tmpSizeof);
		i+= tmpSizeof;
	}
	else
	{
		return BUFFER_EMPTY;
	}

	if(frameBuffer[i] != MPPT_LINK_STOP_CHAR)
	{
		return CORRUPTION;
	}

	return MPPT_OK;
}
