
#include <stdlib.h>
#include <stdio.h>

#include "pid.h"
#include "nrf.h"

void PIDinit(PID_t * pid, float *pgain, float *igain, float *dgain, float *ilimit, float *numsamples)
{
	pid->processGain = pgain;
	pid->integralTime = igain;
	pid->derivateTime = dgain;

	pid->error = 0;
	pid->accumulatedError = 0;
	pid->differentialError = 0;
	pid->lastFeedbackReading = 0;

	pid->iLimit=ilimit;
	pid->EMAnumberSamples = numsamples;
}

float exponentialMovingAverage( const float value, const float previousEMA, const float alpha )
{
	float EMA = previousEMA;
	EMA += alpha * (value - previousEMA);
	return EMA;
}

void calculateP( const float setpoint, const float actual_position, PID_t* pPID )
{
	pPID->error = setpoint - actual_position;
}

void calculateI( const float setpoint, const float dTmilliseconds, PID_t* pPID )
{
	pPID->accumulatedError += pPID->error * dTmilliseconds/(*pPID->integralTime);

	if(pPID->accumulatedError >  (*pPID->iLimit))
	{
		pPID->accumulatedError = (*pPID->iLimit);
	}
	else if(pPID->accumulatedError < -(*pPID->iLimit))
	{
		pPID->accumulatedError = -(*pPID->iLimit);
	}
}

void calculateD( const float actual_position, const float dTmilliseconds, PID_t* pPID )
{
	float currentDifferentialError = -1 * (*pPID->derivateTime)*((actual_position - pPID->lastFeedbackReading)/dTmilliseconds);
	pPID->lastFeedbackReading = actual_position;

	if( *pPID->EMAnumberSamples > 0)
	{
		pPID->differentialError = exponentialMovingAverage( currentDifferentialError, pPID->differentialError, ( 1.0 / *pPID->EMAnumberSamples ) );
	}
	else
	{
		pPID->differentialError = currentDifferentialError;
	}
}

float PIDUpdate( float setpoint, float actual_position, float dTmilliseconds, PID_t* pPID )
{
	float controllerOutput = 0;

	calculateP(setpoint, actual_position, pPID);

	if ( *pPID->integralTime == 0 ) pPID->accumulatedError = 0;
	else calculateI(setpoint, dTmilliseconds, pPID);

	if ( *pPID->derivateTime == 0 ) pPID->differentialError = 0;
	else calculateD( actual_position, dTmilliseconds, pPID );

	controllerOutput = ( *pPID->processGain )*( pPID->error + pPID->accumulatedError + pPID->differentialError );
	return controllerOutput;
}
