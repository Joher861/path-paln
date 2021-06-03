#include "teb_planner/teb_local_planner/SimPID.h"
#include "teb_planner/teb_local_planner/teb_log.h"

#include <stdio.h>
#include <math.h>


/**
 * Initializes the SimPID object. All parameters default to 0.
 */
//ff是对目标值的scale，epsilon是最小误差，小于该值将不再计算pid输出
SimPID::SimPID(float p, float i, float d, float ff, float epsilon)
{
	m_p = p;
	m_i = i;
	m_d = d;
	m_ff = ff;

	m_errorEpsilon = epsilon;
	m_desiredValue = 0; // Default to 0, set later by the user
	m_firstCycle = true;
	m_maxOutput = 0.0; // Default to full range
	m_minOutput = 0.0;
	m_errorIncrement = 1;

	m_cycleCount = 0;
	m_minCycleCount = 10; // Default
	m_errorSum = 0;
	m_previousValue = 0;

	IsContinuousAngle = false; //其作用是让底盘从最小夹角方向旋转
	isDegrees = false;
}

/**
 * Sets the PID constants to new values.
 */
void SimPID::setConstants(float p, float i, float d)
{
	m_p = p;
	m_i = i;
	m_d = d;
	
}

void SimPID::reset()
{
	m_cycleCount = 0;
	m_errorSum = 0;
	m_previousValue = 0;
}

void SimPID::setFF(float ff)
{
	m_ff = ff;
}

void SimPID::setContinuousAngle(bool set)
{
	IsContinuousAngle = set;
}

void SimPID::setIsDegrees(bool set) {isDegrees = set;}

float SimPID::getError(void){
	return error;
}

inline float SimPID::normal(float x)
{
	if(isDegrees){
		if(x > 180)
			x -= 360;
		else if(x < -180)
			x += 360;
	}
	else
		x = atan2(sin(x), cos(x));
	return x;
}
float SimPID::getP()
{
	return m_p;
}

float SimPID::getI()
{
	return m_i;
}

float SimPID::getD()
{
	return m_d;
}

float SimPID::getFF()
{
	return m_ff;
}


float SimPID::getErrorSum()
{
	return m_errorSum;
}

/**
 * Sets the allowable error range away from the desired value.
 */
void SimPID::setErrorEpsilon(float epsilon)
{
	m_errorEpsilon = epsilon;
}

/**
 * Sets the maximum increment to the error sum used in the I component
 * calculation.
 */
void SimPID::setErrorIncrement(float inc)
{
	m_errorIncrement = inc;
}

/**
 * Sets the desired value.
 */
void SimPID::setDesiredValue(float val)
{
	m_desiredValue = val;
	if(IsContinuousAngle == true)
		m_desiredValue = normal(m_desiredValue);
}
	
/**
 * Sets the ceiling for the output of the calculation.
 * This means the limited max speed output.
 */
void SimPID::setMaxOutput(float max)
{	

	m_maxOutput = max;

}

/**
 * Sets the ceiling for the output of the calculation.
 * This means the min speed output.
 */
void SimPID::setMinOutput(float min)
{

	m_minOutput = min;

}

/**
 * Resets the error sum back to zero.
 */
void SimPID::resetErrorSum(void)
{
	m_errorSum = 0;
}

/**
 * Calculates the PID output based on the current value.
 * PID constants and desired value should be set before calling this
 * function.
 */
float SimPID::calcPID(float currentValue)
{	
	// Initialize all components to 0.0 to start.
	float pVal = 0.0;
	float iVal = 0.0;
	float dVal = 0.0;
	float ffVal = 0.0;
		
	// Don't apply D the first time through.
	if(m_firstCycle)
	{	
		m_previousValue = currentValue;  // Effective velocity of 0
		m_firstCycle = false;
	}
	
	// Calculate P Component.
	error = m_desiredValue - currentValue;
	if(IsContinuousAngle == true)  //找最小的夹角来旋转
		error = normal(m_desiredValue - currentValue);
	pVal = m_p * (float)error;
	//printf("SIMPID SAYS: Target: %f, current: %f, error: %f\n", m_desiredValue, currentValue, error);

	
	// Calculate I Component.
	// Error is positive and outside the epsilon band.
	if(error >= m_errorEpsilon)
	{	
		// Check if epsilon was pushing in the wrong direction.
		if(m_errorSum < 0)
		{
			// If we are fighting away from the point, reset the error.
			m_errorSum = 0;
			// TEB_DEBUG_LOG("m_errorSum < 0");
		}
		if(error < m_errorIncrement)
		{
			// If the error is smaller than the max increment amount, add it.
			m_errorSum += error;
			// TEB_DEBUG_LOG("error < m_errorIncrement.m_errorSum=%f",m_errorSum);
		}
		else 
		{
			// Otherwise, add the maximum increment per cycle.
			m_errorSum += m_errorIncrement;    
			// TEB_DEBUG_LOG("error >= m_errorIncrement.m_errorSum=%f",m_errorSum);  
		}
	}
	// Error is negative and outside the epsilon band.
	else if(error <= -m_errorEpsilon)
	{	
		if(m_errorSum > 0)
		{
			// If we are fighting away from the point, reset the error.
			m_errorSum = 0;
			// TEB_DEBUG_LOG("m_errorSum > 0");
		}
		// error is small than max contribution -> just subtract error amount
		if(error > -m_errorIncrement)
		{
			// If the error is smaller than the max increment amount, add it.
			m_errorSum += error; // Error is negative
			// TEB_DEBUG_LOG("error > -m_errorIncrement.m_errorSum=%f",m_errorSum);
		}
		else
		{
			// Otherwise, subtract the maximum increment per cycle.
			m_errorSum -= m_errorIncrement;
			// TEB_DEBUG_LOG("error <= -m_errorIncrement.m_errorSum=%f",m_errorSum);
		}
	}
	// Error is inside the epsilon band. 
	else
	{
		m_errorSum = 0;
	}
	iVal = m_i * (float)m_errorSum;
	
	// Calculate D Component.
	int velocity = currentValue - m_previousValue;
	dVal = m_d * (float)velocity;

	//calculate FF*target component
	ffVal = m_ff*m_desiredValue;

	// Calculate and limit the ouput: Output = P + I - D + FF
	float output = pVal + iVal - dVal + ffVal;
	if(output > m_maxOutput)
	{
		output = m_maxOutput;
	}
	else if(output < -m_maxOutput)
	{
		output = -m_maxOutput;
	}
	
	if (output < m_minOutput && output > -m_minOutput){
		if (output > 0)
			output = m_minOutput;
		else
			output = -m_minOutput;
	}
	//误差小于一定范围就停止输出
	// if (m_previousValue <= m_desiredValue + m_errorEpsilon
	// 			&& m_previousValue >= m_desiredValue - m_errorEpsilon
	// 			&& !m_firstCycle)
	// 	output = 0.0;

	// Save the current value for next cycle's D calculation.
	m_previousValue = currentValue;
	
	return output;
}

/**
 * Sets the minimum number of cycles the value must be in the epsilon range
 * before the system is considered stable.
 */
void SimPID::setMinDoneCycles(int n)
{
	m_minCycleCount = n;
}

/**
 * Returns true if the last input was within the epsilon range of the
 * destination value, and the system is stable.
 */
bool SimPID::isDone(void)
{	
	if (m_previousValue <= m_desiredValue + m_errorEpsilon
			&& m_previousValue >= m_desiredValue - m_errorEpsilon
			&& !m_firstCycle)
	{
		if(m_cycleCount >= m_minCycleCount)
		{
			return true;
		}
		else 
		{	
			m_cycleCount++;
			return false;
		}
	}
	else
	{
		m_cycleCount = 0;
		return false;
	}
}
