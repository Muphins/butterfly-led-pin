/*******************************************************************************
*                                  INCLUDES                                    *
*******************************************************************************/
#include "filters.h"
/*******************************************************************************
*                                    CODE                                      *
*******************************************************************************/
cLPF::cLPF(uint8_t ratio_32): m_value(0), m_retValue(0)/*, m_sign(false)*/
{
	m_ratio = ratio_32;
	m_rationInv = FLOAT_TO_FACTOR(1) - ratio_32;
}

int8_t cLPF::run(int8_t sample)
{
	m_value *= m_rationInv;
	m_value += (((int32_t)sample * (m_ratio)) << (FILTER_HEADSPACE-BITSHIFT_FACTOR)) +(1<<(FILTER_HEADSPACE-BITSHIFT_FACTOR))-1;
	m_value = m_value >> (BITSHIFT_FACTOR);
	m_retValue = m_value >> (FILTER_HEADSPACE-BITSHIFT_FACTOR);
	
	return m_retValue;
}
