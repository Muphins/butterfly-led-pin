#ifndef RNG_H_
#define RNG_H_

#define COEF_MULT	25
#define COEF_INCR	123

class cRng{
	uint8_t seed;
  public:
	cRng(void){};
	/* Generates a sequence of 256 different numbers that repeats itself */
	uint8_t run(void){seed = seed * COEF_MULT + COEF_INCR; return seed;};	// 18 cycles
};

class cRngStrong{
	uint8_t m_seed;
	uint8_t m_coef;
	uint8_t m_counter;
  public:
	cRngStrong(uint8_t seed=6):m_coef(1),m_counter(0){m_seed=seed<<2|1;};
	/* Generates 64 sequences of 256 different numbers. resulting in 16384 different chains of two numbers */
	uint8_t run(void){m_seed = m_seed * m_coef + COEF_INCR;m_counter++;if(!m_counter)m_coef+=4;return m_seed;};	// 28 cycles
};

#endif /* RNG_H_ */