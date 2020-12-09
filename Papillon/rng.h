#ifndef RNG_H_
#define RNG_H_

#define COEF_MULT	129 //245
#define COEF_INCR	123

class cRng{
	uint8_t seed;
	public:
	cRng(void){};
	uint8_t run(void){seed = seed * COEF_MULT + COEF_INCR; return seed;}
};

#endif /* RNG_H_ */