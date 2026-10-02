#ifndef INC_FORCE_H_
#define INC_FORCE_H_

#include "main.h"

#define OUTPUT_MIN 3276       //  (20% of 2^14 counts or 0x0ccc)
#define OUTPUT_MAX 13107       // (80% of 2^14 counts or 0x3333)
//#define FORCE_MAX 1.4   			// 15N (I want results in N)
#define FORCE_MAX	15

extern uint8_t Fstatus;
extern float force0, force1, force2, force3, force4;
extern float Temperature0, Temperature1, Temperature2;

void Force_Process(void);

#endif /* INC_FORCE_H_ */
