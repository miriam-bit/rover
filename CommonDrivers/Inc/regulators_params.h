/*
 * regulators_params.h
 *
 *  Created on: May 23, 2024
 *      Author: antonio
 */

#ifndef INC_REGULATORS_PARAMS_H_
#define INC_REGULATORS_PARAMS_H_

#include "pid_regulator.h"

#if defined(USE_NO_ANTI_WINDUP)


#define PID_ANT_DX_KP_FAST 		((double)0.003135809859137)
#define PID_ANT_SX_KP_FAST 		((double)0.002091775327127)

#define PID_POS_DX_KP_FAST 		((double)0.002132005179307)
#define PID_POS_SX_KP_FAST 		((double)0.003169034195340)

#define PID_ANT_DX_KI_FAST 	    ((double)-0.000245983178968)
#define PID_ANT_SX_KI_FAST 	    ((double)0.0008255792861618)

#define PID_POS_DX_KI_FAST 	    ((double)0.0007350948102003)
#define PID_POS_SX_KI_FAST 	    ((double)-0.000389059396464)

#define PID_ANT_DX_KD_FAST 	    ((double)0.00)
#define PID_ANT_SX_KD_FAST 	    ((double)0.00)

#define PID_POS_DX_KD_FAST 	    ((double)0.00)
#define PID_POS_SX_KD_FAST 	    ((double)0.00)

#define PID_ANT_DX_KP_SLOW 		((double)0.006907732110081)
#define PID_ANT_SX_KP_SLOW 		((double)0.006425291921394)

#define PID_POS_DX_KP_SLOW 		((double)0.006440085363628)
#define PID_POS_SX_KP_SLOW 		((double)0.006856298214178)

#define PID_ANT_DX_KI_SLOW 	    ((double)-0.005563934794570)
#define PID_ANT_SX_KI_SLOW 	    ((double)-0.005070885527091)

#define PID_POS_DX_KI_SLOW 	    ((double)-0.005107001871519)
#define PID_POS_SX_KI_SLOW 	    ((double)-0.005556666856265)


#define PID_ANT_DX_KD_SLOW	    ((double)0.00)
#define PID_ANT_SX_KD_SLOW	    ((double)0.00)

#define PID_POS_DX_KD_SLOW	    ((double)0.00)
#define PID_POS_SX_KD_SLOW	    ((double)0.00)


#elif defined(USE_CLAMPING)

#define PID_ANT_DX_KP_FAST 		((double)0.001690896519053)
#define PID_ANT_SX_KP_FAST 		((double)0.000633098020482)

#define PID_POS_DX_KP_FAST 		((double)0.000698455184553)
#define PID_POS_SX_KP_FAST 		((double)0.001779046795902)

#define PID_ANT_DX_KI_FAST 	    ((double)0.002889826680169)
#define PID_ANT_SX_KI_FAST 	    ((double)0.002917354613289)

#define PID_POS_DX_KI_FAST 	    ((double)0.002867099989508)
#define PID_POS_SX_KI_FAST 	    ((double)0.002779974798875)

#define PID_ANT_DX_KD_FAST 	    ((double)0.00)
#define PID_ANT_SX_KD_FAST 	    ((double)0.00)

#define PID_POS_DX_KD_FAST 	    ((double)0.00)
#define PID_POS_SX_KD_FAST 	    ((double)0.00)

#define PID_ANT_DX_KP_SLOW 		((double)0.006235833452325)
#define PID_ANT_SX_KP_SLOW 		((double)0.005748088724243)

#define PID_POS_DX_KP_SLOW 		((double)0.005773543617574)
#define PID_POS_SX_KP_SLOW 		((double)0.006206482535221)

#define PID_ANT_DX_KI_SLOW 	    ((double)0.001343797315511)
#define PID_ANT_SX_KI_SLOW 	    ((double)0.001354406394304)

#define PID_POS_DX_KI_SLOW 	    ((double)0.001333083492109)
#define PID_POS_SX_KI_SLOW 	    ((double)0.001299631357913)

#define PID_ANT_DX_KD_SLOW	    ((double)0.00)
#define PID_ANT_SX_KD_SLOW	    ((double)0.00)

#define PID_POS_DX_KD_SLOW	    ((double)0.00)
#define PID_POS_SX_KD_SLOW	    ((double)0.00)

#endif

#endif /* INC_REGULATORS_PARAMS_H_ */
