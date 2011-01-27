#ifndef _DEBUG_CHANNELS_H_
#define _DEBUG_CHANNELS_H_

enum debug_channel {
	ALT_GAS = 0,
	ALT_PID_ERR = 1,
	ALT_PID_INT = 2,
	ALT_PID_D = 3,
	ALT_PID_SP = 4,
	BUMP_GAS = 5,
	BUMP_ENABLE = 6,
	BUMP_DT_S = 7,
	DBG_VALID_USS = 8,
	DBG_VALID_IR1 = 9,
	DBG_VALID_IR2 = 10,
	ALT_DT_S = 11
};

#endif
