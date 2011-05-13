#ifndef _DEBUG_CHANNELS_H_
#define _DEBUG_CHANNELS_H_

// mavlink debug channels
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
	ALT_DT_S = 11,
	ALT_CORR_BASE = 12,
	CH1_MEAN = 16,
	CH1_VAR = 17,
	CH1_VAR_E = 18
	// next is 16 + numchan(5) * 3 = 31
};

// huch-FC/MK debugchannels
/// debugout type to index map
enum mk_debugout_map_t {
	USSvalue = 0,
	USSlastvalid = 1,
	StickNick = 2, // XXX: changed in FC to sticknick, roll, yaw
	ADval_press = 3,
	ATTabsh = 4,
	StickRoll = 5,
	StickYaw = 6,
	USSstatus = 7,
	ADval_gyrroll = 8,
	ADval_gyrnick = 9,
	ADval_gyryaw = 10,
	ATTrelacctopint = 11,
	ADval_ubat = 12,
	GASmixfrac1 = 13,
	GASmixfrac2 = 14,
	RC_rssi = 15,
	ATTmeanaccnick = 16,
	ATTmeanaccroll = 17,
	ATTmeanacctop = 18,
	ATTintnickl = 19,
	ATTintrolll = 20,
	ATTintyawl = 21,
	FCParam_extctlswitch = 22,
	FCParam_gpsswitch = 23,
	ADval_accnick = 24,
	ADval_accroll = 25,
	ADval_acctop = 26,
	CTL_stickgas = 27,
	ADval_acctopraw = 28,
	ATTintnickh = 29,
	ATTintrollh = 30,
	ATTintyawh = 31
};


#endif
