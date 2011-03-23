#ifndef QK_HELPER_H
#define QK_HELPER_H

/************************************************/
/* external control values                      */
/* size: 10 bytes                               */
/************************************************/
/* APFlags - autopilot flags */
/* if FCParam.ExternalControlSwitch is not set (>128) these flags have no function */
#define APFLAG_GENERAL_ON  				0x01	/* enables autopilot function */
#define APFLAG_KEEP_VALUES				0x02	/* if set values are kept until new one arrives */
#define APFLAG_TEMP_OFF	  				0x04	/* autopilot is temporary off for new RC-values */
#define APFLAG_FULL_CTRL    			0x08	/* autopilots fully controls the steering otherwise relativ control */
#define APFLAG_HEIGHT_CTRL0		      	0x10	/* enable height control: 00 - off, 10 - height value -> usshc setpoint...*/
#define APFLAG_HEIGHT_CTRL1		      	0x20	/* ...01 - height value -> baro set point, 11 - hold actual height */
#define APFLAG_RESERVE1		      		0x40
#define APFLAG_EXTCTL		      		0x80	/* read only: extern control value */

typedef struct 
{
	uint8_t	 remote_buttons;
 	int16_t  nick;
	int16_t  roll;
	int16_t  yaw;
	uint16_t gas;
 	// uint16_t height;
	volatile uint8_t AP_flags;
	uint8_t	frame;
	uint8_t	config;
} __attribute__((packed)) extern_control_t;

#endif
