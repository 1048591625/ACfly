#include "Basic.hpp"
#include "drv_ExtLed_TCA62724FMG.hpp"
#include "drv_ExtIIC.hpp"
#include "Commulink.hpp"

#define ADDR			0x55	/**< I2C adress of TCA62724FMG */
#define SUB_ADDR_START		0x01	/**< write everything (with auto-increment) */
#define SUB_ADDR_PWM0		0x81	/**< blue     (without auto-increment) */
#define SUB_ADDR_PWM1		0x82	/**< green    (without auto-increment) */
#define SUB_ADDR_PWM2		0x83	/**< red      (without auto-increment) */
#define SUB_ADDR_SETTINGS	0x84	/**< settings (without auto-increment)*/

#define SETTING_NOT_POWERSAVE	0x01	/**< power-save mode not off */
#define SETTING_ENABLE   	0x02	/**< on */

float ExtLedR = 0;
float ExtLedG = 0;
float ExtLedB = 0;

static void ExtLed_Server(void* pvParameters)
{
	Aligned_DMABuf uint8_t tx_buf[32];
	bool res;
reTry:
	while(1)
	{		
		os_delay(1.0);	
	
		tx_buf[0] = SUB_ADDR_SETTINGS;
		tx_buf[1] = SETTING_NOT_POWERSAVE;	
		res = ExtIIC_SendAddr7( ADDR, tx_buf, 2, IICSpeed_Standard50k );
		if(!res)
			continue;
		os_delay(2);
		
		tx_buf[0] = SUB_ADDR_PWM0;	tx_buf[1] = 0;
		tx_buf[2] = SUB_ADDR_PWM1;	tx_buf[3] = 0;
		tx_buf[4] = SUB_ADDR_PWM2;	tx_buf[5] = 0;
		res = ExtIIC_SendAddr7( ADDR, tx_buf, 6, IICSpeed_Standard50k );
		if(!res)
			continue;
		
		
		tx_buf[0] = SUB_ADDR_SETTINGS;
		tx_buf[1] = SETTING_NOT_POWERSAVE | SETTING_ENABLE;
		res = ExtIIC_SendAddr7( ADDR, tx_buf, 2, IICSpeed_Standard50k );	os_delay(1);
		if(!res)
			continue;
		
		break;
	}
	
	int retry_cnt = 0;
  while(1)
	{
		tx_buf[0] = SUB_ADDR_PWM0;	tx_buf[1] = ExtLedB*15.0f/100;
		tx_buf[2] = SUB_ADDR_PWM1;	tx_buf[3] = ExtLedG*15.0f/100;
		tx_buf[4] = SUB_ADDR_PWM2;	tx_buf[5] = ExtLedR*15.0f/100;
		res = ExtIIC_SendAddr7( ADDR, tx_buf, 6, IICSpeed_Standard50k );
		if(!res)
		{
			if( ++retry_cnt >= 3 )
				goto reTry;
		}
		else 
			retry_cnt = 0;
		
		os_delay(0.05);
	}
}

static bool I2C_ExtLed_TCA62724FMG_DriverInit()
{
	return true;
}
static bool I2C_ExtLed_TCA62724FMG_DriverRun()
{
	xTaskCreate( ExtLed_Server, "ExtLed", 500, NULL, SysPriority_UserTask, NULL);
	return true;
}

void init_drv_ExtLed_TCA62724FMG()
{
	I2CFunc_Register( 1, I2C_ExtLed_TCA62724FMG_DriverInit, I2C_ExtLed_TCA62724FMG_DriverRun );
}
