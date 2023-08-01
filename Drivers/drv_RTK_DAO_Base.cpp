#include "drv_RTK_DAO_Base.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "StorageSystem.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
};

static const uint8_t ubloxClr[] = {
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x0A,0x00,0x04,0x23,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x09,0x00,0x03,0x21,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x0D,0x00,0x07,0x29,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x06,0x00,0x00,0x1B,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x07,0x00,0x01,0x1D,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x0E,0x00,0x08,0x2B,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x0F,0x00,0x09,0x2D,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x08,0x00,0x02,0x1F,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x00,0x00,0xFB,0x12,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x01,0x00,0xFC,0x14,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x03,0x00,0xFE,0x18,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x04,0x00,0xFF,0x1A,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x05,0x00,0x00,0x1C,
		0xB5,0x62,0x06,0x01,0x03,0x00,0xF1,0x06,0x00,0x01,0x1E,
		
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0B ,0x30 ,0x00 ,0x45 ,0xC0,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0B ,0x50 ,0x00 ,0x65 ,0x00,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0B ,0x33 ,0x00 ,0x48 ,0xC6,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0B ,0x31 ,0x00 ,0x46 ,0xC2,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0B ,0x00 ,0x00 ,0x15 ,0x60,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x10 ,0x02 ,0x00 ,0x1C ,0x73,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x10 ,0x02 ,0x00 ,0x1C ,0x73,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x10 ,0x02 ,0x00 ,0x1C ,0x73,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x10 ,0x10 ,0x00 ,0x2A ,0x8F,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x21 ,0x0E ,0x00 ,0x39 ,0xBE,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x21 ,0x08 ,0x00 ,0x33 ,0xB2,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x21 ,0x09 ,0x00 ,0x34 ,0xB4,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x21 ,0x0B ,0x00 ,0x36 ,0xB8,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x21 ,0x0F ,0x00 ,0x3A ,0xC0,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x21 ,0x0D ,0x00 ,0x38 ,0xBC,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x05 ,0x00 ,0x19 ,0x67,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x09 ,0x00 ,0x1D ,0x6F,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x0B ,0x00 ,0x1F ,0x73,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x02 ,0x00 ,0x16 ,0x61,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x06 ,0x00 ,0x1A ,0x69,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x07 ,0x00 ,0x1B ,0x6B,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x21 ,0x00 ,0x35 ,0x9F,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x2E ,0x00 ,0x42 ,0xB9,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0A ,0x08 ,0x00 ,0x1C ,0x6D,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x60 ,0x00 ,0x6B ,0x02,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x22 ,0x00 ,0x2D ,0x86,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x31 ,0x00 ,0x3C ,0xA4,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x04 ,0x00 ,0x0F ,0x4A,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x40 ,0x00 ,0x4B ,0xC2,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x61 ,0x00 ,0x6C ,0x04,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x09 ,0x00 ,0x14 ,0x54,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x34 ,0x00 ,0x3F ,0xAA,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x01 ,0x00 ,0x0C ,0x44,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x02 ,0x00 ,0x0D ,0x46,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x07 ,0x00 ,0x12 ,0x50,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x35 ,0x00 ,0x40 ,0xAC,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x32 ,0x00 ,0x3D ,0xA6,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x06 ,0x00 ,0x11 ,0x4E,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x03 ,0x00 ,0x0E ,0x48,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x30 ,0x00 ,0x3B ,0xA2,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x24 ,0x00 ,0x2F ,0x8A,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x23 ,0x00 ,0x2E ,0x88,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x20 ,0x00 ,0x2B ,0x82,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x21 ,0x00 ,0x2C ,0x84,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x11 ,0x00 ,0x1C ,0x64,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x12 ,0x00 ,0x1D ,0x66,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x01 ,0x3c ,0x00 ,0x47 ,0xBA,
		
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x30 ,0x00 ,0x3C ,0xA5,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x31 ,0x00 ,0x3D ,0xA7,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x10 ,0x00 ,0x1C ,0x65,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x15 ,0x00 ,0x21 ,0x6F,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x11 ,0x00 ,0x1D ,0x67,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x13 ,0x00 ,0x1F ,0x6B,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x02 ,0x20 ,0x00 ,0x2C ,0x85,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x11 ,0x00 ,0x28 ,0x88,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x16 ,0x00 ,0x2D ,0x92,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x13 ,0x00 ,0x2A ,0x8C,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x04 ,0x00 ,0x1B ,0x6E,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x03 ,0x00 ,0x1A ,0x6C,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x12 ,0x00 ,0x29 ,0x8A,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x01 ,0x00 ,0x18 ,0x68,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x0D ,0x06 ,0x00 ,0x1D ,0x72,
		0xB5,0x62,0x06,0x01,0x03,0x00, 0x27 ,0x01 ,0x00 ,0x32 ,0xb6,	
};

static const uint8_t ubloxInit[] = {
		0xb5, 0x62, 0x06, 0x24, 0x24, 0x00,  0x05, 0x00,  0x07, 0x03,  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0x5d, 0xc9,	//NAV5 2g
		//0xb5, 0x62,  0x06, 0x24, 0x24, 0x00,  0x05, 0x00,  0x08, 0x03,  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0x5e, 0xeb,	//NAV5 4g
		//0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x07,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x4E,0xFD,
		0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12,            // set rate to 10Hz
	  //0xB5,0x62,0x06,0x08,0x06,0x00,0xc,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A,            // set rate to 5Hz

		//UBX-CFG-TP5,GPS time
//		0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x40,0x42, 
//		0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7, 0x00, 0x00, 0x00, 0xA5,0x73,
		0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x40,0x42, 
    0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x86, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7, 0x00, 0x00, 0x00, 0xCA,0xB6,
				
		0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x20, 0x01, 0x2C, 0x83,	// set UBX-NAV-TIMEGPS rate			
		0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51, // set UBX-NAV-PVT rate
		0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x15, 0x01, 0x22, 0x70, // set UBX-RXM-RAWX rate
		0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x13, 0x01, 0x20, 0x6C, // set UBX-RXM-SFRBX rate
//		0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x04, 0x01, 0x10, 0x4B, // set UBX-NAV-DOP rate
		0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0D, 0x03, 0x01, 0x1B, 0x6D, // set UBX-TIM-TM2 rate
	
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf5, 0xfe, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x41,	//40720
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf5, 0xfd, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x3a,	//40721
//	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf5, 0x05, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0A, 0x72,	//1005
//	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf5, 0x4a, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4F, 0x55,	//1074
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf5, 0x4d, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x52, 0x6a,	//1077
//	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf5, 0x54, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x59, 0x9B,	//1084
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf5, 0x57, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x5C, 0xB0,	//1087
//	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf5, 0x5E, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x63, 0xE1,	//1094
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf5, 0x61, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x66, 0xF6,	//1097
//	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf5, 0x7C, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x81, 0xB3,	//1124
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf5, 0x7F, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x84, 0xC8,	//1127
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf5, 0xE6, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xEB, 0x99,	//1230
	
	
	
	//RTCM输出波特率460800, IN：UBX+NMEA+RTCM3, OUT:UBX
		0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x02, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 
		0x00, 0x08, 0x07, 0x00, 0x23, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x5E,
	
    //飞控端波特率460800, IN：UBX+NMEA+RTCM3, OUT:UBX
		0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 
		0x00, 0x08, 0x07, 0x00, 0x23, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x90
};

#define write_ubx(buf,i,x, cka,ckb) {buf[i++]=x; cka+=x; ckb+=cka;}

static uint8_t GNSS_cfg_header[] = { 0x06,0x3e, 60,0 ,0x00,0x00,0x20, 7  };
#define GNSS_cfg_header_length_pos 2
#define GNSS_cfg_header_N_pos 7
static const uint8_t GNSS_cfgs[][8] =
{
	{ 0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01 },
	{ 0x01,0x01,0x03,0x00,0x01,0x00,0x01,0x01 },
	{ 0x02,0x04,0x08,0x00,0x01,0x00,0x01,0x01 },
	{ 0x03,0x08,0x10,0x00,0x01,0x00,0x01,0x01 },
	{ 0x04,0x00,0x08,0x00,0x01,0x00,0x01,0x01 },
	{ 0x05,0x00,0x03,0x00,0x01,0x00,0x01,0x01 },
	{ 0x06,0x08,0x0E,0x00,0x01,0x00,0x01,0x01 },
};

enum GPS_Scan_Operation
{
	//在指定波特率下发送初始化信息
	GPS_Scan_Baud9600 = 9600 ,
	GPS_Scan_Baud38400 = 38400 ,
	GPS_Scan_Baud460800 = 460800 ,
	GPS_Scan_Baud115200 = 115200 ,
	
	//检查是否设置成功
	GPS_Check_Baud ,
	//在当前波特率下再次发送配置
	GPS_ResendConfig ,
	//GPS已检测存在
	GPS_Present ,
};

struct GPS_State_Machine
{
	uint32_t frame_datas_ind = 0;
	uint16_t frame_datas_length;
	uint8_t read_state = 0;	//0=
	uint8_t CK_A , CK_B;	//checksum
};
static inline void ResetRxStateMachine( GPS_State_Machine* state_machine )
{
	state_machine->read_state=state_machine->frame_datas_ind=0;
}
static inline uint16_t GPS_ParseByte( GPS_State_Machine* state_machine, uint8_t* frame_datas, uint8_t r_data )
{
	frame_datas[ state_machine->frame_datas_ind++ ] = r_data;
	switch( state_machine->read_state )
	{
		case 0:	//找包头
		{				
			if( state_machine->frame_datas_ind == 1 )
			{
				if( r_data != 0xb5 )
					state_machine->frame_datas_ind = 0;
			}
			else
			{
				if( r_data == 0x62 )	//header found
				{
					state_machine->read_state = 1;
//					state_machine->frame_datas_ind = 0;
					state_machine->CK_A = state_machine->CK_B = 0;	//reset checksum
				}		
				else
					state_machine->frame_datas_ind = 0;
			}	
			break;
		}
		case 1:	//读Class ID和包长度
		{
			state_machine->CK_A += r_data;
			state_machine->CK_B += state_machine->CK_A;
			if( state_machine->frame_datas_ind == 6 )
			{
				state_machine->frame_datas_length = (*(unsigned short*)&frame_datas[4]) + 6;
				if( state_machine->frame_datas_length > 6 && state_machine->frame_datas_length < 1500 )
					state_machine->read_state = 2;						
				else
					ResetRxStateMachine(state_machine);
			}
			break;
		}
		case 2:	//读包内容
		{
			state_machine->CK_A += r_data;
			state_machine->CK_B += state_machine->CK_A;
			
			if( state_machine->frame_datas_ind == state_machine->frame_datas_length )
			{
				//payload read completed
				state_machine->read_state = 3;
			}
			break;
		}
		case 3://校验
		{
			if( state_machine->frame_datas_ind == state_machine->frame_datas_length + 1 )
			{
				if( r_data != state_machine->CK_A )
					ResetRxStateMachine(state_machine);
			}
			else
			{
				ResetRxStateMachine(state_machine);
				if( r_data == state_machine->CK_B )
					return state_machine->frame_datas_length;				
			}
		}
	}
	return 0;
}

static void RTK_Server(void* pvParameters)
{	
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	//GPS识别状态
	GPS_Scan_Operation current_GPS_Operation = GPS_Scan_Baud9600;
	//数据读取状态机
	__attribute__((aligned(4))) uint8_t frame_datas[2048];
	//上次更新时间
	TIME last_update_time;
	
	//注册Rtk端口
	RtkPort rtk_port;
	rtk_port.ena = false;
	rtk_port.write = driver_info.port.write;
	rtk_port.lock = driver_info.port.lock;
	rtk_port.unlock = driver_info.port.unlock;
	int8_t rtk_port_ind = RtkPortRegister(rtk_port);
	
	bool rtc_updated = false;

	//等待初始化完成
	while( getInitializationCompleted() == false )
		os_delay(0.1);
	
	//读取是否需要记录PPK
	bool record_ppk = false;
	uint8_t log_ppk[8];
	if( ReadParam( "SDLog_PPK", 0, 0, (uint64_t*)log_ppk, 0 ) == PR_OK )
	{
		if( log_ppk[0] == 2 )
			record_ppk = true;
	}
	
GPS_CheckBaud:
	while(1)
	{
		//更改指定波特率
		driver_info.port.SetBaudRate( current_GPS_Operation, 3, 0.1 );
		//发送配置
		driver_info.port.write( ubloxClr, sizeof(ubloxClr), portMAX_DELAY, portMAX_DELAY );
		uint16_t sd_length = 0;
		for(uint32_t i=0; i < sizeof(ubloxInit); i++)
		{
			if( i<sizeof(ubloxInit)-1 && ubloxInit[i] == 0xB5 && ubloxInit[i+1] == 0x62 && i>2 )
			{
				driver_info.port.wait_sent(1);
				os_delay(0.15);				
				driver_info.port.write( &ubloxInit[i-sd_length], sd_length, portMAX_DELAY, portMAX_DELAY );
				sd_length = 0;
			}
			++sd_length;
		}
		driver_info.port.wait_sent(1);
		os_delay(0.15);
		driver_info.port.write( &ubloxInit[sizeof(ubloxInit)-sd_length], sd_length, portMAX_DELAY, portMAX_DELAY );
		switch(current_GPS_Operation)
		{
			case GPS_Scan_Baud9600:
				current_GPS_Operation = GPS_Scan_Baud38400;
				break;
			case GPS_Scan_Baud38400:
				current_GPS_Operation = GPS_Scan_Baud460800;
				break;
			case GPS_Scan_Baud460800:
				current_GPS_Operation = GPS_Scan_Baud115200;
				break;
			default:
			case GPS_Scan_Baud115200:
				current_GPS_Operation = GPS_Scan_Baud9600;
				break;
		}
		
		//更改波特率
		driver_info.port.SetBaudRate( 460800, 3, 0.1 );
		//清空接收缓冲区准备接收数据
		driver_info.port.reset_rx(0.1);
		GPS_State_Machine gps_state;
		ResetRxStateMachine(&gps_state);
		TIME RxChkStartTime = TIME::now();
		while( RxChkStartTime.get_pass_time() < 2 )
		{
			uint8_t r_data;
			if( driver_info.port.read( &r_data, 1, 0.5, 0.1 ) )
			{
				uint16_t pack_length = GPS_ParseByte( &gps_state, frame_datas, r_data );
				if( pack_length )
				{
					if( frame_datas[2]==0x01 && frame_datas[3]==0x07 )
					{	//已识别到PVT包
						//跳转到gps接收程序
						goto GPS_Present;
					}
				}
			}
		}
	}
	
GPS_Present:
	//重发配置
	uint16_t sd_length = 0;
	for(uint32_t i=0; i < sizeof(ubloxInit); i++)
	{
		if( i<sizeof(ubloxInit)-1 && ubloxInit[i] == 0xB5 && ubloxInit[i+1] == 0x62 && i>2 )
		{
			driver_info.port.wait_sent(1);
			os_delay(0.11);
			driver_info.port.write( &ubloxInit[i-sd_length], sd_length, portMAX_DELAY, portMAX_DELAY );
			sd_length = 0;
		}
		++sd_length;
	}
	driver_info.port.wait_sent(1);
	os_delay(0.11);
	driver_info.port.write( &ubloxInit[sizeof(ubloxInit)-sd_length], sd_length, portMAX_DELAY, portMAX_DELAY );
	
	uint32_t sensor_key = 0;
	//发送GNSS配置
	GpsDAOConfig gps_cfg;
	if( ReadParamGroup( "GPSDAOCfg", (uint64_t*)&gps_cfg, 0 ) == PR_OK )
	{
		gps_cfg.GNSS_Mode[0] &= 0b1111111;
		if( gps_cfg.GNSS_Mode[0] != 0 )
		{	//发送GNSS配置
			
			//发送包头数据
			uint8_t buf[100];
			uint8_t ind = 2;	uint8_t cka = 0, ckb = 0;
			buf[0] = 0xb5;	buf[1] = 0x62;
			for( uint8_t i = 0; i < sizeof(GNSS_cfg_header); ++i )
				write_ubx( buf, ind, GNSS_cfg_header[i] , cka, ckb )
			
			//发送配置内容
			for( uint8_t i = 0; i < 7; ++i )
			{
				for( uint8_t k = 0; k < 8; ++k )
				{
					if( k != 4 )
						write_ubx( buf, ind, GNSS_cfgs[i][k] , cka, ckb )
					else							
					{
						if( gps_cfg.GNSS_Mode[0] & (1<<i) )
							write_ubx( buf, ind, 1 , cka, ckb )
						else
							write_ubx( buf, ind, 0 , cka, ckb )
					}
					
				}
			}
			
			//校验
			buf[ind++] = cka;	buf[ind++] = ckb;
			//发送
			driver_info.port.write( buf, ind, 0.1, 0.1 );
		}
		
		//注册传感器
		sensor_key = PositionSensorRegister( default_rtk_sensor_index , \
																					"RTK_Base_Ubx" ,\
																					Position_Sensor_Type_GlobalPositioning , \
																					Position_Sensor_DataType_sv_xy , \
																					Position_Sensor_frame_ENU , \
																					gps_cfg.delay[0] , //延时
																					30 , //xy信任度
																					30 //z信任度
																				);
	}
	else
	{	//注册传感器
		sensor_key = PositionSensorRegister( default_rtk_sensor_index , \
																					"RTK_Base_Ubx" ,\
																					Position_Sensor_Type_GlobalPositioning , \
																					Position_Sensor_DataType_sv_xy , \
																					Position_Sensor_frame_ENU , \
																					0.1 , //延时
																					30 , //xy信任度
																					30 //z信任度
																				);
	}
	
	//开启Rtk注入
	RtkPort_setEna( rtk_port_ind, true );
	
	//gps状态
	bool gps_available = false;
	bool z_available = false;
	TIME GPS_stable_start_time(false);
	double gps_alt;
	double alt_offset = -1000000;
	TIME gps_update_TIME;
	
	//清空接收缓冲区准备接收数据
	driver_info.port.reset_rx(0.1);
	GPS_State_Machine gps_state;
	ResetRxStateMachine(&gps_state);
	frame_datas[0] = frame_datas[1] = 0;
	last_update_time = TIME::now();
	
	//附加数据
	double addition_inf[8] = {0};
	
	while(1)
	{
		uint8_t r_data;
		if( driver_info.port.read( &r_data, 1, 2, 0.1 ) )
		{
			uint16_t pack_length = GPS_ParseByte( &gps_state, frame_datas, r_data );
			if( pack_length )
			{
				if( frame_datas[2]==0x01 && frame_datas[3]==0x07 )
				{//UBX-NAV-PVT  
					//记录
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, 100 );

					last_update_time = TIME::now();
					struct UBX_NAV_PVT_Pack
					{
						uint8_t CLASS;
						uint8_t ID;
						uint16_t length;
						
						uint32_t iTOW;
						uint16_t year;
						uint8_t month;
						uint8_t day;
						uint8_t hour;
						uint8_t min;
						uint8_t sec;
						uint8_t valid;
						uint32_t t_Acc;
						int32_t nano;
						uint8_t fix_type;
						uint8_t flags;
						uint8_t flags2;
						uint8_t numSV;
						int32_t lon;
						int32_t lat;
						int32_t height;
						int32_t hMSL;
						uint32_t hAcc;
						uint32_t vAcc;
						int32_t velN;
						int32_t velE;
						int32_t velD;
						int32_t gSpeed;
						int32_t headMot;
						uint32_t sAcc;
						uint32_t headAcc;
						uint16_t pDOP;
						uint8_t res1[6];
						int32_t headVeh;
						uint8_t res2[4];
					}__attribute__((packed));
					UBX_NAV_PVT_Pack* pack = (UBX_NAV_PVT_Pack*)&frame_datas[2];
					
					/*更新RTC时间(本地时间)*/	
						if( rtc_updated==false && gps_available )	{
							if( Lock_RTC() )
							{
								if( get_RTC_Updated() == false )
								{
									int8_t TimeZone = 8;//默认东8区(北京时区)
									RTC_TimeStruct rtc;
									if((pack->flags & 1) == 1 && pack->fix_type == 0x03)
										TimeZone = GetTimeZone(pack->lat*1e-7,pack->lon*1e-7);
									UTC2LocalTime(&rtc, pack->year, pack->month, pack->day, pack->hour, pack->min, pack->sec, TimeZone, 0);
									Set_RTC_Time(&rtc);	
								}
								rtc_updated = true;
								UnLock_RTC();
							}
						}
          /*更新RTC时时间(本地时间)*/

					uint8_t gps_fix = 0;
					if( pack->fix_type == 0 )
						gps_fix= 1;
					else
					{
						if( (pack->flags & 3) == 3 )
						{
							switch( pack->flags >> 6 )
							{
								case 1:
									gps_fix= 5;
									break;
								case 2:
									gps_fix= 6;
									break;
								default:
									gps_fix = pack->fix_type;
							}
						}
						else
							gps_fix = pack->fix_type;
					}
					if( ((pack->flags & 1) == 1) && (pack->fix_type == 0x03) && (pack->numSV >= 5) )
					{
						if( gps_available == false )
						{
							if( pack->hAcc < 2500 )
							{
								if( GPS_stable_start_time.is_valid() == false )
									GPS_stable_start_time = TIME::now();
								else if( GPS_stable_start_time.get_pass_time() > 3.0f )
								{
									gps_available = true;
									GPS_stable_start_time.set_invalid();
								}
							}
							else
								GPS_stable_start_time.set_invalid();
						}
						else
						{
							if( pack->hAcc > 3500 )
								gps_available = z_available = false;
						}
						
					}
					else
					{
						gps_available = z_available = false;
						GPS_stable_start_time.set_invalid();
					}
					
					addition_inf[0] = pack->numSV;
					addition_inf[1] = gps_fix;
					addition_inf[4] = pack->hAcc*0.1;
					addition_inf[5] = pack->vAcc*0.1;
					addition_inf[6] = pack->sAcc*0.1;
					
					if( z_available )
					{
						if( pack->vAcc > 4500 )
							z_available = false;
					}
					else
					{
						if( pack->vAcc < 2500 )
						{
							gps_alt = pack->hMSL * 1e-1;
							z_available = true;
						}
					}
					double t = gps_update_TIME.get_pass_time_st();
					if( t > 1 )
						t = 1;						
					
					vector3<double> velocity;
					velocity.y = pack->velN * 0.1;	//North
					velocity.x = pack->velE * 0.1;	//East
					velocity.z = -pack->velD * 0.1;	//Up
					gps_alt += velocity.z * t;
					if( pack->vAcc < 350 )
					{	//高精度高度位置结果
						//使用绝对高度
						double r_height = pack->hMSL*0.1;
						if( alt_offset <= -100000 )
							alt_offset = r_height - gps_alt;
						gps_alt += 0.5*t * ( r_height - gps_alt - alt_offset );
					}
					else
						alt_offset = -1000000;
					
					vector3<double> position_Global;
					position_Global.x = pack->lat * 1e-7;
					position_Global.y = pack->lon * 1e-7;
					position_Global.z = gps_alt;
	
					if( z_available )
						PositionSensorChangeDataType( default_rtk_sensor_index,sensor_key, Position_Sensor_DataType_sv_xyz );
					else
						PositionSensorChangeDataType( default_rtk_sensor_index,sensor_key, Position_Sensor_DataType_sv_xy );
					
					//信任度
					double xy_trustD = pack->hAcc * 0.1;
					double z_trustD = pack->vAcc * 0.1;
					if( z_trustD>20 && z_trustD<150 )
						z_trustD = 150;
					PositionSensorUpdatePositionGlobalVel( default_rtk_sensor_index,sensor_key, position_Global, velocity, 
						gps_available, //available
						-1, //delay
						xy_trustD, 
						z_trustD, 
						addition_inf,
						xy_trustD,	//xy LTtrust
						z_trustD	//z LTtrust
					);
				}
				else if( frame_datas[2]==0x02 && frame_datas[3]==0x15 )
				{//UBX-RXM-RAWX	
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, 24 + 32 * frame_datas[11+6] );					
				}
				else if( frame_datas[2]==0x02 && frame_datas[3]==0x13 )
				{//UBX-RXM-SFRBX	
					if(record_ppk)
						SDLog_Ubx( (const char*)frame_datas, 16 + 4 * frame_datas[4+6] );					
				}		
				else if( frame_datas[2]==0x0D && frame_datas[3]==0x03 )
				{//UBX-TIM-TM2				
					static uint16_t camTrigCount = 0;
					if( record_ppk)
					{
					 if((frame_datas[7] & 0x04 ) == 0x04)
					 {
						if(( frame_datas[7] & 0x80 ) == 0x80){
						 frame_datas[7] &=~(1<<7);
						}
						*(uint16_t*)&frame_datas[8]=++camTrigCount;
						uint8_t checksum_a = 0,checksum_b = 0;     
						for(uint8_t i = 2; i < 34; i++)
						{//重新计算校验
						 checksum_a += frame_datas[i];
						 checksum_b += checksum_a;
						}
						frame_datas[34] = checksum_a;
						frame_datas[35] = checksum_b;          
						SDLog_Ubx( (const char*)frame_datas, 36 );         
					 }
					}
				}					
				else if( frame_datas[2]==0x01 && frame_datas[3]==0x20 )
				{//UBX-NAV-TIMEGPS
					struct UBX_NAV_TIMEGPS_Pack
					{
						uint8_t CLASS;
						uint8_t ID;
						uint16_t length;
						
						uint32_t iTOW;
						int32_t fTOW;
						uint16_t week;
						uint8_t leapS;
						uint8_t valid;
						uint32_t tAcc;
					}__attribute__((packed));
					UBX_NAV_TIMEGPS_Pack* pack = (UBX_NAV_TIMEGPS_Pack*)&frame_datas[2];
					
          if( (pack->valid & (uint8_t)0x03) == (uint8_t)0x03 )
					{
						addition_inf[2] = pack->week;
						addition_inf[3] = pack->iTOW*1e-3 + pack->fTOW*1e-9;
					}
					if(record_ppk)					
						SDLog_Ubx( (const char*)frame_datas, 24 );												
				}					
			}
			
			if( last_update_time.get_pass_time() > 2 )
			{	//接收不到数据
				PositionSensorUnRegister( default_rtk_sensor_index,sensor_key );
				//关闭Rtk注入
				RtkPort_setEna( rtk_port_ind, false );
				goto GPS_CheckBaud;
			}
		}
		else
		{	//接收不到数据
			PositionSensorUnRegister( default_rtk_sensor_index,sensor_key );
			//关闭Rtk注入
			RtkPort_setEna( rtk_port_ind, false );
			goto GPS_CheckBaud;
		}
	}
}

static bool RTK_DriverInit( Port port, uint32_t param )
{
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( RTK_Server, "RTK_DAO_Base", 3200, driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_RTK_DAO_Base()
{
	//注册GPS参数	
	GpsDAOConfig initial_cfg;
	initial_cfg.GNSS_Mode[0] = 0;
	initial_cfg.delay[0] = 0.1;	
	initial_cfg.DRTK_VecX[0] = 0;
	initial_cfg.DRTK_VecY[0] = 30;
	initial_cfg.DRTK_VecZ[0] = 0;
	
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
	};
	SName param_names[] = {
		"GPSDAO_GNSS" ,
		"GPSDAO_delay" ,
		"GPSDAO_VecX" ,
		"GPSDAO_VecY" ,
		"GPSDAO_VecZ" ,
	};
	ParamGroupRegister( "GPSDAOCfg", 2, sizeof(initial_cfg)/8, param_types, param_names, (uint64_t*)&initial_cfg );
	
	PortFunc_Register( 15, RTK_DriverInit );
}
