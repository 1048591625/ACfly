#include "M32_PosCtrl.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "AC_Math.hpp"
#include "Receiver.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "StorageSystem.hpp"
#include "Missions.hpp"
#include "NavCmdProcess.hpp"
#include "InFlightCmdProcess.hpp"
#include "drv_PWMOut.hpp"
#include "AuxFuncs.hpp"
#include "ctrl_Main.hpp"
#include "Filters_LP.hpp"
#include "Avoidance.hpp"
#include "vector2.hpp"

M32_PosCtrl::M32_PosCtrl():Mode_Base( "PosCtrl", 32 )
{
	
}

void M32_PosCtrl::get_MavlinkMode( ModeFuncCfg cfg, Receiver rc, 
																	uint8_t btn_zones[4], AFunc* mode )
{	//获取飞行模式
	if( rc.available )
	{	//接收机可用更新模式按钮状态		
		uint8_t MF_mode = *mode;
		
		uint8_t new_btn_zones[4];
		new_btn_zones[0] = get_RcButtonZone( rc.data[4], btn_zones[0] );
		new_btn_zones[1] = get_RcButtonZone( rc.data[5], btn_zones[1] );
		new_btn_zones[2] = get_RcButtonZone( rc.data[6], btn_zones[2] );
		new_btn_zones[3] = get_RcButtonZone( rc.data[7], btn_zones[3] );
		
		if( new_btn_zones[0]<=5 && new_btn_zones[0]!=btn_zones[0] )
		{	//模式按钮改变更改模式
			MF_mode = cfg.Bt1AFunc1[8*new_btn_zones[0]];			
		}
		
		/*判断执行任务*/
			if( cfg.MissionBt[0]>=2 && cfg.MissionBt[0]<=4 )
			{	//按钮按下执行任务
				if( rc.available_channels >= cfg.MissionBt[0]+4 )
				{
					uint8_t new_btn_zone = new_btn_zones[cfg.MissionBt[0]-1];
					uint8_t old_btn_zone = btn_zones[cfg.MissionBt[0]-1];
					if( new_btn_zone != old_btn_zone )
					{	//按钮状态发生变化
						if( new_btn_zone>=4 )
							MF_mode = AFunc_Mission;
						else if( old_btn_zone<=5 )
							MF_mode = 0;
					}
				}
			}
			else if( cfg.MissionBt[0]>=12 && cfg.MissionBt[0]<=14 )
			{	//按钮变化执行任务
				if( rc.available_channels >= cfg.MissionBt[0]-10+4 )
				{
					//获取按钮状态
					uint8_t new_btn_zone = new_btn_zones[cfg.MissionBt[0]-11];
					uint8_t old_btn_zone = btn_zones[cfg.MissionBt[0]-11];
					if( old_btn_zone<=5 && new_btn_zone!=old_btn_zone )
					{	//按钮状态发生变化
						if( *mode != AFunc_Mission )
							MF_mode = AFunc_Mission;
						else
							MF_mode = 0;
					}
				}
			}
		/*判断执行任务*/
		
		/*判断返航*/
			if( cfg.RTLBt[0]>=2 && cfg.RTLBt[0]<=4 )
			{	//按钮按下返航
				if( rc.available_channels >= cfg.RTLBt[0]+4 )
				{
					uint8_t new_btn_zone = new_btn_zones[cfg.RTLBt[0]-1];
					uint8_t old_btn_zone = btn_zones[cfg.RTLBt[0]-1];	
					if( new_btn_zone!=old_btn_zone )
					{	//按钮状态发生变化	
						if( new_btn_zone>=4 )
							MF_mode = AFunc_RTL;
						else if( old_btn_zone<=5 )
							MF_mode = 0;
					}
				}
			}
			else if( cfg.RTLBt[0]>=12 && cfg.RTLBt[0]<=14 )
			{	//按钮变化返航
				if( rc.available_channels >= cfg.RTLBt[0]-10+4 )
				{
					//获取按钮状态
					uint8_t new_btn_zone = new_btn_zones[cfg.RTLBt[0]-11];
					uint8_t old_btn_zone = btn_zones[cfg.RTLBt[0]-11];
					if( old_btn_zone<=5 && new_btn_zone!=old_btn_zone )
					{	//按钮状态发生变化
						if( *mode != AFunc_RTL )
							MF_mode = AFunc_RTL;
						else
							MF_mode = 0;
					}
				}
			}
		/*判断返航*/
		if( MF_mode == 0 )
			MF_mode = cfg.Bt1AFunc1[8*new_btn_zones[0]];
		
		*mode = (AFunc)MF_mode;
		
		btn_zones[0] = new_btn_zones[0];
		btn_zones[1] = new_btn_zones[1];
		btn_zones[2] = new_btn_zones[2];
		btn_zones[3] = new_btn_zones[3];
	}
	else
	{
		if( *mode == 0 )
			*mode = AFunc_RTL;
	}
}

ModeResult M32_PosCtrl::main_func( void* param1, uint32_t param2 )
{
	double freq = 50;
	double h = 1.0/freq;
	setLedMode(LEDMode_Flying1);
	Altitude_Control_Enable();
	Position_Control_Enable();
	uint16_t exit_mode_counter_rs = 0;
	uint16_t exit_mode_counter = 0;
	uint16_t exit_mode_Gcounter = 0;
	uint32_t RollOverProtectCounter = 0;
	
	//读取模式配置
	ModeFuncCfg MFunc_cfg;
	ReadParamGroup( "MFunc", (uint64_t*)&MFunc_cfg, 0 );
	
	//初始化模式判断
	AFunc cMode = AFunc_PosHold;
	if(param2)
	{
		cMode = (AFunc)param2;
		
		if( !is_AFunc_NoPos(cMode) && get_Position_MSStatus()!=MS_Ready )
		{	//切换到未定位模式错误
			sendLedSignal(LEDSignal_Err1);
			const char text[50] = {"Unable to arm in this mode without positioning."};						
			for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
			{
				const Port* port = get_CommuPort(i);
				if( port->write != 0 )
				{	
					mavlink_message_t msg_sd;
					if(mavlink_lock_chan( i, 0.01 )){
						mavlink_msg_statustext_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							i ,	//chan
							&msg_sd,
							MAV_SEVERITY_ALERT,
							text,
							0,0
						);				
						mavlink_msg_to_send_buffer(port->write, 
																			 port->lock,
																			 port->unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(i);
					}								
				}
			}
			return MR_Err;
		}
		
		if( (cMode==AFunc_PosHoldAv || cMode==AFunc_PosHoldNHAv) && getAvTargetsCount()==0 && (MFunc_cfg.configs[0]&MCfg_CanUnlockWithouAVSYS_Bit)==0 )
		{	//无避障传感器不允许在避障模式解锁
			sendLedSignal(LEDSignal_Err1);
			const char text[50] = {"Unable to arm in this mode without OASYS."};						
			for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
			{
				const Port* port = get_CommuPort(i);
				if( port->write != 0 )
				{	
					mavlink_message_t msg_sd;
					if(mavlink_lock_chan( i, 0.01 )){
						mavlink_msg_statustext_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							i ,	//chan
							&msg_sd,
							MAV_SEVERITY_ALERT,
							text,
							0,0
						);				
						mavlink_msg_to_send_buffer(port->write, 
																			 port->lock,
																			 port->unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(i);
					}								
				}
			}
			return MR_Err;
		}
	}
	
	//读取初始航线信息
	ReadParamGroup( "RestoreWpInf", (uint64_t*)&restoreWpInf, 0 );
	
	if( MFunc_cfg.configs[0] & MCfg_RstWp0_Bit )
		//需要解锁初始化航点为0
		setCurrentMission(0);

	//手动模式
	bool AvLocked = false;
	double AvMinDistance = -1;
	//无头
	double HeadlessYaw = -200;
	//无遥控持续时间
	TIME noRcStartTIME(false);
	
	//Loiter模式
	
	//任务模式
	bool mode_switched = true;
	#define change_Mode(x) {cMode=x; mode_switched = true;}
	uint16_t current_mission_ind;
	MissionInf current_mission_inf;
	uint8_t ModeButtonZone = 255;
	uint8_t MissionButtonZone = 255;
	uint8_t RTLButtonZone = 255;
	uint8_t rcv_Mode = 255;
	TIME rcvTIME(false);
	bool wpRNeedDecent = false;
	uint16_t doJumpCount = 0;
	uint16_t doJumpWP = 0;
	double missionTraveledDist = 0;
	
	//任务状态机
	NavCmdInf navInf; 
	init_NavCmdInf(&navInf);
	//指令执行是否完成（手动模式）
	bool ManualModeNavCmdInprogress = false;
	ModeMsg ManualModeNavCmd;
	
	//任务模式飞到上次坐标点状态
	uint8_t MissionMode_BackToLastWp = 0;
	
	//连续任务状态
	#define ctMissionState 0
	//是否处理inFlightCmd
	#define DealInFlightCmd 1
	//下一个任务递增量（中间的InFlightCmd个数）
	#define MissionInc 2
	//定距拍照当前距离倍数
	#define CamTriggDistMult 3
	
	//跟随模式
	vector3<double> followPos;
	vector3<double> followOrigin;
	TIME followDataUpdateTime(false);
	vector3<double> followTPos;
	vector3<double> followTVel;
	
	//Offboard模式临时变量
	TIME Offboard_ThrTIME(false);
	TIME Offboard_YawTIME(false);
	TIME Offboard_PitRolTIME(false);
	
	//初始化相机触发距离
	CamTriggDist = 0;
		
	//初始化Aux处理
	init_process_AuxFuncs();
		
	while(1)
	{
		os_delay(h);
		
		//获取接收机
		Receiver rc;
		getReceiver( &rc, 0, 0.02 );
		
		//处理Aux通道
		process_AuxFuncs(&rc,h);
		
		//获取消息
		bool msg_available;
		ModeMsg msg;
		msg_available = ModeReceiveMsg( &msg, 0 );
		uint8_t msg_handled = 0;	//1-返回acceped 2-返回denied
		
		bool inFlight;
		get_is_inFlight(&inFlight);
		
		if( msg_available && msg.cmd==MAV_CMD_COMPONENT_ARM_DISARM )
		{	//地面站加锁
			if( msg.params[0] == 0 )
			{
				if( (msg.cmd_type & CMD_TYPE_MASK) == CMD_TYPE_MAVLINK )
				{
					Attitude_Control_Disable();
					os_delay(1.0);
					
					uint8_t port_index = msg.cmd_type & CMD_TYPE_PORT_MASK;
					const Port* port = get_CommuPort( port_index );
					if( port->write )
					{
						mavlink_message_t msg_sd;
						if( mavlink_lock_chan( port_index, 0.01 ) )
						{
							mavlink_msg_command_ack_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								port_index ,
								&msg_sd,
								msg.cmd,	//command
								MAV_RESULT_ACCEPTED ,	//result
								100 ,	//progress
								0 ,	//param2
								msg.sd_sysid ,	//target system
								msg.sd_compid //target component
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(port_index);
						}
					}
				}
				return MR_OK;
			}
			msg_handled = 1;
		}
		if( get_CrashedState() )
		{	//侧翻加锁
			Attitude_Control_Disable();
			return MR_Err;
		}
		

		uint8_t reqMode = cMode;
		if( msg_available && msg.cmd==176 )
		{	//指令更改模式
			if( (int)msg.params[0] & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED )
			{	//mavlink定义模式
				uint32_t main_mode = msg.params[1];
				uint32_t sub_mode = msg.params[2];
				if( main_mode==PX4_CUSTOM_MAIN_MODE_POSCTL && sub_mode==PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL )
				{	//指令进入手动
					reqMode = AFunc_PosHold;
					msg_handled = 1;
				}
				else if( main_mode==PX4_CUSTOM_MAIN_MODE_POSCTL && sub_mode==PX4_CUSTOM_SUB_MODE_POSCTL_Avoidance )
				{
					reqMode = AFunc_PosHoldAv;
					msg_handled = 1;
				}
				else if( main_mode==PX4_CUSTOM_MAIN_MODE_POSCTL && sub_mode==PX4_CUSTOM_SUB_MODE_POSCTL_Headless )
				{
					reqMode = AFunc_PosHoldNH;
					msg_handled = 1;
				}
				else if( main_mode==PX4_CUSTOM_MAIN_MODE_POSCTL && sub_mode==PX4_CUSTOM_SUB_MODE_POSCTL_HeadlessAv )
				{
					reqMode = AFunc_PosHoldNHAv;
					msg_handled = 1;
				}
				else if( main_mode==PX4_CUSTOM_MAIN_MODE_POSCTL && sub_mode==PX4_CUSTOM_SUB_MODE_POSCTL_Circle )
				{
					reqMode = AFunc_ManualCircle;
					msg_handled = 1;
				}
				else if( main_mode==PX4_CUSTOM_MAIN_MODE_ALTCTL )
				{	//指令进入手动
					reqMode = AFunc_AltHold;
					msg_handled = 1;
				}
				else if( (main_mode==PX4_CUSTOM_MAIN_MODE_AUTO || main_mode==0) && sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_MISSION )
				{	//指令进入任务模式
					reqMode = AFunc_Mission;
					msg_handled = 1;
				}
				else if( (main_mode==PX4_CUSTOM_MAIN_MODE_AUTO || main_mode==0) && sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_RTL )
				{	//指令进入返航模式
					reqMode = AFunc_RTL;
					msg_handled = 1;
				}
				else if( (main_mode==PX4_CUSTOM_MAIN_MODE_AUTO || main_mode==0) && sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_LAND )
				{	//指令进入降落模式
					reqMode = AFunc_Land;
					msg_handled = 1;
				}
				else if( main_mode==PX4_CUSTOM_MAIN_MODE_OFFBOARD )
				{	//指令进入OFFBOARD模式
					reqMode = AFunc_Offboard;
					msg_handled = 1;
				}
			}
		}
		if( msg_available && msg.cmd==20 )
		{	//指令返航
			if( msg.params[0] == 1 )
			{
				navInf.temp[0] = msg.params[4];
				navInf.temp[1] = msg.params[5];
				navInf.temp[2] = msg.params[6];
				param1 = navInf.temp;
				reqMode = AFunc_RTL2Point;
				msg_handled = 1;
			}
		}
		if( rc.available )
		{	//接收机可用
			
			noRcStartTIME.set_invalid();
			//手势强制加锁
			if( (rc.data[0] < 10 && rc.data[1] < 10 && rc.data[2] < 10 && rc.data[3] > 90) )
			{
				if( ++exit_mode_Gcounter >= freq*1.5 )
				{
					Attitude_Control_Disable();
					return MR_OK;
				}
			}
			else
				exit_mode_Gcounter = 0;
			
			uint8_t new_ModeButtonZone = get_RcButtonZone( rc.data[4], ModeButtonZone );
			if( ModeButtonZone<=5 && new_ModeButtonZone!=ModeButtonZone )
			{	//模式按钮改变更改模式
				reqMode = MFunc_cfg.Bt1AFunc1[8*new_ModeButtonZone];			
			}
			ModeButtonZone = new_ModeButtonZone;
			
			//使用遥控器更新飞行模式
			bool sticks_in_neutral = 
				in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] );
			if( !sticks_in_neutral )
			{	//摇杆没回中不允许自动操作
				if( is_AFunc_auto(cMode) && ((MFunc_cfg.configs[0]&MCfg_NoRcCtrlInAuto_Bit)==0) )
				{
					if( is_AFunc_auto(MFunc_cfg.Bt1AFunc1[8*ModeButtonZone]) == false )
						reqMode = MFunc_cfg.Bt1AFunc1[8*ModeButtonZone];
					else
						reqMode = AFunc_PosHold;
					MissionButtonZone = RTLButtonZone = 255;
					rcv_Mode = cMode;
					rcvTIME = TIME::now();
				}
				if( rcv_Mode != 255 )
				{	//手动操作时间过长退出恢复
					if( rcvTIME.get_pass_time() > 4.5 )
						rcv_Mode = 255;
				}
			}
			else
			{	//摇杆回中可执行自动操作	
				
				/*回舵恢复模式*/
					bool rcv = false;
					if( rcv_Mode != 255 )
					{
						if( MFunc_cfg.configs[0] & MCfg_RcvAuto_Bit )
						{
							reqMode = rcv_Mode;
							rcv = true;
						}
						rcv_Mode = 255;
					}
				/*回舵恢复模式*/
				
				/*判断执行任务*/
					if( MFunc_cfg.MissionBt[0]>=2 && MFunc_cfg.MissionBt[0]<=4 )
					{	//按钮按下执行任务
						if( rc.available_channels >= MFunc_cfg.MissionBt[0]+4 )
						{			
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.MissionBt[0]-1+4];
							uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );									
							if( new_MissionButtonZone!=MissionButtonZone && !rcv )
							{	//按钮状态发生变化
								if( new_MissionButtonZone>=4 )
									reqMode = AFunc_Mission;
								else if( MissionButtonZone<=5 )
									reqMode = 0;
							}
							MissionButtonZone = new_MissionButtonZone;
						}
					}
					else if( MFunc_cfg.MissionBt[0]>=12 && MFunc_cfg.MissionBt[0]<=14 )
					{	//按钮变化执行任务
						if( rc.available_channels >= MFunc_cfg.MissionBt[0]-10+4 )
						{
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.MissionBt[0]-11+4];
							uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );
							if( MissionButtonZone<=5 && new_MissionButtonZone!=MissionButtonZone && !rcv )
							{	//按钮状态发生变化
								if( cMode != AFunc_Mission )
									reqMode = AFunc_Mission;
								else
									reqMode = 0;
							}
							MissionButtonZone = new_MissionButtonZone;
						}
					}
				/*判断执行任务*/
				
				/*判断返航*/
					if( MFunc_cfg.RTLBt[0]>=2 && MFunc_cfg.RTLBt[0]<=4 )
					{	//按钮按下返航
						if( rc.available_channels >= MFunc_cfg.RTLBt[0]+4 )
						{
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.RTLBt[0]-1+4];
							uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );		
							if( new_RTLButtonZone!=RTLButtonZone && !rcv )
							{	//按钮状态发生变化	
								if( new_RTLButtonZone>=4 )
									reqMode = AFunc_RTL;
								else if( RTLButtonZone<=5 )
									reqMode = 0;
							}
							RTLButtonZone = new_RTLButtonZone;
						}
					}
					else if( MFunc_cfg.RTLBt[0]>=12 && MFunc_cfg.RTLBt[0]<=14 )
					{	//按钮变化返航
						if( rc.available_channels >= MFunc_cfg.RTLBt[0]-10+4 )
						{
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.RTLBt[0]-11+4];
							uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );	
							if( RTLButtonZone<=5 && new_RTLButtonZone!=RTLButtonZone && !rcv )
							{	//按钮状态发生变化
								if( cMode != AFunc_RTL )
									reqMode = AFunc_RTL;
								else
									reqMode = 0;
							}
							RTLButtonZone = new_RTLButtonZone;
						}
					}
				/*判断返航*/
					
				if( reqMode == 0 )
				{	//有按钮松开重新检测按钮位置
					
					/*判断执行任务*/
						if( MFunc_cfg.MissionBt[0]>=2 && MFunc_cfg.MissionBt[0]<=4 )
						{	//按钮按下执行任务
							if( MissionButtonZone>=4 )
								reqMode = AFunc_Mission;
						}
					/*判断执行任务*/
						
					/*判断返航*/
						if( MFunc_cfg.RTLBt[0]>=2 && MFunc_cfg.RTLBt[0]<=4 )
						{	//按钮按下返航
							if( RTLButtonZone>=4 )
								reqMode = AFunc_RTL;
						}
					/*判断返航*/
						
					if( reqMode == 0 )
						reqMode = MFunc_cfg.Bt1AFunc1[8*ModeButtonZone];
				}
			}
		}
		else
		{	//接收机不可用重置遥控状态
			ModeButtonZone = MissionButtonZone = RTLButtonZone = 255;
//			//如果不是自动模式则切换到返航模式
//			if( is_AFunc_auto(cMode)==false )
//				reqMode = AFunc_RTL;
			if( noRcStartTIME.is_valid() == false )
				noRcStartTIME = TIME::now();
			exit_mode_Gcounter = 0;
		}
		
		//if( is_AFunc_auto(reqMode) || is_AFunc_auto(cMode) )
		if( 1 )
		{	//进出自动模式置位mode_swithced
			if( cMode != reqMode )
			{
				cMode = (AFunc)reqMode;
				mode_switched = true;
			}
		}
		else
			cMode = (AFunc)reqMode;
		#define swManualMode if( is_AFunc_auto(MFunc_cfg.Bt1AFunc1[8*ModeButtonZone]) == false )\
														cMode = (AFunc)MFunc_cfg.Bt1AFunc1[8*ModeButtonZone];\
													else\
														cMode = AFunc_PosHold;\
		
		if( cMode!=AFunc_PosHoldNH && cMode!=AFunc_PosHoldNHAv )
			HeadlessYaw = -200;
		
		if( is_MSafeCtrl() )
		{	//当前处于安全模式控制
			/*判断退出模式*/
				if( inFlight==false )
				{
					Attitude_Control_Disable();
					return MR_OK;
				}
			/*判断退出模式*/
		}
		
		if( cMode==AFunc_RTL )
		{	//进入安全模式返航
RTL:
			enter_MSafe(true);
			/*判断退出模式*/
				if( inFlight==false )
				{
					Attitude_Control_Disable();
					return MR_OK;
				}
			/*判断退出模式*/
		}
		else if( cMode==AFunc_TakeOff )
		{	//起飞模式
			if( mode_switched )
			{
				mode_switched = false;
				if( inFlight ){
					swManualMode}
				else
				{
					Position_Control_Enable();
					bool pos_ena;
					is_Position_Control_Enabled(&pos_ena);
					if( pos_ena && param1 )
					{
						Position_Control_set_XYLock();
						float* takeoff_params = (float*)param1;
						if( takeoff_params[0] == 0 )
							Position_Control_Takeoff_HeightGlobal(takeoff_params[1]);
						else if( takeoff_params[0] == 1 )
							Position_Control_Takeoff_HeightRelative(takeoff_params[1]);
						else {
							swManualMode }
					}
					else{
						swManualMode}
				}
			}
			else
			{
				Position_Control_Enable();
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
				if( pos_ena == false )
				{	//位置控制器无法打开返回手动模式
					swManualMode
					goto Manual_Mode;
				}
				
				//上报模式状态
				setCurrentFlyMode(AFunc_TakeOff);
				
				Position_Control_set_XYLock();
				Position_ControlMode alt_mode;
				get_Altitude_ControlMode(&alt_mode);
				if( alt_mode == Position_ControlMode_Position )
				{
					swManualMode
				}
			}			
		}
		else if( cMode==AFunc_RTL2Point )
		{	//返航到指定点模式
			setCurrentFlyMode(AFunc_RTL2Point);
			if( mode_switched )
			{
				mode_switched = false;
				navInf.counter1 = 0;
				
				Position_Control_Enable();
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
				if( pos_ena && param1 )
				{
					Position_Control_set_XYLock();
					double* rtl_params = (double*)param1;
					double lat = rtl_params[0];
					double lon = rtl_params[1];
					double alt = rtl_params[2];
					if( isvalid(alt) )
					{	//海拔信息可用
						if( inFlight )
						{
							//获取最优全球定位传感器信息
							PosSensorHealthInf1 global_inf;
							if( get_OptimalGlobal_Z( &global_inf ) )
							{
								MSafeCfg safeCfg;
								ReadParamGroup( "MSafe", (uint64_t*)&safeCfg, 0 );
								
								double homeZ;
								getHomeLocalZ(&homeZ);
								
								double reqAlt = alt*100 + safeCfg.GbRtHeight[0] - global_inf.HOffset;
								if( reqAlt - homeZ < safeCfg.GbRtHeight[0] )
									reqAlt = homeZ + safeCfg.GbRtHeight[0];
								
								double curAlt = global_inf.PositionENU.z;
								navInf.usr_temp[0] = 100;
								navInf.usr_temp[1] = alt*100 - global_inf.HOffset;
								if( curAlt < reqAlt )
									Position_Control_set_TargetPositionZ( reqAlt );
								else
									Position_Control_set_ZLock();
							}
							else
							{
								navInf.usr_temp[0] = -100;
								MSafeCfg safeCfg;
								ReadParamGroup( "MSafe", (uint64_t*)&safeCfg, 0 );
								
								double RtHeight = safeCfg.LcRtHeight[0];
								double homeZ;
								getHomeLocalZ(&homeZ);
								vector3<double> pos;
								get_Position_Ctrl(&pos);
								if( RtHeight>0 && homeZ+RtHeight>pos.z )
								{	//返航高度大于当前高度才进行升高
									Position_Control_set_TargetPositionZRA( RtHeight );
								}
								else
									Position_Control_set_ZLock();
							}
						}
						else
						{	//未起飞
							PosSensorHealthInf1 global_inf;
							if( get_OptimalGlobal_Z( &global_inf ) )
							{
								navInf.usr_temp[0] = 100;
								navInf.usr_temp[1] = alt*100 - global_inf.HOffset;
								
								MSafeCfg safeCfg;
								ReadParamGroup( "MSafe", (uint64_t*)&safeCfg, 0 );
								
								double homeZ;
								getHomeLocalZ(&homeZ);
								
								double reqAlt = alt*100 + safeCfg.GbRtHeight[0] - global_inf.HOffset;
								if( reqAlt - homeZ < safeCfg.GbRtHeight[0] )
									reqAlt = homeZ + safeCfg.GbRtHeight[0];
								
								Position_Control_Takeoff_Height(reqAlt);
							}
							else
							{	//无机载海拔信息
								navInf.usr_temp[0] = -100;
								MSafeCfg safeCfg;
								ReadParamGroup( "MSafe", (uint64_t*)&safeCfg, 0 );
								
								Position_Control_Takeoff_HeightRelative( safeCfg.LcRtHeight[0] );
							}
						}
					}
					else
					{	//海拔信息不可用
						navInf.usr_temp[0] = -100;
						MSafeCfg safeCfg;
						ReadParamGroup( "MSafe", (uint64_t*)&safeCfg, 0 );
						if( inFlight )
						{
							double RtHeight = safeCfg.LcRtHeight[0];
							double homeZ;
							getHomeLocalZ(&homeZ);
							vector3<double> pos;
							get_Position_Ctrl(&pos);
							if( RtHeight>0 && homeZ+RtHeight>pos.z )
							{	//返航高度大于当前高度才进行升高
								Position_Control_set_TargetPositionZRA( RtHeight );
							}
							else
								Position_Control_set_ZLock();
						}
						else	//未起飞
							Position_Control_Takeoff_HeightRelative( safeCfg.LcRtHeight[0] );
					}
				}
				else	//无定位返回手动模式
					swManualMode 
			}
			else
			{
				switch(navInf.counter1)
				{
					case 0:
					{	//等待高度调整完成
						Position_ControlMode alt_mode;
						get_Altitude_ControlMode(&alt_mode);
						Position_Control_set_XYLock();
						if( alt_mode == Position_ControlMode_Position )
						{	//高度调整完成
							++navInf.counter1;
							//飞往遥控器坐标点
							double* rtl_params = (double*)param1;
							double lat = rtl_params[0];
							double lon = rtl_params[1];
							double alt = rtl_params[2];
							Position_Control_set_TargetPositionXY_LatLon( lat, lon, -1 );
						}
						break;
					}
					case 1:
					{	//等待飞行完成
						Position_Control_set_ZLock();
						Position_ControlMode pos_mode;
						get_Position_ControlMode(&pos_mode);
						if( pos_mode == Position_ControlMode_Position )
						{	//位置调整完成
							++navInf.counter1;
							Position_Control_set_XYLock();
						}
						break;
					}
					case 2:
					{	//降落
						Position_Control_set_XYLock();
						
						double land_vel = getPosCtrlCfg()->LandVel[0];
						if( navInf.usr_temp[0] < 0 )
							Position_Control_set_TargetVelocityZ(-land_vel);
						else
						{	//根据提供的海拔高度计算降落速度
							vector3<double> pos;
							get_Position_Ctrl(&pos);
							double height = pos.z - navInf.usr_temp[1];
							
							//获取降落速度
							float sp[2];
							sp[0] = 200;
							ReadParam("PC_maxAutoVelDn", 0, 0, (uint64_t*)sp, 0 );
							if( sp[0] < 100 )
								sp[0] = 100;
							
							#define max_acc 50.0
							if( height > 5000 + sp[0]*2 )
								Position_Control_set_TargetVelocityZ(-sp[0]);
							else if( height > 5000 )
								Position_Control_set_TargetVelocityZ( - ( (height-5000)*(sp[0]-land_vel)/sp[0]/2 + land_vel ) );
							else
								Position_Control_set_TargetVelocityZ(-land_vel);
						}
						
						bool inFlight;
						get_is_inFlight(&inFlight);
						if( inFlight==false )
						{	//降落完成
							Attitude_Control_Disable();
							return MR_OK;
						}
						break;
					}
				}
			}
		}
		else if( cMode==AFunc_Land )
		{	//降落模式
			
			//上报模式状态
			setCurrentFlyMode(AFunc_Land);
			
			if( mode_switched )
			{	//首先刹车等待								
				Position_Control_set_XYLock();
				Position_Control_set_ZLock();
				Attitude_Control_set_YawLock();
				
				//等待刹车完成
				Position_ControlMode alt_mode, pos_mode;
				get_Altitude_ControlMode(&alt_mode);
				get_Position_ControlMode(&pos_mode);
				if( alt_mode==Position_ControlMode_Position && pos_mode==Position_ControlMode_Position )
				{	//刹车完成
					mode_switched = false;
				}
			}
			else
			{
				/*判断退出模式*/
					//获取飞行状态
					bool inFlight;
					get_is_inFlight(&inFlight);
					//降落后自动加锁
					if( inFlight==false )
					{
						Attitude_Control_Disable();
						return MR_OK;
					}
				/*判断退出模式*/
				
				if( rc.available )
				{
					//油门杆控制垂直速度
					if( in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) )
						Position_Control_set_TargetVelocityZ(-50);
					else
					{
						double thr_stick = remove_deadband( rc.data[0] - 50.0 , (double)MFunc_cfg.NeutralZone[0] );
						if( thr_stick > 0 )
							thr_stick *= getPosCtrlCfg()->maxVelUp[0] / 50;
						else
							thr_stick *= getPosCtrlCfg()->maxVelDown[0] / 50;
						thr_stick -= 50;
						Position_Control_set_TargetVelocityZ(thr_stick);
					}
					
					//偏航杆在中间锁偏航
					//不在中间控制偏航速度
					double YCtrlScale = degree2rad( getAttCtrlCfg()->maxYSp[0] / 50.0 );
					if( in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) )
						Attitude_Control_set_YawLock();
					else
						Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );
						
					if( in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) &&
							in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] ) )
						Position_Control_set_XYLock();
					else
					{
						//计算避障最大速度
						double maxVelXY = getPosCtrlCfg()->maxVelXY[0];
						
						double roll_sitck = rc.data[3] - 50.0;
						double pitch_sitck = rc.data[2] - 50.0;
						double roll_sitck_d = remove_deadband( roll_sitck, (double)MFunc_cfg.NeutralZone[0] );
						double pitch_sitck_d = remove_deadband( pitch_sitck, (double)MFunc_cfg.NeutralZone[0] );
						vector2<double> targetVel;
						double yaw;	double yaw_declination;
						get_YawDeclination(&yaw_declination);
						Attitude_Control_get_TargetTrackYaw(&yaw);
						yaw += yaw_declination;
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						targetVel.x = BodyHeading2ENU_x( pitch_sitck_d , -roll_sitck_d , sin_Yaw , cos_Yaw );
						targetVel.y = BodyHeading2ENU_y( pitch_sitck_d , -roll_sitck_d , sin_Yaw , cos_Yaw );
						
						vector3<double> velocityEnu;
						get_VelocityENU_Ctrl(&velocityEnu);
						
						double RPCtrlScale = atan2( getPosCtrlCfg()->maxAccXY[0] / 50.0, GravityAcc );
						double XYCtrlScale = maxVelXY / 50.0;						
						double vel_stick_err = safe_sqrt( sq(velocityEnu.x/XYCtrlScale - targetVel.x) + sq(velocityEnu.y/XYCtrlScale - targetVel.y) );
						if( vel_stick_err > 50 )
							vel_stick_err = 50;
						targetVel *= XYCtrlScale;
						targetVel.constrain(maxVelXY);
						Position_Control_set_TargetVelocityXY_AngleLimit(
							targetVel.x ,
							targetVel.y ,
							vel_stick_err*RPCtrlScale
						);
					}
				}
				else
				{
					Position_Control_set_XYLock();
					Position_Control_set_TargetVelocityZ(-50);
					Attitude_Control_set_YawLock();
				}
			}
		}
		else if( cMode==AFunc_Mission )
		{	//任务模式
			Position_Control_Enable();
			bool pos_ena;
			is_Position_Control_Enabled(&pos_ena);
			if( pos_ena == false )
			{	//位置控制器无法打开返回手动模式
				swManualMode
				goto Manual_Mode;
			}
			
			if( rc.available==false && (MFunc_cfg.configs[0]&MCfg_WpNRcRTL_Bit) )
			{
				//无遥控信号进入安全模式
				change_Mode(AFunc_RTL)
				goto RTL;
			}
			
			if( MFunc_cfg.configs[0] & MCfg_NoAutoWithouFixed_Bit )
			{	//无fix不允许自动
				bool getFixed = false;
				Position_Sensor gps_sensor;
				if( GetPositionSensor( default_rtk_sensor_index, &gps_sensor ) && gps_sensor.inf.addition_inf[1]==6 )
					getFixed = true;
				else if( GetPositionSensor( default_gps_sensor_index, &gps_sensor ) && gps_sensor.inf.addition_inf[1]==6 )
					getFixed = true;
				if( !getFixed )
				{	//无FIX返航
					change_Mode(AFunc_RTL)
					goto RTL;
				}
			}
			
			if(msg_available)
			{
				if( check_NavCmd( msg.cmd, freq, default_NavCmd_frame, msg.params ) )
				{	//指令可被执行
					//前往手动模式执行指令
					cMode = AFunc_PosHold;
					goto Manual_Mode;
				}
			}
			
			if( mode_switched )
			{	//刚切换到任务模式
				//首先刹车等待								
				Position_Control_set_XYLock();
				Position_Control_set_ZLock();
				Attitude_Control_set_YawLock();
				
				//复位拍照间距
				CamTriggDist = 0;
				
				//等待刹车完成
				Position_ControlMode alt_mode, pos_mode;
				get_Altitude_ControlMode(&alt_mode);
				get_Position_ControlMode(&pos_mode);
				if( alt_mode==Position_ControlMode_Position && pos_mode==Position_ControlMode_Position )
				{	//刹车完成
					++navInf.counter2;
					//等待1秒再进入任务飞行
					if( navInf.counter2 >= 1*freq )
					{
						mode_switched = false;					
						if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
						{	//载入下一航点成功
							//初始化任务信息
							init_NavCmdInf(&navInf);
							if( current_mission_ind == restoreWpInf.CurrentWp[0] )
								//恢复航线飞行
								MissionMode_BackToLastWp = 0;
							else
							{	//当前和记录的航点直接前往
								//复位当前航点信息
								reset_RestoreWpInf(restoreWpInf);
								MissionMode_BackToLastWp = 0;
							}
						}
						else
						{	//获取不到航点信息
							//先试着把航点设置为首个
							setCurrentMission(0);
							//复位当前航点信息
							reset_RestoreWpInf(restoreWpInf);
							MissionMode_BackToLastWp = 0;
							if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
							{	//载入下一航点成功
								//初始化任务信息
								init_NavCmdInf(&navInf);
							}
							else
							{	//无航点信息返回手动模式
								swManualMode
								goto Manual_Mode;
							}
						}
						
						if( current_mission_ind == 0 )
						{	//航点为首个时首先排除航线头指令和不能识别的航点
							reset_RestoreWpInf(restoreWpInf);
							
							while(1)
							{
								if( Process_InflightCmd( current_mission_inf.cmd, current_mission_inf.params ) )
								{	//尝试执行指令成功
									if( setCurrentMission( getCurrentMissionInd() + 1 ) )
									{
										if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
										{
											continue;
										}
										else
										{	//无航点信息返回手动模式
											setCurrentMission( 0 );
											swManualMode
											goto Manual_Mode;
										}
									}
									else
									{	//无航点信息返回手动模式
										setCurrentMission( 0 );
										swManualMode
										goto Manual_Mode;
									}
								}
								else
								{	//尝试排除无效指令
									if( check_NavCmd(current_mission_inf.cmd,
											freq, 
											current_mission_inf.frame,
											current_mission_inf.params
										) )
									{	//指令可执行退出
										restoreWpInf.CurrentWp[0] = current_mission_ind;
										break;
									}
									else
									{	//命令不可执行
										//跳过
										if( setCurrentMission( getCurrentMissionInd() + 1 ) )
										{
											if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
											{
												continue;
											}
											else
											{	//无航点信息返回手动模式
												setCurrentMission( 0 );
												swManualMode
												goto Manual_Mode;
											}
										}
										else
										{	//无航点信息返回手动模式
											setCurrentMission( 0 );
											swManualMode
											goto Manual_Mode;
										}
									}
								}
							}
						}
					}
				}
				else
					navInf.counter2 = 0;
			}
			else if( MissionMode_BackToLastWp != 255 )
			{	//首先飞到上次航线飞行位置
				
				//上报模式状态
				setCurrentFlyMode(AFunc_Mission);
				
				double AB_length = safe_sqrt( sq(restoreWpInf.line_x) + sq(restoreWpInf.line_y) + sq(restoreWpInf.line_z) );
				if( current_mission_inf.cmd!=MAV_CMD_NAV_WAYPOINT )
				{	//不需要恢复直接进入任务飞行							
					MissionMode_BackToLastWp = 255;
				}
				else
				{	//需要恢复到上次飞行位置
					switch( MissionMode_BackToLastWp )
					{
						case 0:
						{	//没起飞先起飞
							if( restoreWpInf.line_fs < 0 )
								restoreWpInf.line_fs = 0;
							if(inFlight)
							{
								CamTriggDist = restoreWpInf.CamTrigDist;
								++MissionMode_BackToLastWp;
							}
							else
							{
								Position_Control_set_XYLock();
								Position_Control_set_TargetVelocityZ(50);
							}
							break;
						}
						case 1:
						{	//高度调整
							//锁定xy
							Position_Control_set_XYLock();							
							//求Z偏移距离
							double z_offset = 0;
							if( AB_length > 0.1 )
							{
								double inv_AB_length = 1.0 / AB_length;
								double line_fs;
								if( restoreWpInf.line_fs > 0 )
									line_fs = restoreWpInf.line_fs;
								else
									line_fs = 0;
								double ufs = AB_length - line_fs;
								if( ufs < 0 )
									ufs = 0;
								z_offset += ufs * restoreWpInf.line_z*inv_AB_length;
							}
							double params[7] = {0};
							params[3] = std::nan("");
							params[6] = current_mission_inf.params[6] + z_offset*0.01;
							vector3<double> OC;
							NavCmd16_WAYPOINT_GetInfo( current_mission_inf.frame, params, 0, &OC, 0 );
							//执行结果
							if( OC.z > 0 )
							{
								wpRNeedDecent = false;
								bool res = false;
								switch(current_mission_inf.frame)
								{
									case MAV_FRAME_GLOBAL_INT:
									case MAV_FRAME_GLOBAL:
									{
										res = Position_Control_set_TargetPositionZGlobal( current_mission_inf.params[6]*100 + z_offset, 0 );
										break;
									}
									case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
									case MAV_FRAME_GLOBAL_RELATIVE_ALT:
									{
										res = Position_Control_set_TargetPositionZRA( current_mission_inf.params[6]*100 + z_offset, 0 );
										break;
									}
									case MAV_FRAME_GLOBAL_TERRAIN_ALT:
									case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
									{
										res = Position_Control_set_TargetPositionZRA( current_mission_inf.params[6]*100 + z_offset, 0 );
										break;
									}
										
									case MAV_FRAME_LOCAL_NED:
										res = Position_Control_set_TargetPositionZ( -(current_mission_inf.params[6]*100 + z_offset), 0 );
										break;
										
									case MAV_FRAME_LOCAL_ENU:
										res = Position_Control_set_TargetPositionZ( current_mission_inf.params[6]*100 + z_offset, 0 );
										break;
										
									case MAV_FRAME_BODY_NED:
									case MAV_FRAME_BODY_FRD:
									case MAV_FRAME_BODY_OFFSET_NED:
									case MAV_FRAME_LOCAL_OFFSET_NED:
										res = Position_Control_set_TargetPositionZRelative( -(current_mission_inf.params[6]*100 + z_offset), 0 );
										break;
										
										
									case MAV_FRAME_BODY_FLU:
									{
										res = Position_Control_set_TargetPositionZRelative( current_mission_inf.params[6]*100 + z_offset, 0 );
										break;
									}
										
									default:
										MissionMode_BackToLastWp = 255;
								}
									
								if(res)
									++MissionMode_BackToLastWp;
								else
									MissionMode_BackToLastWp = 255;
							}
							else
							{
								wpRNeedDecent = true;
								MissionMode_BackToLastWp += 2;
							}
							break;
						}
						
						case 2:
						{	//等待高度调整完成
							//锁定xy
							Position_Control_set_XYLock();
							Position_ControlMode alt_mode;
							get_Altitude_ControlMode(&alt_mode);
							if( alt_mode == Position_ControlMode_Position )
								++MissionMode_BackToLastWp;
							break;
						}
						
						case 3:
						{	//旋转偏航
														
							//锁定XYZ
							Position_Control_set_XYLock();
							Position_Control_set_ZLock();
							//求AB向量长度倒数
							double offset_x=0, offset_y=0;
							if( AB_length > 0.1 )
							{
								double inv_AB_length = 1.0 / AB_length;
								double line_fs;
								if( restoreWpInf.line_fs > 0 )
									line_fs = restoreWpInf.line_fs;
								else
									line_fs = 0;
								double ufs = AB_length - line_fs;
								if( ufs < 0 )
									ufs = 0;
								offset_x = ufs * restoreWpInf.line_x*inv_AB_length;
								offset_y = ufs * restoreWpInf.line_y*inv_AB_length;
							}
							
							double LA, LB;
							switch(current_mission_inf.frame)
							{
								case MAV_FRAME_GLOBAL:
								case MAV_FRAME_GLOBAL_RELATIVE_ALT:
								case MAV_FRAME_GLOBAL_INT:
								case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
								case MAV_FRAME_GLOBAL_TERRAIN_ALT:
								case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
								{	//全球定位
									if( current_mission_inf.params[4]<-90 ||  current_mission_inf.params[4]> 90 
										|| current_mission_inf.params[5]<-180 || current_mission_inf.params[5]>180 )
									{	//经纬度为不正确不转偏航
										MissionMode_BackToLastWp += 2;
										goto ModeLoopFin;
									}
									
									//获取最优全球定位传感器信息
									PosSensorHealthInf2 global_inf;
									if( get_OptimalGlobal_XY( &global_inf ) == false )
									{
										MissionMode_BackToLastWp = 255;
										goto ModeLoopFin;
									}
									//获取指定经纬度平面坐标
									double x, y;
									map_projection_project( &global_inf.mp, current_mission_inf.params[4], current_mission_inf.params[5], &x, &y );
									x -= global_inf.HOffset.x;
									y -= global_inf.HOffset.y;
									x += offset_x;
									y += offset_y;
									LA = y - global_inf.PositionENU.y;
									LB = x - global_inf.PositionENU.x;
									break;
								}
								
								case MAV_FRAME_LOCAL_NED:
								{
									vector3<double> position;
									get_Position_Ctrl(&position);
									double x, y;
									x = current_mission_inf.params[5]*100;
									y = current_mission_inf.params[4]*100;
									x += offset_x;
									y += offset_y;
									LA = y - position.y;
									LB = x - position.x;
									break;
								}
								
								case MAV_FRAME_LOCAL_ENU:
								{
									vector3<double> position;
									get_Position_Ctrl(&position);
									double x, y;
									x = current_mission_inf.params[4]*100;
									y = current_mission_inf.params[5]*100;
									x += offset_x;
									y += offset_y;
									LA = y - position.y;
									LB = x - position.x;
									break;
								}
								
								default:
									MissionMode_BackToLastWp = 255;
									goto ModeLoopFin;
							}
							
							if( sq(LA) + sq(LB) > sq(500) )
								Attitude_Control_set_Target_Yaw( atan2(LA,LB) );
							++MissionMode_BackToLastWp;
								
							break;
						}
						
						case 4:
						{	//等待偏航旋转开始航点飞行
							Position_Control_set_XYLock();
							Position_Control_set_ZLock();
							double yawTrackErr;
							Attitude_Control_get_YawTrackErr(&yawTrackErr);
							if( yawTrackErr < 0.01 )
							{						
								//求AB向量长度倒数
								double offset_x=0, offset_y=0;
								if( AB_length > 0.1 )
								{
									double inv_AB_length = 1.0 / AB_length;
									double line_fs;
									if( restoreWpInf.line_fs > 0 )
										line_fs = restoreWpInf.line_fs;
									else
										line_fs = 0;
									double ufs = AB_length - line_fs;
									if( ufs < 0 )
										ufs = 0;
									offset_x = ufs * restoreWpInf.line_x*inv_AB_length;
									offset_y = ufs * restoreWpInf.line_y*inv_AB_length;
								}
								double Tx, Ty;
								switch(current_mission_inf.frame)
								{
									case MAV_FRAME_GLOBAL:
									case MAV_FRAME_GLOBAL_RELATIVE_ALT:
									case MAV_FRAME_GLOBAL_INT:
									case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
									case MAV_FRAME_GLOBAL_TERRAIN_ALT:
									case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
									{	//全球定位
										if( current_mission_inf.params[4]<-90 ||  current_mission_inf.params[4]> 90 
											|| current_mission_inf.params[5]<-180 || current_mission_inf.params[5]>180 )
										{	//经纬度为不正确退出
											MissionMode_BackToLastWp = 255;
											goto ModeLoopFin;
										}
										
										//获取最优全球定位传感器信息
										PosSensorHealthInf2 global_inf;
										if( get_OptimalGlobal_XY( &global_inf ) == false )
										{
											MissionMode_BackToLastWp = 255;
											goto ModeLoopFin;
										}
										//获取指定经纬度平面坐标
										double x, y;
										map_projection_project( &global_inf.mp, current_mission_inf.params[4], current_mission_inf.params[5], &x, &y );
										x -= global_inf.HOffset.x;
										y -= global_inf.HOffset.y;
										x += offset_x;
										y += offset_y;
										Tx = x;	Ty = y;
										break;
									}
									
									case MAV_FRAME_LOCAL_NED:
									{
										double x, y;
										x = current_mission_inf.params[5]*100;
										y = current_mission_inf.params[4]*100;
										x += offset_x;
										y += offset_y;
										Tx = x;	Ty = y;
										break;
									}
									
									case MAV_FRAME_LOCAL_ENU:
									{
										double x, y;
										x = current_mission_inf.params[4]*100;
										y = current_mission_inf.params[5]*100;
										x += offset_x;
										y += offset_y;
										Tx = x;	Ty = y;
										break;
									}
									
									default:
										MissionMode_BackToLastWp = 255;
										goto ModeLoopFin;
								}
								
								bool res = Position_Control_set_TargetPositionXY( Tx, Ty, 0 );
								if(res)
									++MissionMode_BackToLastWp;
								else
									MissionMode_BackToLastWp = 255;
							}
							break;
						}
						
						case 5:
						{	//等待航线飞行完成
							
							//获取避障参数
							double AvDist = -1;
							const AvoidanceCfg* avCfg = getAvCfg();
							if( AvMode_Enabled(avCfg->AvoidanceMode[0]) )
							{	//需要进行避障
								double AvDistance = -1;
								vector3<double> lineAB;
								Position_Control_get_LineFlightABDistance( &lineAB, 0 );
								get_AvLineDistanceEnu( &AvDistance, -lineAB, avCfg->wheelbase[0] );
								if( AvDistance>=0 )
								{
									AvDistance -= avCfg->AvoidanceDist[0] + 0.5f*avCfg->wheelbase[0];
									if( AvDistance < 0 )
										AvDistance = 0;
									AvDist = AvDistance;
								}
							}
							
							bool terrain_mode = false;
							if( current_mission_inf.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT || current_mission_inf.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT_INT ) 
							{
								terrain_mode = true;
								
								PosSensorHealthInf1 pos_inf;
								get_OptimalRange_Z(&pos_inf);
								Position_Sensor_Data sensor;
								if( GetPositionSensorData(pos_inf.sensor_ind, &sensor) && isvalid(current_mission_inf.params[6]) ) 
								{	//根据测高传感器调整对地高度
									vector3<double> pos;
									get_Position_Ctrl(&pos);
									double target_height;
									target_height = current_mission_inf.params[6];
										
									double err = target_height - sensor.position.z;
									Position_Control_set_TargetVelocityZ( constrain(err, 200.0) );
									
									//根据对地高度限速
									double maxFrontPointL = -1;
									double abs_err = fabs(err);
									
									double errLimit = getPosCtrlCfg()->maxVelXY[0] - 3*(abs_err-50);
									double groundLimit = 1.0*(sensor.position.z - 20);
									if( groundLimit < 20 )
										groundLimit = 20;
									
									if( errLimit < groundLimit )
										maxFrontPointL = errLimit;
									else
										maxFrontPointL = groundLimit;
									
									if( maxFrontPointL>=0 && ( AvDist<0 || maxFrontPointL<AvDist ) )
										AvDist = maxFrontPointL;
								}
								else
								{	//无高度传感器锁定高度
									Position_Control_set_ZLock();
								}
							}
							else 
							{	//不需要仿地
								Position_Control_set_ZLock();
							}
							
							//设置避障距离
							Position_Control_set_RouteLineAvoidanceRelative( AvDist );
						
							Position_ControlMode pos_mode;
							get_Position_ControlMode(&pos_mode);
							if( pos_mode == Position_ControlMode_Position )
							{	//已成功移动到上次航线位置
								MissionMode_BackToLastWp = 255;
							}
							break;
						}
						
						case 6:
						{	//高度调整
							//锁定xy
							Position_Control_set_XYLock();							
							//求Z偏移距离
							double z_offset = 0;
							if( AB_length > 0.1 )
							{
								double inv_AB_length = 1.0 / AB_length;
								double line_fs;
								if( restoreWpInf.line_fs > 0 )
									line_fs = restoreWpInf.line_fs;
								else
									line_fs = 0;
								double ufs = AB_length - line_fs;
								if( ufs < 0 )
									ufs = 0;
								z_offset += ufs * restoreWpInf.line_z*inv_AB_length;
							}
							bool res = false;
							switch(current_mission_inf.frame)
							{
								case MAV_FRAME_GLOBAL_INT:
								case MAV_FRAME_GLOBAL:
								{
									res = Position_Control_set_TargetPositionZGlobal( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								}
								case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
								case MAV_FRAME_GLOBAL_RELATIVE_ALT:
								{
									res = Position_Control_set_TargetPositionZRA( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								}
								case MAV_FRAME_GLOBAL_TERRAIN_ALT:
								case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
								{
									res = Position_Control_set_TargetPositionZRA( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								}
								
								case MAV_FRAME_LOCAL_NED:
									res = Position_Control_set_TargetPositionZ( -(current_mission_inf.params[6]*100 + z_offset), 0 );
									break;
								
								case MAV_FRAME_LOCAL_ENU:
									res = Position_Control_set_TargetPositionZ( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								
								case MAV_FRAME_BODY_NED:
								case MAV_FRAME_BODY_FRD:
								case MAV_FRAME_BODY_OFFSET_NED:
								case MAV_FRAME_LOCAL_OFFSET_NED:
									res = Position_Control_set_TargetPositionZRelative( -(current_mission_inf.params[6]*100 + z_offset), 0 );
									break;
								
								
								case MAV_FRAME_BODY_FLU:
								{
									res = Position_Control_set_TargetPositionZRelative( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								}
								
								default:
									MissionMode_BackToLastWp = 255;
							}
							
							if(res)
								++MissionMode_BackToLastWp;
							else
								MissionMode_BackToLastWp = 255;
							break;
						}
						
						case 7:
						{	//等待高度调整完成
							//锁定xy
							Position_Control_set_XYLock();
							Position_ControlMode alt_mode;
							get_Altitude_ControlMode(&alt_mode);
							if( alt_mode == Position_ControlMode_Position )
								MissionMode_BackToLastWp = 255;
							break;
						}
						
						default:
							MissionMode_BackToLastWp = 255;
							break;
					}
				}
			}
			else
			{	//任务飞行
				
				//任务需要保存
				set_RestoreWpInf_needStore();
				
				//上报模式状态
				setCurrentFlyMode(AFunc_Mission);
				
				int16_t res = -100;
				if( current_mission_inf.cmd == 177 )
				{
					if( doJumpWP != getCurrentMissionInd() )
					{
						doJumpWP = getCurrentMissionInd();
						doJumpCount = 0;
					}
					++doJumpCount;
					if( doJumpCount > current_mission_inf.params[1] )
						res = -1;
					else
					{
						if( current_mission_inf.params[0] > 0 )
							res = current_mission_inf.params[0] - 1;
						else
							res = 0;
					}
				}
				else if( Process_InflightCmd( current_mission_inf.cmd, current_mission_inf.params ) == false )
				{	//尝试执行航点任务
					if( navInf.usr_temp[ctMissionState] == 0 )
					{	//连续任务未开始
						missionTraveledDist = 0;
						switch(current_mission_inf.cmd)
						{
							case MAV_CMD_NAV_WAYPOINT:	//航线任务
							{
								init_NavCmdInf(&navInf);
								navInf.counter1 = 0;
								navInf.usr_temp[ctMissionState] = 1;
								
								res = -3;
								break;
							}
						}
					}
					
					if( navInf.usr_temp[ctMissionState] == 0 )
					{	//不进行连续任务
						//直接执行指令
						res = Process_NavCmd(
							current_mission_inf.cmd,
							freq, 
							current_mission_inf.frame,
							current_mission_inf.params,
							&navInf
						);
					}
					else
					{	//需要进行连续飞行任务
						
						if( CamTriggDist > 0 )
						{	//积分求飞行路程
							vector3<double> vel;
							get_VelocityENU_Ctrl(&vel);
							missionTraveledDist += safe_sqrt(vel.get_square()) * h;
						}
						else
						{
							navInf.usr_temp[CamTriggDistMult] = 0;
							missionTraveledDist = 0;
						}
						
						if( navInf.usr_temp[ctMissionState] < 100 )
						{	//执行任务
							res = -3;
							switch(current_mission_inf.cmd)
							{
								case MAV_CMD_NAV_WAYPOINT:
								{	//航线任务
									if( navInf.counter1 == 0 )
									{	//旋转偏航
										Position_Control_set_XYLock();
										Position_Control_set_ZLock();
										Attitude_Control_set_YawLock();
										
										//按指令旋转机头
										if( isvalid(current_mission_inf.params[3]) == false )
										{	//机头指向航点方向
											
											//获取当前航点Local坐标
											vector3<double> OC;
											NavCmd16_WAYPOINT_GetInfo( current_mission_inf.frame, current_mission_inf.params, 0, &OC, 0 );
											
											if( sq(OC.x) + sq(OC.y) > sq(5) )
												Attitude_Control_set_Target_Yaw( atan2(OC.y,OC.x) );
										}
										else if( current_mission_inf.params[3]>-360 && current_mission_inf.params[3]<360 )
										{	//指定偏航朝向
											Attitude_Control_set_Target_Yaw( degree2rad(90 - current_mission_inf.params[3]) );
										}
										
										navInf.counter1 = 1;
									}
									else if( navInf.counter1 == 1)
									{	//等待偏航旋转完成
										
										Position_Control_set_XYLock();
										Position_Control_set_ZLock();
										
										double yawTrackErr;
										Attitude_Control_get_YawTrackErr(&yawTrackErr);
										if( yawTrackErr < 0.01 )
											navInf.counter1 = 100;
									}
									else if( navInf.counter1 == 50 )
									{	//开始航线飞行(继续任务)
										
										//设置转弯半径
										Position_Control_set_TurnRaius(current_mission_inf.params[1]*100);
										if( current_mission_inf.params[1]>0 && current_mission_inf.params[0]<0.1 )
										{	//切换到航线预存
											navInf.usr_temp[ctMissionState] = 100;
										}
										else
										{	//不预存航线
											navInf.usr_temp[ctMissionState] = 3;
										}
										navInf.counter1 = 101;
									}
									else if( navInf.counter1 == 100 )
									{	//开始航线飞行
										
										//获取当前航点Local坐标
										vector3<double> pC;
										NavCmd16_WAYPOINT_GetInfo( current_mission_inf.frame, current_mission_inf.params, 0, 0, &pC );
										
										//开始航线任务
										if( current_mission_inf.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT || current_mission_inf.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT_INT )
										{	//仿地航线
											Position_Control_set_TargetPositionXY( pC.x, pC.y, 0 );
											Position_Control_set_ZLock();
											
											if( isvalid(current_mission_inf.params[6]) == false )	
											{	//高度不正确
												//保持当前对地高度飞行
												//获取对地传感器高度
												PosSensorHealthInf1 pos_inf;
												get_OptimalRange_Z(&pos_inf);
												Position_Sensor_Data sensor;
												if( GetPositionSensorData(pos_inf.sensor_ind, &sensor) && isvalid(current_mission_inf.params[6]) )
													navInf.temp[0] = sensor.position.z;
												else
												{	//无定位锁定Z
													Position_Control_set_ZLock();
													navInf.temp[0] = -11;
												}
											}
											else
											{	//按提供的高度飞行
												navInf.temp[0] = -1;
											}
										}
										else
										{	//三维航线
											Position_Control_set_TargetPositionXYZ( pC.x, pC.y, pC.z, 0 );
										}
										
										//设置转弯半径
										Position_Control_set_TurnRaius(current_mission_inf.params[1]*100);
										
										if( current_mission_inf.params[1]>0 && current_mission_inf.params[0]<0.1 )
										{	//切换到航线预存
											navInf.usr_temp[ctMissionState] = 100;
										}
										else
										{	//不预存航线
											navInf.usr_temp[ctMissionState] = 3;
										}
										++navInf.counter1;
									}
									else if( navInf.counter1 == 101 )
									{	//等待航线飞行完成
										
										//获取航线飞行状态
										Position_ControlMode alt_mode, pos_mode;
										double frontDir;
										vector3<double> frontVec;
										get_Altitude_ControlMode(&alt_mode);
										get_Position_ControlMode(&pos_mode, &frontDir, &frontVec);
										
										//定距拍照
										if( CamTriggDist > 0 )
										{
											if( missionTraveledDist >= 0 )
											{
												int mult = (int)(missionTraveledDist / CamTriggDist) + 1;
												if( mult > navInf.usr_temp[CamTriggDistMult] )
												{
													AuxCamTakePhotoAsync();
													navInf.usr_temp[CamTriggDistMult] = mult;
												}
											}
										}
										
										//存储飞行状态
										if( pos_mode == Position_ControlMode_RouteLine3D )
										{	//直线飞行
											vector3<double> line_AB;
											double flightDistance = -1;
											if( Position_Control_get_LineFlightABDistance( &line_AB, &flightDistance ) )
											{
												restoreWpInf.CurrentWp[0] = current_mission_ind;
												restoreWpInf.line_x = line_AB.x;
												restoreWpInf.line_y = line_AB.y;
												restoreWpInf.line_z = line_AB.z;
												restoreWpInf.line_fs = flightDistance;
												restoreWpInf.wpR = current_mission_inf.params[1]*100;
												restoreWpInf.CamTrigDist = CamTriggDist;
											}
										}
										
										if( isvalid(current_mission_inf.params[3]) == false )
										{	//需要调整航向
											switch(pos_mode)
											{
												case Position_ControlMode_RouteLine:
												case Position_ControlMode_RouteLine3D:
												case Position_ControlMode_MissionEndArc3D:
												case Position_ControlMode_MissionEndArc:
												{
													Attitude_Control_set_Target_Yaw(frontDir);
													break;
												}
												default:
												{
													break;
												}
											}
										} 
										
										//获取避障参数
										double AvDistance = -1;
										double AvDist = -1;
										const AvoidanceCfg* avCfg = getAvCfg();
										if( Is_AutoMode(pos_mode) )
										{	//航线避障
											if( AvMode_Enabled(avCfg->AvoidanceMode[0]) )
											{	//需要进行避障
												get_AvLineDistanceEnu( &AvDistance, frontVec, avCfg->wheelbase[0] );
												if( AvDistance>=0 )
												{
													AvDist = AvDistance - (avCfg->AvoidanceDist[0] + 0.5f*avCfg->wheelbase[0]);
													if( AvDist < 0 )
														AvDist = 0;
												}
											}
										}
										
										if( current_mission_inf.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT || current_mission_inf.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT_INT )
										{	//仿地航线
											PosSensorHealthInf1 pos_inf;
											get_OptimalRange_Z(&pos_inf);
											Position_Sensor_Data sensor;
											if( GetPositionSensorData(pos_inf.sensor_ind, &sensor) && isvalid(current_mission_inf.params[6]) ) 
											{	//根据测高传感器调整对地高度
												vector3<double> pos;
												get_Position_Ctrl(&pos);
												double target_height;
												if( isvalid(current_mission_inf.params[6]) && current_mission_inf.params[6]*100>5 )
													target_height = current_mission_inf.params[6]*100;
												else if( navInf.temp[0] > 5 )
													target_height = navInf.temp[0];
												else
													target_height = 500;
												
												//计算当前等效高度
												double current_height;
												if( AvDistance < 0 )
												{	//前向距离不可用
													current_height = sensor.position.z;
												}
												else
												{	//使用前向和下向最小距离
													if( AvDistance < sensor.position.z )
														current_height = AvDistance;
													else
														current_height = sensor.position.z;
												}
												
												//计算目标速度
												double err = target_height - current_height;
												double maxVel;
												if( err > 0 )
													maxVel = getPosCtrlCfg()->maxAutoVelUp[0]*0.7;
												else
													maxVel = getPosCtrlCfg()->maxAutoVelDown[0]*0.7;
												Position_Control_set_TargetVelocityZ( constrain(err, 150.0) );
												
												double vv[3];
												vv[0] = sensor.position.z;
												vv[1] = AvDistance;
												vv[2] = current_height;
												SDLog_Msg_DebugVect( "bzz", vv, 3 );
												
												//根据对地高度限速
												double maxFrontPointL = -1;
												double abs_err = fabs(err);
												double errLimit = -1;
												double errLimitRange = target_height * 0.1;
												if( abs_err > errLimitRange + 0.1 )
												{
													errLimit = errLimitRange / (abs_err-errLimitRange);
													if( errLimit < 0.3 )
														errLimit = 0;
													else
														errLimit *= 5;
												}
												double groundLimit = 1.0*(current_height - 20);
												if( groundLimit < 20 )
													groundLimit = 20;
												
												if( errLimit>=0 && errLimit<groundLimit )
													maxFrontPointL = errLimit;
												else
													maxFrontPointL = groundLimit;
												
												if( maxFrontPointL>=0 && ( AvDist<0 || maxFrontPointL<AvDist ) )
													AvDist = maxFrontPointL;
											}
											else
											{	//无高度传感器锁定高度
												Position_Control_set_ZLock();
											}
										}
										else
										{	//三维航线
											
										}
										
										//设置避障距离
										Position_Control_set_RouteLineAvoidanceRelative( AvDist );
										
										if( navInf.usr_temp[ctMissionState] == 2 )
										{	//曾预存过航点
											if( get_prestoreMissionState() == false )
											{	//当前航点已飞完
												//切换到任务跳转
												navInf.usr_temp[ctMissionState] = 110;
											}
										}
										else if( navInf.usr_temp[ctMissionState] == 3 )
										{	//无预存航点(继续)
											if( pos_mode == Position_ControlMode_Position )
											{	//航线飞行已完成
												Position_Control_set_XYLock();
												Position_Control_set_ZLock();
												if( ++(navInf.counter2) >= freq*current_mission_inf.params[0] )
												{	//当前航点已飞完
													//切换到任务跳转
													navInf.usr_temp[ctMissionState] = 111;
												}
											}
										}
										else
										{	//无预存航点(航线结束)
											if( pos_mode == Position_ControlMode_Position )
											{	//航线飞行已完成
												Position_Control_set_XYLock();
												Position_Control_set_ZLock();
												if( ++(navInf.counter2) >= freq*current_mission_inf.params[0] )
												{	//连续任务结束
													res = -1;
													init_NavCmdInf(&navInf);
													navInf.usr_temp[ctMissionState] = 0;
												}
											}
										}
									}
									
									break;
								}
								
								default:
								{	//无连续飞行任务
									init_NavCmdInf(&navInf);
									res = Process_NavCmd(
										current_mission_inf.cmd,
										freq, 
										current_mission_inf.frame,
										current_mission_inf.params,
										&navInf
									);
									break;
								}
							}
						}
						else if( navInf.usr_temp[ctMissionState] == 100 )
						{	//航点预存
							uint16_t rdWpInd = getCurrentMissionInd() + 1;
							while(1)
							{
								//读取下一个航点
								MissionInf rdMissionInf;
								if( ReadMission( rdWpInd, &rdMissionInf ) == false )
								{
									navInf.usr_temp[ctMissionState] = 1;	//未预存航线
									break;
								}
								if( rdMissionInf.cmd == 177 )
								{	//首先检查航点跳转
									if( (doJumpWP==rdWpInd && doJumpCount<rdMissionInf.params[1]) || (doJumpWP!=rdWpInd && 0<rdMissionInf.params[1]) )
									{
										rdWpInd = rdMissionInf.params[0] - 1;
										continue;
									}
								}
								else if( check_NavCmd(rdMissionInf.cmd,
										freq, 
										rdMissionInf.frame,
										rdMissionInf.params
									) )
								{	//指令可执行
									switch(rdMissionInf.cmd)
									{
										case MAV_CMD_NAV_WAYPOINT:
										{	//航线
											vector3<double> pD;
											NavCmd16_WAYPOINT_GetInfo( rdMissionInf.frame, rdMissionInf.params, 0, 0, &pD );
											
											//预存航线任务
											bool prestored = false;
											if( current_mission_inf.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT || current_mission_inf.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT_INT )
											{	//仿地航线
												Position_Control_set_TargetPositionXY( pD.x, pD.y, 0, MissionPrestoreMode_Prestore, &prestored );
												
												if( isvalid(rdMissionInf.params[6]) == false )	
												{	//高度不正确
													//保持当前对地高度飞行
													//获取对地传感器高度
													PosSensorHealthInf1 pos_inf;
													get_OptimalRange_Z(&pos_inf);
													Position_Sensor_Data sensor;
													if( GetPositionSensorData(pos_inf.sensor_ind, &sensor) && isvalid(rdMissionInf.params[6]) )
														navInf.temp[0] = sensor.position.z;
													else
													{	//无定位锁定Z
														navInf.temp[0] = -11;
													}
												}
												else
												{	//按提供的高度飞行
													navInf.temp[0] = -1;
												}
											}
											else
											{	//三维航线
												Position_Control_set_TargetPositionXYZ( pD.x, pD.y, pD.z, 0, MissionPrestoreMode_Prestore, &prestored );
											}
											if(prestored)
												navInf.usr_temp[ctMissionState] = 2;	//已预存航线
											else
												navInf.usr_temp[ctMissionState] = 1;	//未预存航线
											break;
										}
										
										default:
										{
											navInf.usr_temp[ctMissionState] = 1;	//未预存航线
											break;
										}
									}
									
									break;
								}
								
								++rdWpInd;
							}
							res = -3;
						}
						else if( navInf.usr_temp[ctMissionState]==110 || navInf.usr_temp[ctMissionState]==111 )
						{	//任务跳转
							while(1)
							{
								if( setCurrentMission( getCurrentMissionInd() + 1 ) )
								{
									doJumpReadMission:
									if( ReadCurrentMission( &current_mission_inf, &current_mission_ind ) )
									{
										if( current_mission_inf.cmd == 177 )
										{	//首先处理航点跳转
											if( doJumpWP != getCurrentMissionInd() )
											{
												doJumpWP = getCurrentMissionInd();
												doJumpCount = 0;
											}
											++doJumpCount;
											if( doJumpCount > current_mission_inf.params[1] )
											{	//循环次数已达到
												continue;
											}
											else
											{
												if( current_mission_inf.params[0] > 0 )
												{
													if( setCurrentMission( current_mission_inf.params[0] - 1 ) )
														goto doJumpReadMission;
													else
													{	//无航点信息
														navInf.usr_temp[ctMissionState] = 1;
														res = -3;
														break;
													}
												}
												else
												{	//跳转航点错误
													navInf.usr_temp[ctMissionState] = 1;
													res = -3;
													break;
												}
											}
											res = -3;
										}
										else if( Process_InflightCmd( current_mission_inf.cmd, current_mission_inf.params ) == false )
										{
											if( check_NavCmd(current_mission_inf.cmd,
													freq, 
													current_mission_inf.frame,
													current_mission_inf.params
												) )
											{	//指令可执行
												if( navInf.usr_temp[ctMissionState]==110 )
													navInf.counter1 = 50;	//继续预存任务
												else
													navInf.counter1 = 0;	//继续任务(无预存)
												navInf.counter2 = 0;
												navInf.usr_temp[ctMissionState] = 1;
												
												//保存任务信息
												vector3<double> BA;
												NavCmd16_WAYPOINT_GetInfo( current_mission_inf.frame, current_mission_inf.params, 0, &BA, 0 );
												restoreWpInf.CurrentWp[0] = current_mission_ind;
												restoreWpInf.line_x = -BA.x;
												restoreWpInf.line_y = -BA.y;
												restoreWpInf.line_z = -BA.z;
												restoreWpInf.line_fs = 0;
												restoreWpInf.CamTrigDist = CamTriggDist;
												
												res = -3;
												break;
											}
										}
									}
									else
									{	//无航点信息
										navInf.usr_temp[ctMissionState] = 1;
										res = -3;
										break;
									}
								}
								else
								{	//无航点信息
									navInf.usr_temp[ctMissionState] = 1;
									res = -3;
									break;
								}
							}
						}
						else
							//异常指令
							navInf.usr_temp[ctMissionState] = 0;
					}
				}
				
				if( NavCmdRs_SuccessOrFault(res) )
				{	//错误或执行完成
					
					//不自动执行返回手动模式
					if( current_mission_inf.autocontinue == 0 )
					{
						swManualMode
					}
					
					if( res < 0 )
					{	//切换到下一模式
						MissionInf chk_inf;
						uint16_t chk_ind;
						if( ReadCurrentMission(&chk_inf, &chk_ind) )
						{	//读取当前任务信息比较						
							if( chk_ind==current_mission_ind && memcmp( &chk_inf, &current_mission_inf, sizeof(MissionInf) ) == 0 )
							{	//如果相同才切换下一个任务
								if( setCurrentMission( getCurrentMissionInd() + navInf.usr_temp[MissionInc] + 1 ) == false )
								{	//无航点信息返回手动模式
									setCurrentMission( 0 );
									swManualMode
									//复位当前航点信息
									reset_RestoreWpInf(restoreWpInf);
									if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
									{
										bool inFlight;
										get_is_inFlight(&inFlight);
										if( inFlight==false )
										{	//降落完成加锁
											Attitude_Control_Disable();
											return MR_OK;
										}
									}
								}
								else
								{
									if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
									{	//载入下一航点成功
										//初始化任务信息
										init_NavCmdInf(&navInf);
										//更新任务状态
										if( current_mission_inf.cmd == MAV_CMD_NAV_WAYPOINT )
										{	//waypoint任务更新航线信息
											vector3<double> BA;
											NavCmd16_WAYPOINT_GetInfo( current_mission_inf.frame, current_mission_inf.params, 0, &BA, 0 );
											restoreWpInf.CurrentWp[0] = current_mission_ind;
											restoreWpInf.line_x = -BA.x;
											restoreWpInf.line_y = -BA.y;
											restoreWpInf.line_z = -BA.z;
											restoreWpInf.line_fs = 0;
											restoreWpInf.CamTrigDist = CamTriggDist;
										}
										else
										{	//其他类型任务
											restoreWpInf.CurrentWp[0] = current_mission_ind;
											restoreWpInf.line_x = 0;
											restoreWpInf.line_y = 0;
											restoreWpInf.line_z = 0;
											restoreWpInf.line_fs = 0;
											restoreWpInf.CamTrigDist = CamTriggDist;
										}
									}
									else
									{	//无航点信息返回手动模式
										setCurrentMission( 0 );
										swManualMode						
										//复位当前航点信息
										reset_RestoreWpInf(restoreWpInf);
										if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
										{
											bool inFlight;
											get_is_inFlight(&inFlight);
											if( inFlight==false )
											{	//降落完成加锁
												Attitude_Control_Disable();
												return MR_OK;
											}
										}
									}
								}
							}
							else
							{	//航点信息不相同不切换下一任务
								//使用新获取的任务信息
								current_mission_inf = chk_inf;
								current_mission_ind = chk_ind;
								//初始化任务信息
								init_NavCmdInf(&navInf);
								//更新任务状态																	
								if( current_mission_inf.cmd == MAV_CMD_NAV_WAYPOINT )
								{	//waypoint任务更新航线信息
									vector3<double> BA;
									NavCmd16_WAYPOINT_GetInfo( current_mission_inf.frame, current_mission_inf.params, 0, &BA, 0 );
									restoreWpInf.CurrentWp[0] = current_mission_ind;
									restoreWpInf.line_x = -BA.x;
									restoreWpInf.line_y = -BA.y;
									restoreWpInf.line_z = -BA.z;
									restoreWpInf.line_fs = 0;
									restoreWpInf.CamTrigDist = CamTriggDist;
								}
								else
								{	//其他类型任务
									restoreWpInf.CurrentWp[0] = current_mission_ind;
									restoreWpInf.line_x = 0;
									restoreWpInf.line_y = 0;
									restoreWpInf.line_z = 0;
									restoreWpInf.line_fs = 0;
									restoreWpInf.CamTrigDist = CamTriggDist;
								}
							}
						}
						else
						{	//无航点信息返回手动模式
							setCurrentMission( 0 );
							swManualMode
							//复位当前航点信息
							reset_RestoreWpInf(restoreWpInf);
							if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
							{	//降落完成加锁
								bool inFlight;
								get_is_inFlight(&inFlight);
								if( inFlight==false )
								{
									Attitude_Control_Disable();
									return MR_OK;
								}
							}
						}
					}
					else
					{	//切换到指定模式
						if( setCurrentMission( res ) == false )
						{	//切换失败返回手动模式
							setCurrentMission( 0 );
							swManualMode
							//复位当前航点信息
							reset_RestoreWpInf(restoreWpInf);
						}
						if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
						{	//载入下一航点成功
							//初始化任务信息
							init_NavCmdInf(&navInf);
							//更新任务状态														
							if( current_mission_inf.cmd == MAV_CMD_NAV_WAYPOINT )
							{	//waypoint任务更新航线信息
								vector3<double> BA;
								NavCmd16_WAYPOINT_GetInfo( current_mission_inf.frame, current_mission_inf.params, 0, &BA, 0 );
								restoreWpInf.CurrentWp[0] = current_mission_ind;
								restoreWpInf.line_x = -BA.x;
								restoreWpInf.line_y = -BA.y;
								restoreWpInf.line_z = -BA.z;
								restoreWpInf.line_fs = 0;
								restoreWpInf.CamTrigDist = CamTriggDist;
							}
							else
							{	//其他类型任务
								restoreWpInf.CurrentWp[0] = current_mission_ind;
								restoreWpInf.line_x = 0;
								restoreWpInf.line_y = 0;
								restoreWpInf.line_z = 0;
								restoreWpInf.line_fs = 0;
								restoreWpInf.CamTrigDist = CamTriggDist;
							}
						}
						else
						{	//无航点信息返回手动模式
							setCurrentMission( 0 );
							swManualMode							
							//复位当前航点信息
							reset_RestoreWpInf(restoreWpInf);
							if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
							{
								bool inFlight;
								get_is_inFlight(&inFlight);
								if( inFlight==false )
								{	//降落完成加锁
									Attitude_Control_Disable();
									return MR_OK;
								}
							}
						}					
					}
				}
				else
				{	//任务执行中
					if( NavCmdRs_InProgress_CanExInFlightCmd(res) )
					{	//可执行InFlightCmd
						
						if(msg_available) {
							Process_InflightCmd( msg.cmd, msg.params );
						}
						
//						if( navInf.usr_temp[DealInFlightCmd] == 0 )
//						{	//还未执行inFlightCmd
//							//执行所有inFlightCmd
//							MissionInf inFlightMs_inf;
//							while(1)
//							{
//								if( ReadMission( getCurrentMissionInd() + navInf.usr_temp[MissionInc] + 1, &inFlightMs_inf ) )
//								{
//									if( Process_InflightCmd( inFlightMs_inf.cmd, inFlightMs_inf.params ) )
//										navInf.usr_temp[MissionInc] += 1;
//									else
//										break;
//								}
//								else
//									break;
//							}
//						}
						navInf.usr_temp[DealInFlightCmd] = 1;
						
//						//储存飞行状态
//						vector3<double> line_AB;
//						double flightDistance = -1;
//						if( Position_Control_get_LineFlightABDistance( &line_AB, &flightDistance ) )
//						{
//							restoreWpInf.CurrentWp[0] = current_mission_ind;
//							restoreWpInf.line_x = line_AB.x;
//							restoreWpInf.line_y = line_AB.y;
//							restoreWpInf.line_z = line_AB.z;
//							restoreWpInf.line_fs = flightDistance;
//							restoreWpInf.CamTrigDist = camTriggDist;
//						}
//						
//						//定距拍照
//						if( camTriggDist > 0 )
//						{
//							Position_Control_get_LineFlightDistance(&flightDistance);
//							if( flightDistance >= 0 )
//							{
//								int mult = (int)(flightDistance / camTriggDist) + 1;
//								if( mult > navInf.usr_temp[CamTriggDistMult] )
//								{
//									InflightCmd_CamTakePhoto();
//									navInf.usr_temp[CamTriggDistMult] = mult;
//								}
//							}
//						}
					}
				}
			}
		}
		else if( cMode==AFunc_Offboard )
		{
			Position_Control_Enable();
			bool pos_ena;
			is_Position_Control_Enabled(&pos_ena);
			if( pos_ena == false )
			{	//位置控制器无法打开返回手动模式
				swManualMode
				goto Manual_Mode;
			}
			
			if(mode_switched)
			{	//刚进入模式初始化变量
				mode_switched = false;
				ManualModeNavCmdInprogress = false;
				Attitude_Control_set_YawLock();
			}
			
			//上报模式状态
			setCurrentFlyMode(AFunc_Offboard);
			
			//执行指令
			if(msg_available)
			{
				//执行inflight cmd
				if( Process_InflightCmd( msg.cmd, msg.params ) == false )
				if( check_NavCmd( msg.cmd, freq, default_NavCmd_frame, msg.params ) )
				{	//指令可被执行
					init_NavCmdInf(&navInf);
					ManualModeNavCmdInprogress = true;
					ManualModeNavCmd = msg;
					msg_handled = 1;
				}
				
				//执行set_position_target指令
				if( msg.cmd == 1084 )
				{	//SET_POSITION_TARGET_LOCAL_NED
					uint16_t type_mask = msg.params[0];
					//thr
					if( (type_mask&POSITION_TARGET_TYPEMASK_Z_IGNORE)==0 )
					{	//pva
						double posz=msg.params[3]*100;
						double velz=0;
						double accz=0;
						if( (type_mask&POSITION_TARGET_TYPEMASK_VZ_IGNORE)==0 )
						{
							velz = msg.params[6]*100;
							if( (type_mask&POSITION_TARGET_TYPEMASK_AZ_IGNORE)==0 )
								accz = msg.params[9]*100;
						}
						switch( msg.frame )
						{
							case MAV_FRAME_LOCAL_NED:
							{
								Position_Control_set_TargetPosVelAccZ_OffBoard( -posz, -velz, -accz );
								break;
							}
							default:
							case MAV_FRAME_LOCAL_ENU:
							{
								Position_Control_set_TargetPosVelAccZ_OffBoard( posz, velz, accz );
								break;
							}
							case MAV_FRAME_BODY_FLU:
							{
								Position_Control_set_TargetPosRelativeVelAccZ_OffBoard( posz, velz, accz );
								break;
							}
							case MAV_FRAME_BODY_FRD:
							case MAV_FRAME_BODY_NED:
							case MAV_FRAME_BODY_OFFSET_NED:
							{
								Position_Control_set_TargetPosRelativeVelAccZ_OffBoard( -posz, -velz, -accz );
								break;
							}
						}
						Offboard_ThrTIME = TIME::now();
					}
					else if( (type_mask&POSITION_TARGET_TYPEMASK_VZ_IGNORE)==0 )
					{	//va
						double velz=0;
						double accz=0;
						velz = msg.params[6]*100;
						if( (type_mask&POSITION_TARGET_TYPEMASK_AZ_IGNORE)==0 )
							accz = msg.params[9]*100;
						switch( msg.frame )
						{
							case MAV_FRAME_BODY_FRD:
							case MAV_FRAME_BODY_NED:
							case MAV_FRAME_BODY_OFFSET_NED:
							case MAV_FRAME_LOCAL_NED:
							{
								Position_Control_set_TargetVelAccZ_OffBoard( -velz, -accz );
								break;
							}
							default:
							case MAV_FRAME_BODY_FLU:
							case MAV_FRAME_LOCAL_ENU:
							{
								Position_Control_set_TargetVelAccZ_OffBoard( velz, accz );
								break;
							}
						}
						Offboard_ThrTIME = TIME::now();
					}
					//yaw
					if( (type_mask&POSITION_TARGET_TYPEMASK_YAW_IGNORE)==0 )
					{	//pva
						double Tyaw = 0.5*Pi - msg.params[10];
						double Trate = 0;
						if( (type_mask&POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)==0 )
							Trate = -msg.params[11];
						Attitude_Control_set_Target_Yaw_Offboard( Tyaw, Trate );
						Offboard_YawTIME = TIME::now();
					}
					else if( (type_mask&POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)==0 )
					{
						double Trate = -msg.params[11];
						Attitude_Control_set_Target_YawRate( Trate );
						Offboard_YawTIME = TIME::now();
					}
					//pit rol
					if( (type_mask&POSITION_TARGET_TYPEMASK_X_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_Y_IGNORE)==0 )
					{	//pva
						double posx=msg.params[1]*100;	double posy=msg.params[2]*100;
						double velx=0, vely=0;
						double accx=0, accy=0;
						if( (type_mask&POSITION_TARGET_TYPEMASK_VX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_VY_IGNORE)==0 )
						{
							velx = msg.params[4]*100;	vely = msg.params[5]*100;
							if( (type_mask&POSITION_TARGET_TYPEMASK_AX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_AY_IGNORE)==0 )
								accx = msg.params[7]*100;	accy = msg.params[8]*100;
						}
						switch( msg.frame )
						{
							case MAV_FRAME_LOCAL_NED:
							{
								Position_Control_set_TargetPosVelAccXY_OffBoard( posy, posx, vely, velx, accy, accx );
								break;
							}
							default:
							case MAV_FRAME_LOCAL_ENU:
							{
								Position_Control_set_TargetPosVelAccXY_OffBoard( posx, posy, velx, vely, accx, accy );
								break;
							}
							case MAV_FRAME_BODY_FLU:
							{
								double yaw;	double yaw_declination;
								get_YawDeclination(&yaw_declination);
								Attitude_Control_get_TargetTrackYaw(&yaw);
								yaw += yaw_declination;
								double sin_Yaw, cos_Yaw;
								fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
								double posx_enu = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
								double posy_enu = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
								double velx_enu = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
								double vely_enu = BodyHeading2ENU_y( velx , vely , sin_Yaw , cos_Yaw );
								double accx_enu = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
								double accy_enu = BodyHeading2ENU_y( accx , accy , sin_Yaw , cos_Yaw );
								Position_Control_set_TargetPosRelativeVelAccXY_OffBoard( posx_enu, posy_enu, velx_enu, vely_enu, accx_enu, accy_enu );
								break;
							}
							case MAV_FRAME_BODY_FRD:
							case MAV_FRAME_BODY_NED:
							case MAV_FRAME_BODY_OFFSET_NED:
							{
								double yaw;	double yaw_declination;
								get_YawDeclination(&yaw_declination);
								Attitude_Control_get_TargetTrackYaw(&yaw);
								yaw += yaw_declination;
								double sin_Yaw, cos_Yaw;
								fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
								double posx_enu = BodyHeading2ENU_x( posx , -posy , sin_Yaw , cos_Yaw );
								double posy_enu = BodyHeading2ENU_y( posx , -posy , sin_Yaw , cos_Yaw );
								double velx_enu = BodyHeading2ENU_x( velx , -vely , sin_Yaw , cos_Yaw );
								double vely_enu = BodyHeading2ENU_y( velx , -vely , sin_Yaw , cos_Yaw );
								double accx_enu = BodyHeading2ENU_x( accx , -accy , sin_Yaw , cos_Yaw );
								double accy_enu = BodyHeading2ENU_y( accx , -accy , sin_Yaw , cos_Yaw );
								Position_Control_set_TargetPosRelativeVelAccXY_OffBoard( posx_enu, posy_enu, velx_enu, vely_enu, accx_enu, accy_enu );
								break;
							}
						}
						Offboard_PitRolTIME = TIME::now();
					}
					else if( (type_mask&POSITION_TARGET_TYPEMASK_VX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_VY_IGNORE)==0 )
					{	//va
						double velx=0, vely=0;
						double accx=0, accy=0;
						velx = msg.params[4]*100;	vely = msg.params[5]*100;
						if( (type_mask&POSITION_TARGET_TYPEMASK_AX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_AY_IGNORE)==0 )
							accx = msg.params[7]*100;	accy = msg.params[8]*100;
						switch( msg.frame )
						{
							case MAV_FRAME_LOCAL_NED:
							{
								Position_Control_set_TargetVelAccXY_OffBoard( vely, velx, accy, accx );
								break;
							}
							default:
							case MAV_FRAME_LOCAL_ENU:
							{
								Position_Control_set_TargetVelAccXY_OffBoard( velx, vely, accx, accy );
								break;
							}
							case MAV_FRAME_BODY_FLU:
							{
								double yaw;	double yaw_declination;
								get_YawDeclination(&yaw_declination);
								Attitude_Control_get_TargetTrackYaw(&yaw);
								yaw += yaw_declination;
								double sin_Yaw, cos_Yaw;
								fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
								double velx_enu = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
								double vely_enu = BodyHeading2ENU_y( velx , vely , sin_Yaw , cos_Yaw );
								double accx_enu = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
								double accy_enu = BodyHeading2ENU_y( accx , accy , sin_Yaw , cos_Yaw );
								Position_Control_set_TargetVelAccXY_OffBoard( velx_enu, vely_enu, accx_enu, accy_enu );
								break;
							}
							case MAV_FRAME_BODY_FRD:
							case MAV_FRAME_BODY_NED:
							case MAV_FRAME_BODY_OFFSET_NED:
							{
								double yaw;	double yaw_declination;
								get_YawDeclination(&yaw_declination);
								Attitude_Control_get_TargetTrackYaw(&yaw);
								yaw += yaw_declination;
								double sin_Yaw, cos_Yaw;
								fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
								double velx_enu = BodyHeading2ENU_x( velx , -vely , sin_Yaw , cos_Yaw );
								double vely_enu = BodyHeading2ENU_y( velx , -vely , sin_Yaw , cos_Yaw );
								double accx_enu = BodyHeading2ENU_x( accx , -accy , sin_Yaw , cos_Yaw );
								double accy_enu = BodyHeading2ENU_y( accx , -accy , sin_Yaw , cos_Yaw );
								Position_Control_set_TargetVelAccXY_OffBoard( velx_enu, vely_enu, accx_enu, accy_enu );
								break;
							}
						}
						Offboard_PitRolTIME = TIME::now();
					}
					
					//发送ACK
					uint8_t port_id = msg.cmd_type&0xf;
					const Port* port = get_CommuPort(port_id);
					if( port->write )
					{
						if( mavlink_lock_chan(port_id,0.01) )
						{
							mavlink_message_t msg_sd;
							
							mavlink_msg_position_target_local_ned_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								port_id , 	//chan
								&msg_sd ,
								TIME::get_System_Run_Time()*1e3,
								msg.frame,
								msg.params[0],
								msg.params[1],
								msg.params[2],
								msg.params[3],
								msg.params[4],
								msg.params[5],
								msg.params[6],
								msg.params[7],
								msg.params[8],
								msg.params[9],
								msg.params[10],
								msg.params[11]
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, -1);
							mavlink_unlock_chan(port_id);
						}
					}
				}
				else if( msg.cmd == 1086 )
				{	//SET_POSITION_TARGET_GLOBAL_INT
					uint16_t type_mask = msg.params[0];
					//thr
					if( (type_mask&POSITION_TARGET_TYPEMASK_Z_IGNORE)==0 )
					{	//pva
						double posz=msg.params[3]*100;
						double velz=0;
						double accz=0;
						if( (type_mask&POSITION_TARGET_TYPEMASK_VZ_IGNORE)==0 )
						{
							velz = msg.params[6]*100;
							if( (type_mask&POSITION_TARGET_TYPEMASK_AZ_IGNORE)==0 )
								accz = msg.params[9]*100;
						}
						switch( msg.frame )
						{
							case MAV_FRAME_GLOBAL_INT:
							case MAV_FRAME_GLOBAL:
							{
								Position_Control_set_TargetPosGlobalVelAccZ_OffBoard( -posz, -velz, -accz );
								break;
							}
							default:
							case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
							case MAV_FRAME_GLOBAL_RELATIVE_ALT:
							case MAV_FRAME_GLOBAL_TERRAIN_ALT:
							case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
							{
								Position_Control_set_TargetPosZRAVelAccZ_OffBoard( -posz, -velz, -accz );
								break;
							}
						}
						Offboard_ThrTIME = TIME::now();
					}
					else if( (type_mask&POSITION_TARGET_TYPEMASK_VZ_IGNORE)==0 )
					{	//va
						double velz=0;
						double accz=0;
						velz = msg.params[6]*100;
						if( (type_mask&POSITION_TARGET_TYPEMASK_AZ_IGNORE)==0 )
							accz = msg.params[9]*100;
						Position_Control_set_TargetVelAccZ_OffBoard( -velz, -accz );
						Offboard_ThrTIME = TIME::now();
					}
					//yaw
					if( (type_mask&POSITION_TARGET_TYPEMASK_YAW_IGNORE)==0 )
					{	//pva
						double Tyaw = 0.5*Pi - msg.params[10];
						double Trate = 0;
						if( (type_mask&POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)==0 )
							Trate = -msg.params[11];
						Attitude_Control_set_Target_Yaw_Offboard( Tyaw, Trate );
						Offboard_YawTIME = TIME::now();
					}
					else if( (type_mask&POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)==0 )
					{
						double Trate = -msg.params[11];
						Attitude_Control_set_Target_YawRate( Trate );
						Offboard_YawTIME = TIME::now();
					}
					//pit rol
					if( (type_mask&POSITION_TARGET_TYPEMASK_X_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_Y_IGNORE)==0 )
					{	//pva
						double posx=msg.params[1]*100;	double posy=msg.params[2]*100;
						double velx=0, vely=0;
						double accx=0, accy=0;
						if( (type_mask&POSITION_TARGET_TYPEMASK_VX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_VY_IGNORE)==0 )
						{
							velx = msg.params[4]*100;	vely = msg.params[5]*100;
							if( (type_mask&POSITION_TARGET_TYPEMASK_AX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_AY_IGNORE)==0 )
								accx = msg.params[7]*100;	accy = msg.params[8]*100;
						}
						Position_Control_set_TargetPosVelAccXY_OffBoard( posy, posx, vely, velx, accy, accx );
						Offboard_PitRolTIME = TIME::now();
					}
					else if( (type_mask&POSITION_TARGET_TYPEMASK_VX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_VY_IGNORE)==0 )
					{	//va
						double velx=0, vely=0;
						double accx=0, accy=0;
						velx = msg.params[4]*100;	vely = msg.params[5]*100;
						if( (type_mask&POSITION_TARGET_TYPEMASK_AX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_AY_IGNORE)==0 )
							accx = msg.params[7]*100;	accy = msg.params[8]*100;
						Position_Control_set_TargetVelAccXY_OffBoard( vely, velx, accy, accx );
						Offboard_PitRolTIME = TIME::now();
					}
					
					//发送ACK
					uint8_t port_id = msg.cmd_type&0xf;
					const Port* port = get_CommuPort(port_id);
					if( port->write )
					{
						if( mavlink_lock_chan(port_id,0.01) )
						{
							mavlink_message_t msg_sd;
							
							mavlink_msg_position_target_local_ned_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								port_id , 	//chan
								&msg_sd ,
								TIME::get_System_Run_Time()*1e3,
								msg.frame,
								msg.params[0],
								msg.params[1],
								msg.params[2],
								msg.params[3],
								msg.params[4],
								msg.params[5],
								msg.params[6],
								msg.params[7],
								msg.params[8],
								msg.params[9],
								msg.params[10],
								msg.params[11]
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, -1);
							mavlink_unlock_chan(port_id);
						}
					}
				}
			}
			
			if( rc.available )
			{				
				/*判断退出模式*/
					//获取飞行状态
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( rc.data[0] > 30 )
					{
						exit_mode_counter_rs = 480;
						if( exit_mode_counter < exit_mode_counter_rs )
							exit_mode_counter = exit_mode_counter_rs;
					}
					//降落后自动加锁
					if( inFlight==false && rc.data[0]<30 )
					{
						if( ++exit_mode_counter >= 500 )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					}
					else
						exit_mode_counter = exit_mode_counter_rs;
				/*判断退出模式*/

				//判断摇杆是否回中
				bool thr_in_neutral = in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] );
				bool yaw_in_neutral = in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] );
				bool pitrol_in_neutral = 
					in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] );
				
				//油门控制 摇杆在中位且一段时间无控制才锁定
				if( thr_in_neutral )
				{
					if( Offboard_ThrTIME.get_pass_time() > 1.5 )
						Position_Control_set_ZLock();
				}
				else
				{
					double thr_stick = remove_deadband( rc.data[0] - 50.0 , (double)MFunc_cfg.NeutralZone[0] );
					if( thr_stick > 0 )
						thr_stick *= getPosCtrlCfg()->maxVelUp[0] / 50;
					else
						thr_stick *= getPosCtrlCfg()->maxVelDown[0] / 50;
					Position_Control_set_TargetVelocityZ(thr_stick);
					Offboard_ThrTIME.set_invalid();
				}
				
				//偏航控制 摇杆在中位且一段时间无控制才锁定
				if( yaw_in_neutral )
				{
					if( Offboard_YawTIME.get_pass_time() > 1.5 )
						Attitude_Control_set_YawLock();
				}
				else
				{
					double YCtrlScale = degree2rad( getAttCtrlCfg()->maxYSp[0] / 50.0 );
					Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );
					Offboard_YawTIME.set_invalid();
				}
				
				//横滚俯仰控制 摇杆在中位且一段时间无控制才锁定
				if( pitrol_in_neutral )
				{
					if( Offboard_PitRolTIME.get_pass_time() > 1.5 )
						Position_Control_set_XYLock();
				}
				else
				{
					double RPCtrlScale = atan2( getPosCtrlCfg()->maxAccXY[0] / 50.0, GravityAcc );
					double XYCtrlScale = getPosCtrlCfg()->maxVelXY[0] / 50.0;						
					double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
					double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
					vector3<double> velocityFLU;
					get_VelocityFLU_Ctrl(&velocityFLU);
					double vel_stick_err_pitch = velocityFLU.x/XYCtrlScale - pitch_sitck_d;
					double vel_stick_err_roll = velocityFLU.y/XYCtrlScale - -roll_sitck_d;
					constrain_vector( vel_stick_err_roll, vel_stick_err_pitch, 50 );
					Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
						pitch_sitck_d * XYCtrlScale ,\
						-roll_sitck_d * XYCtrlScale , \
						fabs( vel_stick_err_roll  )*RPCtrlScale, \
						fabs( vel_stick_err_pitch )*RPCtrlScale \
					);
					Offboard_PitRolTIME.set_invalid();
				}
			}
			else if( inFlight == false )
			{	//未起飞保持此模式
				Position_Control_set_XYLock();
				Attitude_Control_set_YawLock();
				Position_Control_set_ZLock();
			}
			else if( Offboard_ThrTIME.get_pass_time()>2 && Offboard_PitRolTIME.get_pass_time()>2 )
			{	//无遥控信号且一段时间无信号
				//进入安全模式
				change_Mode(AFunc_RTL)
				goto RTL;
			}
		}
		else
		{	//手动飞行模式（包含定高定点控制）
			Manual_Mode:
						
			//复位跟随当前点
			if( mode_switched || !inFlight )
			{
				vector3<double> pos;
				get_Position_Ctrl(&pos);
				followPos = pos;
			}
			
			if(mode_switched)
			{	//刚进入手动模式初始化变量
				mode_switched = false;
				ManualModeNavCmdInprogress = false;
				Attitude_Control_set_YawLock();
				AvLocked = false;
				AvMinDistance = -1;
				
				//复位跟随原点
				followOrigin.zero();
				//复位跟随
				followDataUpdateTime.set_invalid();
				followTPos.zero();
				followTVel.zero();
			}
			
			if( rc.available )
			{				
				/*判断退出模式*/
					//获取飞行状态
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( rc.data[0] > 30 )
					{
						exit_mode_counter_rs = 480;
						if( exit_mode_counter < exit_mode_counter_rs )
							exit_mode_counter = exit_mode_counter_rs;
					}
					//降落后自动加锁
					if( inFlight==false && rc.data[0]<30 )
					{
						if( ++exit_mode_counter >= 500 )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					}
					else
						exit_mode_counter = exit_mode_counter_rs;
				/*判断退出模式*/
					
				//切换定高定点
				if( cMode==AFunc_AltHold )
					Position_Control_Disable();
				else
					Position_Control_Enable();				
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
				
				if(pos_ena)
					setLedMode(LEDMode_Flying2);
				else
					setLedMode(LEDMode_Flying1);
				
				//上报模式状态
				if( pos_ena )
					setCurrentFlyMode(cMode);
				else
					setCurrentFlyMode(AFunc_AltHold);
				
				//判断摇杆是否回中
				bool sticks_in_neutral = 
					in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] );
				
				if( pos_ena && cMode==AFunc_Loiter )
				{
					//接收跟随命令
					if(msg_available)
					{
						if(msg.cmd==144)
						{	//跟随指令
							double TLat = msg.params[1];
							double TLon = msg.params[2];
							//获取最优全球定位传感器信息
							PosSensorHealthInf2 global_inf;
							if( get_OptimalGlobal_XY( &global_inf ) )
							{
								//获取指定经纬度平面坐标
								double x, y;
								map_projection_project( &global_inf.mp, TLat, TLon, &x, &y );
								x -= global_inf.HOffset.x;
								y -= global_inf.HOffset.y;
								
								if( followDataUpdateTime.is_valid()==false || followDataUpdateTime.get_pass_time()>=2 )
								{
									followOrigin.x = x;
									followOrigin.y = y;
									followOrigin.z = msg.params[3];
								}
								
								followTPos.x = x;
								followTPos.y = y;
								followTPos.z = msg.params[3];
								followTPos -= followOrigin;
								followTVel.x = msg.params[4];
								followTVel.y = msg.params[5];
								followTVel.z = msg.params[6];
								followDataUpdateTime = TIME::now();
							}
						}
					}
					
					bool followAvailable = ( followDataUpdateTime.is_valid() && followDataUpdateTime.get_pass_time()<2 );
					
					//油门杆控制垂直速度
					if( in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) )
						Position_Control_set_ZLock();
					else
					{
						double thr_stick = remove_deadband( rc.data[0] - 50.0 , (double)MFunc_cfg.NeutralZone[0] );
						if( thr_stick > 0 )
							thr_stick *= getPosCtrlCfg()->maxVelUp[0] / 50;
						else
							thr_stick *= getPosCtrlCfg()->maxVelDown[0] / 50;
						Position_Control_set_TargetVelocityZ(thr_stick);
					}
					
					//偏航杆在中间锁偏航
					//不在中间控制偏航速度
					double YCtrlScale = degree2rad( getAttCtrlCfg()->maxYSp[0] / 50.0 );
					if( in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) )
						Attitude_Control_set_YawLock();
					else
						Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );						
					
					//XY期望速度
					double RPCtrlScale = atan2( getPosCtrlCfg()->maxAccXY[0] / 50.0, GravityAcc );
					double XYCtrlScale = getPosCtrlCfg()->maxVelXY[0] / 50.0;						
					double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
					double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
					vector3<double> velocityFLU;
					get_VelocityFLU_Ctrl(&velocityFLU);
					double vel_stick_err_pitch = velocityFLU.x/XYCtrlScale - pitch_sitck_d;
					double vel_stick_err_roll = velocityFLU.y/XYCtrlScale - -roll_sitck_d;
					constrain_vector( vel_stick_err_roll, vel_stick_err_pitch, 50 );
					vector2<double> TvelFlu;
					TvelFlu.x = pitch_sitck_d * XYCtrlScale;
					TvelFlu.y = -roll_sitck_d * XYCtrlScale;
					
					//转弯补偿
					#define CIR_F 
					double w;
					Attitude_Control_get_YawTrackVel(&w);
					vector2<double> TvelCir;
					if( sq(TvelFlu.x)+sq(TvelFlu.y) > sq(velocityFLU.x)+sq(velocityFLU.y) )
					{
						TvelCir.x = -velocityFLU.y*w;
						TvelCir.y =  velocityFLU.x*w;
					}
					else
					{
						TvelCir.x = -TvelFlu.y*w;
						TvelCir.y =  TvelFlu.x*w;
					}
					TvelFlu += TvelCir*(1.0/getPosCtrlCfg()->P1[0]);
					
					//获取ENU系期望速度
					double yaw;	double yaw_declination;
					get_YawDeclination(&yaw_declination);
					Attitude_Control_get_TargetTrackYaw(&yaw);
					yaw += yaw_declination;
					double sin_Yaw, cos_Yaw;
					fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
					vector2<double> TvelEnu;
					TvelEnu.x = BodyHeading2ENU_x( TvelFlu.x , TvelFlu.y , sin_Yaw , cos_Yaw );
					TvelEnu.y = BodyHeading2ENU_y( TvelFlu.x , TvelFlu.y , sin_Yaw , cos_Yaw );

					//限制期望速度
					vector3<double> velocityEnu;
					get_VelocityENU(&velocityEnu);
					vector2<double> velEnu( velocityEnu.x, velocityEnu.y );
					vector2<double> velErr = TvelEnu - velEnu;
					if( followAvailable )
					{	//跟随目标可用
						velErr.x += followTVel.x;
						velErr.y += followTVel.y;
					}
					double velErrDis = velErr.get_square();
					double maxVelErrDis = getPosCtrlCfg()->maxAccXY[0] / getPosCtrlCfg()->P2[0];
					vector2<double> TvelEnuC;
					if( velErrDis > sq(maxVelErrDis) ) {
						velErrDis = safe_sqrt(velErrDis);
						velErr *= maxVelErrDis/velErrDis;
						TvelEnuC = velErr + velEnu;
					} else {
						TvelEnuC = TvelEnu;
					}
					
					//计算期望速度
					vector3<double> TVel;
					TVel.x = TvelEnu.x*getPosCtrlCfg()->VelXYFF[0];
					TVel.y = TvelEnu.y*getPosCtrlCfg()->VelXYFF[0];
					
					//计算期望位置
					followPos.x += TvelEnuC.x*h;
					followPos.y += TvelEnuC.y*h;
					vector3<double> TPos;
					TPos.x = followPos.x;
					TPos.y = followPos.y;
					if( followAvailable)
					{	//跟随目标可用
						TPos.x += followTPos.x;
						TPos.y += followTPos.y;
						TVel.x += followTVel.x;
						TVel.y += followTVel.y;
					}
					
					Position_Control_set_TargetPosVelAccXY_OffBoard( 
						TPos.x, TPos.y, 
						TVel.x, TVel.y, 
						0 , 0
					);
					
				}
				else if( sticks_in_neutral && pos_ena && (cMode==AFunc_PosHold || cMode==AFunc_PosHoldNH || cMode==AFunc_PosHoldAv || cMode==AFunc_PosHoldNHAv) )
				{	//摇杆在中间且在定点模式下允许执行命令
					if(msg_available)
					{
						if( Process_InflightCmd( msg.cmd, msg.params ) == false )
						if( check_NavCmd( msg.cmd, freq, default_NavCmd_frame, msg.params ) )
						{	//指令可被执行
							init_NavCmdInf(&navInf);
							ManualModeNavCmdInprogress = true;
							ManualModeNavCmd = msg;
							msg_handled = 1;
						}
					}
					
					if( ManualModeNavCmdInprogress )
					{	//需要执行NavCmd
						int16_t res = -100;
						res = Process_NavCmd( ManualModeNavCmd.cmd, freq, default_NavCmd_frame, ManualModeNavCmd.params, &navInf );
						if( NavCmdRs_SuccessOrFault(res) )
						{	//NavCmd执行完成
							ManualModeNavCmdInprogress = false;
						}
					}
					else
					{
						Position_Control_set_ZLock();
						Position_Control_set_XYLock();
						Attitude_Control_set_YawLock();
					}
					
					//复位避障锁定标志位
					AvLocked = false;
					AvMinDistance = -1;
				}
				else
				{	//摇杆不在中间手动飞行
								
					ManualModeNavCmdInprogress = false;
					
					//油门杆控制垂直速度
					if( in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) )
						Position_Control_set_ZLock();
					else
					{
						double thr_stick = remove_deadband( rc.data[0] - 50.0 , (double)MFunc_cfg.NeutralZone[0] );
						if( thr_stick > 0 )
							thr_stick *= getPosCtrlCfg()->maxVelUp[0] / 50;
						else
							thr_stick *= getPosCtrlCfg()->maxVelDown[0] / 50;
						Position_Control_set_TargetVelocityZ(thr_stick);
					}
					
					if( pos_ena )
					{
						
						if( cMode==AFunc_ManualCircle )
						{
							double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
							double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
							double yaw_sitck_d = remove_deadband( rc.data[1] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
							double h = 1.0/freq;
							Position_Control_do_ManualCircleRelative( 5*roll_sitck_d*h, -3*pitch_sitck_d*h, 3*yaw_sitck_d*h );
						}
						else
						{
							//计算避障最大速度
							double maxVelXY = getPosCtrlCfg()->maxVelXY[0];
							
							//获取避障参数
							const AvoidanceCfg* avCfg = getAvCfg();
							
							//计算避障
							double roll_sitck = rc.data[3] - 50.0;
							double pitch_sitck = rc.data[2] - 50.0;
							double roll_sitck_d = remove_deadband( roll_sitck, (double)MFunc_cfg.NeutralZone[0] );
							double pitch_sitck_d = remove_deadband( pitch_sitck, (double)MFunc_cfg.NeutralZone[0] );
							vector2<double> targetVel, targetVelNT;
							if( cMode==AFunc_PosHoldNH || cMode==AFunc_PosHoldNHAv )
							{
								if( HeadlessYaw < -100 )
								{
									double yaw;	double yaw_declination;
									get_YawDeclination(&yaw_declination);
									Attitude_Control_get_TargetTrackYaw(&yaw);
									yaw += yaw_declination;
									HeadlessYaw = yaw;
								}
								double sin_Yaw, cos_Yaw;
								fast_sin_cos( HeadlessYaw, &sin_Yaw, &cos_Yaw );
								targetVel.x = BodyHeading2ENU_x( pitch_sitck_d , -roll_sitck_d , sin_Yaw , cos_Yaw );
								targetVel.y = BodyHeading2ENU_y( pitch_sitck_d , -roll_sitck_d , sin_Yaw , cos_Yaw );
								targetVelNT.x = BodyHeading2ENU_x( pitch_sitck , -roll_sitck , sin_Yaw , cos_Yaw );
								targetVelNT.y = BodyHeading2ENU_y( pitch_sitck , -roll_sitck , sin_Yaw , cos_Yaw );
							}
							else
							{
								double yaw;	double yaw_declination;
								get_YawDeclination(&yaw_declination);
								Attitude_Control_get_TargetTrackYaw(&yaw);
								yaw += yaw_declination;
								double sin_Yaw, cos_Yaw;
								fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
								targetVel.x = BodyHeading2ENU_x( pitch_sitck_d , -roll_sitck_d , sin_Yaw , cos_Yaw );
								targetVel.y = BodyHeading2ENU_y( pitch_sitck_d , -roll_sitck_d , sin_Yaw , cos_Yaw );
								targetVelNT.x = BodyHeading2ENU_x( pitch_sitck , -roll_sitck , sin_Yaw , cos_Yaw );
								targetVelNT.y = BodyHeading2ENU_y( pitch_sitck , -roll_sitck , sin_Yaw , cos_Yaw );
							}
							
							vector3<double> velocityEnu;
							get_VelocityENU_Ctrl(&velocityEnu);
							if( AvMode_Enabled(avCfg->AvoidanceMode[0]) && ( cMode==AFunc_PosHoldAv || cMode==AFunc_PosHoldNHAv ) )
							{	//需要进行避障
								if(!AvLocked)
								{	//未避障锁定才进行避障计算
									double AvDistance = -1;
									vector3<double> tVelVec( targetVelNT.x, targetVelNT.y, 0 );
									double avDist = avCfg->AvoidanceDist[0] + 0.5f*avCfg->wheelbase[0];
									get_AvLineDistanceEnu( &AvDistance, tVelVec, avCfg->wheelbase[0] );

									if( AvDistance >= 0 )
									{
										tVelVec.normalize();
										double currentVel = tVelVec * velocityEnu;
										double sRm = AvDistance - avDist;
										if( sq(currentVel) >= 2*(0.5*getPosCtrlCfg()->maxAccXY[0])*sRm )
											AvLocked = true;
									}
								}
							}
							if( avCfg->fenceEnable[0] & FenceEnable_CpxFenceFlag )
							{	//检测围栏
								if(!AvLocked)
								{
									vector3<double> pos;
									get_Position_Ctrl(&pos);
									vector3<double> tVelVec( targetVelNT.x, targetVelNT.y, 0 );
									//判断距离围栏边界距离
									double dis = -1;
									bool res = is_insideFence(pos, tVelVec, &dis);
									double avDist = 0.5f*avCfg->wheelbase[0];
									if( res )
									{	//在围栏内
										//判断边界距离刹车
										if( dis >= 0 )
										{	//有围栏信息
											tVelVec.normalize();
											double currentVel = tVelVec * velocityEnu;
											double sRm = dis - avDist;
											if( sq(currentVel) >= 2*(0.5*getPosCtrlCfg()->maxAccXY[0])*sRm )
												AvLocked = true;
										}
									}
									else if( dis < 0 )
									{	//在围栏外
										//如果速度方向朝向范围外则刹车
										AvLocked = true;
									}
								}
							}
							
							//俯仰横滚杆控水平速度
							if( AvLocked )
								Position_Control_set_XYLockFast();
							else if( in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] ) && in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) )							
								Position_Control_set_XYLock();
							else
							{
								double RPCtrlScale = atan2( getPosCtrlCfg()->maxAccXY[0] / 50.0, GravityAcc );
								double XYCtrlScale = maxVelXY / 50.0;						
								double vel_stick_err = safe_sqrt( sq(velocityEnu.x/XYCtrlScale - targetVel.x) + sq(velocityEnu.y/XYCtrlScale - targetVel.y) );
								if( vel_stick_err > 50 )
									vel_stick_err = 50;
								targetVel *= XYCtrlScale;
								targetVel.constrain(maxVelXY);
								Position_Control_set_TargetVelocityXY_AngleLimit(
									targetVel.x ,
									targetVel.y ,
									vel_stick_err*RPCtrlScale
								);
							}
						}
					}
					else
					{
						//补偿风力扰动
						vector3<double> WindDisturbance;
						get_WindDisturbance( &WindDisturbance );
						Quaternion attitude;
						get_Attitude_quat(&attitude);
						double yaw = attitude.getYaw();		
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
						double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
						
						//俯仰横滚杆控俯仰横滚
						double RPCtrlScale = degree2rad( getAttCtrlCfg()->maxLean[0] / 50.0 );
//						Attitude_Control_set_Target_RollPitch( 
//							( rc.data[3] - 50 )*RPCtrlScale - atan2(-WindDisturbance_Bodyheading_y , GravityAcc ),
//							( rc.data[2] - 50 )*RPCtrlScale - atan2( WindDisturbance_Bodyheading_x , GravityAcc ) 
//						);
						Attitude_Control_set_Target_RollPitch( 
							( rc.data[3] - 50 )*RPCtrlScale,
							( rc.data[2] - 50 )*RPCtrlScale
						);
					}
					
					//偏航杆在中间锁偏航
					//不在中间控制偏航速度
					double YCtrlScale = degree2rad( getAttCtrlCfg()->maxYSp[0] / 50.0 );
					if( in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) )
						Attitude_Control_set_YawLock();
					else
						Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );							
				}
			}
			else
			{
				//无遥控信号进入安全模式
				Position_Control_Enable();				
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
				if(pos_ena)
					Position_Control_set_XYLock();
				else
					Attitude_Control_set_Target_RollPitch(0,0);
				Position_Control_set_ZLock();
				Attitude_Control_set_YawLock();
				if( noRcStartTIME.get_pass_time() > 2 )
				{
					change_Mode(AFunc_RTL)
					goto RTL;
				}
			}
		}
		
ModeLoopFin:
		/*返回消息处理结果*/
			if( msg_available )
			{
				uint8_t port_index = msg.cmd_type & CMD_TYPE_PORT_MASK;
				const Port* port = get_CommuPort( port_index );
				if( port->write )
				{
					mavlink_message_t msg_sd;
					if( mavlink_lock_chan( port_index, 0.01 ) )
					{
						mavlink_msg_command_ack_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							port_index ,
							&msg_sd,
							msg.cmd,	//command
							msg_handled==1 ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED ,	//result
							100 ,	//progress
							0 ,	//param2
							msg.sd_sysid ,	//target system
							msg.sd_compid //target component
						);
						mavlink_msg_to_send_buffer(port->write, 
																			 port->lock,
																			 port->unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(port_index);
					}
				}
			}
		/*返回消息处理结果*/
	}
	return MR_OK;
}
