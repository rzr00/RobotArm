///////////////////////////////////////////////////////////////////////////////
// RobotArmMain.h
#pragma once

#include "RobotArmCPPInterfaces.h"
#include "PID_Position.h"
#include "Elbow.h"
#include "3rdinterpolate.h"
#include "kinematics.h"

class CRobotArmMain 
	: public ITComObject
	, public ITcADI
	, public ITcWatchSource
///<AutoGeneratedContent id="InheritanceList">
	, public ITcCyclic
///</AutoGeneratedContent>
{
public:
	DECLARE_IUNKNOWN()
	DECLARE_IPERSIST(CID_RobotArmCPPCRobotArmMain)
	DECLARE_ITCOMOBJECT_LOCKOP()
	DECLARE_ITCADI()
	DECLARE_ITCWATCHSOURCE()
	DECLARE_OBJPARAWATCH_MAP()
	DECLARE_OBJDATAAREA_MAP()

	CRobotArmMain();
	virtual	~CRobotArmMain();

	void UpdateInputs();		//更新c++输入
	void UpdateOutputs();		//更新c++输出

	void ShoulderUpdateOutputs();
	void ShoulderElbowUpdateInputs();
	void ShoulderSetTarAngle(double LevelShiftAngle, double RotateAngle);

	void ElbowUpdateInputs();	//更新肘关节c++输入
	void ElbowUpdateOutputs();	//更新肘关节c++输出

	double wrist_angle_out(double resistor);//腕关节角度计算
	int wrist_motion(double target);//腕关节移动控制函数
	void hand_motion(int time_one, int time_two, int time_three, int time_four);//手部抓握控制函数
	double hand_temp_out(double num);//手部温度计算
///<AutoGeneratedContent id="InterfaceMembers">
	// ITcCyclic
	virtual HRESULT TCOMAPI CycleUpdate(ITcTask* ipTask, ITcUnknown* ipCaller, ULONG_PTR context);

///</AutoGeneratedContent>

protected:
	DECLARE_ITCOMOBJECT_SETSTATE();

	HRESULT AddModuleToCaller();
	VOID RemoveModuleFromCaller();

	// Tracing
	CTcTrace m_Trace;

///<AutoGeneratedContent id="Members">
	TcTraceLevel m_TraceLevelMax;
	RobotArmMainParameter m_Parameter;
	RobotArmMainInputs m_Inputs;
	RobotArmMainOutputs m_Outputs;
	RobotArmMainPlcToCpp m_PlcToCpp;
	RobotArmMainCppToPlc m_CppToPlc;
	ITcCyclicCallerInfoPtr m_spCyclicCaller;
///</AutoGeneratedContent>

	// TODO: Custom variable
	const double pi;
	double timer;								//plc时间
	double AngleLevelShift;
	double AngleRotate;
	double TarPosLevelShift;					//肩关节平移目标位置值
	double TarPosRotate;						//肩关节旋转目标位置值
	bool ShoulderInitFinish;

	Elbow elbow;
	Polynomial ShoulderLevelShiftPolynomial;	//肩关节平移角度多项式
	Polynomial ElbowPolynomial;					//肘关节角度多项式
	Polynomial WristPolynomial;					//腕关节角度多项式

	int ErrorCode;

	//运动规划测试数据
	//TODO:将数组改为结构体，提高程序可读性
	double KinematicsInverseData[4][5];																		//反解数据值：Ax, Ay, Aphi, t0, tf
	double CurrentPositionData[3];									//当前的反解角度值: 肩、肘、腕
	double NextPositionData[3];										//下一个位置的反解角度值: 肩、肘、腕
	int PositionSize;
	int PositionNum;
	int PositionStatus;
	int InitElbowStatus;
	int StopStatus;
	double StopStartTime;
	double StopStartM1;
	double StopStartM2;

	double const k;//角度与电阻转换公式jiaoud=k*res+b
	double const b;//角度与电阻转换公式jiaodu=k*res+b
	double const temp_k;//手部温度转换系数temp=temp_k*AN+temp_b
	double const temp_b;//手部温度转换系数temp=temp_k*AN+temp_b
	int i;//腕关节角度检测延迟
	double wrist_init_angle;//腕关节初始角度
	double wrist_target_angle;//腕关节目标角度,合并后删除
	double wrist_fact_angle;//腕关节实际角度
	double wrist_pwm_out;//腕关节合金丝pwm输出
	double wrist_pwm_max;//腕关节pwm输出上限值
	double wrist_pwm_min;//腕关节pwm输出下限值，一般设为0
	double wrist_angle_max;//腕关节角度运动上限值
	double wrist_error;//腕关节角度误差
	int wrist_motion_flag;//腕关节到达预期位置标志位
	int hand_motion_flag;//手部完成抓握物体标志位
	int time_one, time_two, time_three, time_four;
};
