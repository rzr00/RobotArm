// RobotArmCPPCtrl.cpp : Implementation of CTcRobotArmCPPCtrl
#include "TcPch.h"
#pragma hdrstop

#include "RobotArmCPPW32.h"
#include "RobotArmCPPCtrl.h"

/////////////////////////////////////////////////////////////////////////////
// CRobotArmCPPCtrl

CRobotArmCPPCtrl::CRobotArmCPPCtrl() 
	: ITcOCFCtrlImpl<CRobotArmCPPCtrl, CRobotArmCPPClassFactory>() 
{
}

CRobotArmCPPCtrl::~CRobotArmCPPCtrl()
{
}

