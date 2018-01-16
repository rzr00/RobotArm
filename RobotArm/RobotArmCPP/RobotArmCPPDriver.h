///////////////////////////////////////////////////////////////////////////////
// RobotArmCPPDriver.h

#ifndef __ROBOTARMCPPDRIVER_H__
#define __ROBOTARMCPPDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define ROBOTARMCPPDRV_NAME        "ROBOTARMCPP"
#define ROBOTARMCPPDRV_Major       1
#define ROBOTARMCPPDRV_Minor       0

#define DEVICE_CLASS CRobotArmCPPDriver

#include "ObjDriver.h"

class CRobotArmCPPDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl ROBOTARMCPPDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(ROBOTARMCPPDRV)
	VxD_Service( ROBOTARMCPPDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __ROBOTARMCPPDRIVER_H__