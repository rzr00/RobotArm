///////////////////////////////////////////////////////////////////////////////
// RobotArmCPPDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "RobotArmCPPDriver.h"
#include "RobotArmCPPClassFactory.h"

DECLARE_GENERIC_DEVICE(ROBOTARMCPPDRV)

IOSTATUS CRobotArmCPPDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CRobotArmCPPClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CRobotArmCPPDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CRobotArmCPPDriver::ROBOTARMCPPDRV_GetVersion( )
{
	return( (ROBOTARMCPPDRV_Major << 8) | ROBOTARMCPPDRV_Minor );
}

