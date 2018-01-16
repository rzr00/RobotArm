///////////////////////////////////////////////////////////////////////////////
// RobotArmCPPCtrl.h

#ifndef __ROBOTARMCPPCTRL_H__
#define __ROBOTARMCPPCTRL_H__

#include <atlbase.h>
#include <atlcom.h>

#define ROBOTARMCPPDRV_NAME "ROBOTARMCPP"

#include "resource.h"       // main symbols
#include "RobotArmCPPW32.h"
#include "TcBase.h"
#include "RobotArmCPPClassFactory.h"
#include "TcOCFCtrlImpl.h"

class CRobotArmCPPCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<CRobotArmCPPCtrl, &CLSID_RobotArmCPPCtrl>
	, public IRobotArmCPPCtrl
	, public ITcOCFCtrlImpl<CRobotArmCPPCtrl, CRobotArmCPPClassFactory>
{
public:
	CRobotArmCPPCtrl();
	virtual ~CRobotArmCPPCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_ROBOTARMCPPCTRL)
DECLARE_NOT_AGGREGATABLE(CRobotArmCPPCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CRobotArmCPPCtrl)
	COM_INTERFACE_ENTRY(IRobotArmCPPCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __ROBOTARMCPPCTRL_H__
