/*
*  3rdinterpolate.h
*  机械臂 插补
*  Created on: 2018-1-18
*  Author: Leo
*/

#pragma once
//#include <math.h>
#include <string.h>
//#include <iostream>

enum ProfileType		  // 枚举类型
{
	unDefine,			 // 未指定类型
	polynomial_3rd_T,	// 3次多项式
};

// 1、接口类定义
class R_INTERP_BASE
{
public:
	R_INTERP_BASE(){ };

	~R_INTERP_BASE(){ };

	// returns the current profile type
	virtual ProfileType profileType() const = 0;

	/// returns true if all time intervals are non negative
	/// and all pos vel acc values are inside the limits.
	virtual bool isValidMovement() const = 0;	
	
	/// get total duration  持续时间  of the trajectory.            
	virtual double getDuration() const = 0;

	/// get the position, velocity and acceleration at passed time >= 0  
	virtual double pos(double t) const = 0;
	virtual double vel(double t) const = 0;
	virtual double acc(double t) const = 0;
	virtual double jerk(double t) const { return 0;}    //加速度变化率

	// get the maximum velocity acceleration and jerk at of the profile
	virtual double max_vel() const = 0;
	virtual double max_acc() const = 0;
	virtual double max_jerk() const = 0;

	// scale 测量 a planned profile 外形 to a longer duration 持续时间    
	virtual double scaleToDuration(double newDuration){ return newDuration; };
};


// 2、3次多项式类定义
class Polynomial:public R_INTERP_BASE
{

public:

	Polynomial()
	{
	//	_plannedProfile = false;
	}

	~Polynomial()
	{
		//delete[] Coefficient系数;
	};
	
	virtual double getDuration() const;
	virtual double pos(double t) const;
	virtual double vel(double t) const;
	virtual double acc(double t) const;
	virtual double jerk(double t) const;
	virtual bool   isValidMovement() const;
	virtual double scaleToDuration(double newDuration);
	virtual ProfileType profileType()const{ return _pType;}

	virtual double max_vel() const;
	virtual double max_acc() const;
	virtual double max_jerk() const;

	// 定时
	void plan3rdProfileT(double t0, double tf, double p0, double pf, double v0, double vf);

	//返回多项式参数
	void Show_coefficient(double* coe);

private:

	double _x[2];
	double _v[2];				// ?[0] is start condition, ?[3] is end condition.
	double _t[2];			
	double _a[2];	
	ProfileType _pType;			// 3rd or 5th polynomial		
	bool _plannedProfile;		// flag indication whether a profile was computed
	double _coefficient[6];		// Coefficients of the polynomial      _coefficient[0]- _coefficient[5] 分别代表5次-0次
};

// 3、通用插补类定义——扩展算法接口
class R_INTERP
{
public:

	// constructor
	R_INTERP()
	{
		_pType = unDefine;
		_tStart = 0.0;
		_tTerminal = 0.0;
		_pStart = 0.0;
		_pTarget = 0.0;
		_vStart = 0.0;
		_vTarget = 0.0;
		_aStart = 0.0;
		_aTarget = 0.0;
		_v_limit = 0.0;
		_a_limit = 0.0;
		_j_limit = 0.0;
		_polynomial = nullptr;
		_pCurProfileType = nullptr;
	}

	// destructor
	~R_INTERP()
	{
		_polynomial = nullptr;
		_pCurProfileType = nullptr;
	}

	// set limit values for movement parameters
	void setLimit (double vel_Limit, double acc_Limit, double jerk_Limit)
	{_v_limit = vel_Limit;_a_limit = acc_Limit;_j_limit = jerk_Limit;}

	void setLimit (double vel_Limit, double acc_Limit)
	{setLimit (vel_Limit, acc_Limit, acc_Limit*3.0);}

	void setLimit (double vel_Limit)
	{setLimit (vel_Limit, vel_Limit*3.0);}

	// set target values for movement parameters
	void setTarget(double pf, double vf, double af)
	{_pTarget = pf; _vTarget = vf; _aTarget = af;}

	void setTarget(double pf, double vf)
	{setTarget(pf,vf,0.0);}

	void setTarget(double pf)
	{setTarget(pf,0.0,0.0);}

	// set start values for movement parameters
	void setStart(double p0,double v0,double a0)
	{_pStart = p0; _vStart = v0; _aStart = a0;}

	void setStart(double p0,double v0)
	{setStart(p0,v0,0.0);}

	void setStart(double p0)
	{setStart(p0,0.0,0.0);}

	// set the start time 
	void setStartTime(double t0){_tStart = t0;}

	// for some profile type, like TRIGONOMETRIC,
	//   terminal time needed to be given
	void setTargetTime(double tf){_tTerminal = tf;}

	// set the profile type for movement parameters
	void setProfileType(ProfileType profile)
	{_pType = profile;}

	// Returns the duration the full movement will need.
	double getDuration() const {return _pCurProfileType->getDuration();}	
	
	// returns true if all time intervals are non negative
	// and all pos vel acc values are inside the limits.
	bool isValidMovement() const{return _pCurProfileType->isValidMovement();}		
	
	// Scales an already planned profile to a longer duration.
	double scaleToDuration(double newDuration)
	{return _pCurProfileType->scaleToDuration(newDuration);}

	// get the position, velocity and acceleration at passed time >= 0   
	double pos(double t) const {return _pCurProfileType->pos(t);}
	double vel(double t) const {return _pCurProfileType->vel(t);}
	double acc(double t) const {return _pCurProfileType->acc(t);}
	double jerk(double t) const {return _pCurProfileType->jerk(t);}

	/// plan the time optimal profile
	bool planProfile();

	bool plan3rdProfileT(double t0, double tf, double p0,double pf, double v0, double vf);

private:
	
	ProfileType		_pType;
	R_INTERP(const R_INTERP&);

	Polynomial*		_polynomial;
	R_INTERP_BASE*	_pCurProfileType;

	double _pTarget;	///< target position
	double _vTarget;	///< target velocity
	double _aTarget;	///< target acceleration

	double _pStart;		///< start position
	double _vStart;		///< start velocity
	double _aStart;		///< start acceleration	

	double _tStart;		///< start time
	double _tTerminal;	///< terminal time

	double _v_limit;	///< limit for velocity
	double _a_limit;	///< limit for acceleration
	double _j_limit;	///< limit for jerk
};