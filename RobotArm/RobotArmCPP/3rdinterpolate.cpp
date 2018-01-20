/*
*  3rdinterpolate.h
*  机械臂 插补
*  Created on: 2018-1-18
*  Author: Leo
*/

#pragma once
#include "3rdinterpolate.h"
#include "Fpu87.h"

double myfmax(double x, double y)
{
	return (x > y ? x : y);
}

// 通用插补--算法扩展
bool R_INTERP::planProfile()
{
	if (_pTarget == _pStart)
	{
		//DEBUG("please input valid target!\n");
	}

	if (_v_limit == 0.0 || _a_limit == 0.0)
	{
		//DEBUG("please input valid velocity or acceleration limit!\n");
	}

	switch(_pType)
	{
	case polynomial_3rd_T:
		if (_tTerminal == 0 || _tTerminal<=_tStart)
		{
			//DEBUG("please input valid terminal time!\n");
		}
		if (_polynomial == nullptr)
		{
			//_polynomial = new Polynomial(); //FIXME:倍福new对象报错，倍福可以新建对象。
		}
		_pCurProfileType = _polynomial;
		_polynomial->plan3rdProfileT(_tStart,_tTerminal,_pStart,_pTarget,_vStart,_vTarget);
		break;
	}

	return isValidMovement();
}

bool R_INTERP::plan3rdProfileT(double t0, double tf, double p0,double pf, double v0, double vf)
{
	setStartTime(t0);
	setTargetTime(tf);
	setStart(p0,v0);
	setTarget(pf,vf);

	if (_polynomial == nullptr)
	{
		//_polynomial = new Polynomial();
	}
	_pCurProfileType = _polynomial;
	_polynomial->plan3rdProfileT(t0,tf,p0,pf,v0,vf);
	return _polynomial->isValidMovement();
}

// 3次多项式
void Polynomial::plan3rdProfileT( double t0, double tf,double p0,double pf, double v0, double vf)
{
	_plannedProfile = false;

	_pType = polynomial_3rd_T;
	_t[0] = t0;
	_t[1] = tf;
	_v[0] = v0;
	_v[1] = vf;
	_x[0] = p0;   
	_x[1] = pf;   
	_a[0] = 0.0;
	_a[1] = 0.0;

	if (tf <= t0)
	{
		_plannedProfile = false;
		//DEBUG("start and terminal time cannot be the same!\n");
		return;
	}

	// the 3rd polynomial profile:
	// p(t) = a*(t-t0)^3 + b*(t-t0)^2 + c*(t-t0) +d
	// p(t0) = d = p0
	// p(tf) = pf

	// v(t) = 3*a*(t-t0)^2 + 2*b*(t-t0) + c
	// v(t0) = c = v0
	// v(tf)  = vf

	double a, b, c, d, T;

	T = tf - t0;
	
	d = p0;
	c = v0;
	b = (3.0*pf - vf*T - 2.0*v0*T -3.0*p0)/(T*T);
	a = (pf - b*T*T - v0*T - p0)/(T*T*T);
	
	_coefficient[0] = 0.0;	    // t^5
	_coefficient[1] = 0.0;	    // t^4
	_coefficient[2] = a;		// t^3
	_coefficient[3] = b;		// t^2
	_coefficient[4] = c;		// t^1
	_coefficient[5] = d;		// t^0

	_plannedProfile = true;
}

double Polynomial::scaleToDuration(double newDuration)
{
	if (_pType == polynomial_3rd_T)
	{
		plan3rdProfileT(_t[0],_t[0] + newDuration,_x[0],_x[1],_v[0],_v[1]);
	}
	return getDuration();
}

bool Polynomial::isValidMovement() const
{
	return _plannedProfile;
}

double Polynomial::getDuration() const
{
	return _t[1] - _t[0];
}

double Polynomial::pos(double t) const
{
	t = (t-_t[0]);

	if (t < 0)
	{
		t = 0;
	}

	if (t > (_t[1] - _t[0]))
	{
		t = _t[1] - _t[0];
	}

	double p = _coefficient[0]*t*t*t*t*t + _coefficient[1]*t*t*t*t + _coefficient[2]*t*t*t + _coefficient[3]*t*t + _coefficient[4]*t + _coefficient[5];
	return p;
}

double Polynomial::vel(double t) const
{
	t = (t-_t[0]);

	if (t < 0)
	{
		t = 0;
	}

	if (t > (_t[1] - _t[0]))
	{
		t = _t[1] - _t[0];
	}

	double v = 5*_coefficient[0]*t*t*t*t + 4*_coefficient[1]*t*t*t + 3*_coefficient[2]*t*t + 2*_coefficient[3]*t + _coefficient[4];
	return v;
}

double Polynomial::acc(double t) const
{
	t = (t-_t[0]);

	if (t < 0)
	{
		t = 0;
	}

	if (t > (_t[1] - _t[0]))
	{
		t = _t[1] - _t[0];
	}

	double a = 20*_coefficient[0]*t*t*t + 12*_coefficient[1]*t*t+ 6*_coefficient[2]*t + 2*_coefficient[3];
	return a;
}

double Polynomial::jerk(double t)const
{
	t = (t-_t[0]);

	if (t < 0)
	{
		t = 0;
	}

	if (t > (_t[1] - _t[0]))
	{
		t = _t[1] - _t[0];
	}

	double j = 60*_coefficient[0]*t*t + 24*_coefficient[1]*t +6*_coefficient[2];
	return j;
}

double Polynomial::max_vel()const
{
	double temp,vmax;
	switch (_pType)
	{
	case polynomial_3rd_T:

		temp = -2.0*_coefficient[3]/(6.0*_coefficient[2]);
		vmax = myfmax(myfmax(fabs_(_v[0]), fabs_(_v[1])), fabs_(vel(temp)));
		//vmax = fmax(fmax(fabs_(_v[0]), fabs_(_v[1])), fabs_(vel(temp)));

		if (vmax == fabs_(vel(_t[0])))
		{
			vmax = vel(_t[0]);
		}

		if (vmax == fabs_(vel(_t[1])))
		{
			vmax = vel(_t[1]);
		}

		if (vmax == fabs_(vel(temp)))
		{
			vmax = vel(temp);
		}
		return vmax;

		break;
	}

	return 0xffffffff;
}

double Polynomial::max_acc()const
{
	switch (_pType)
	{
	case polynomial_3rd_T:

		return myfmax(fabs_(acc(_t[0])), fabs_(acc(_t[1])));

		break;
	}

	return 0;
}

double Polynomial::max_jerk()const
{
	switch (_pType)
	{
	case polynomial_3rd_T:
		return jerk(_t[0]);
		break;
	}
	return 0;
}
