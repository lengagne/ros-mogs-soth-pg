//      MogsDefaultPatternGenerator.cpp
//      Copyright (C) 2012 lengagne (lengagne@gmail.com)
//
//      This program is free software: you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation, either version 3 of the License, or
//      (at your option) any later version.
//
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//
//      You should have received a copy of the GNU General Public License
//      along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//      This program was developped in the following labs:
//      2009-2011:  Joint robotics Laboratory - CNRS/AIST,Tsukuba, Japan.
//      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
//      2012-2013:  IUT de Beziers/ LIRMM, Beziers, France
//	from 2013:  Université Blaise Pascal / axis : ISPR / theme MACCS


#include "MogsDefaultPatternGenerator.h"

MogsDefaultPatternGenerator::MogsDefaultPatternGenerator()
{
	t_start_ = 0.0;
	count_dof_ = 0;
	joystick_ = new MogsJoystick();
	use_joystick_ = joystick_->ready();
	mem_ = false;
}

MogsDefaultPatternGenerator::~MogsDefaultPatternGenerator()
{

}

void MogsDefaultPatternGenerator::read_xml(QDomElement PG_root)
{
	MogsAbstractPatternGenerator::read_xml(PG_root);
}

bool MogsDefaultPatternGenerator::compute(	double time,
						Eigen::Matrix <double,Eigen::Dynamic, 1 > *Q,
						std::vector< RigidBodyDynamics::MogsKinematics<double> *> * kins,
						Eigen::Matrix <double,Eigen::Dynamic, 1 > *DQ,
						Eigen::Matrix <double,Eigen::Dynamic, 1 > *DDQ,
						Eigen::Matrix <double,Eigen::Dynamic, 1 > *torques)
{
	assert( !DDQ && !torques);
	for (int i=0;i<nb_dof_;i++)	if (i != count_dof_)
	{
//		(*Q)[i] = 0.;
		if(DQ) (*DQ)[i] = 0.;
	}
	if  (! use_joystick_)
	{
		if (time - t_start_ > 2*Pi)
		{
			t_start_ = time;
//			(*Q)[count_dof_] = 0.;
			if(DQ) (*DQ)[count_dof_] = 0.;
			count_dof_ ++;
		}
		if (count_dof_ >= nb_dof_)
			return false;

		(*Q)[count_dof_] = 0; //sin(time - t_start_);
		if(DQ) (*DQ)[count_dof_] = 0; //cos(time - t_start_);
		return true;
	}else
	{
		if(joystick_->get_stop())
			return false;

		if (joystick_->get_pause())
		{
			if (!mem_)
			{
				mem_ = true;
				count_dof_ ++;
				if (count_dof_ >= nb_dof_)
					count_dof_ = 0;
				std::cout<<"button pushed  count_dof_ = "<< count_dof_<<"\tnb_dof_ = "<< nb_dof_<<std::endl;
			}
		}else
			mem_ = false;

		(*Q)[count_dof_] += joystick_->get_side_velocity() * (time - t_start_);
		if(DQ) (*DQ)[count_dof_] = joystick_->get_side_velocity() ;
		t_start_ = time;
 		std::cout<<" Q["<< count_dof_<<"] = "<< (*Q)[count_dof_] <<"\t\r";
		return true;
	}
}

extern "C" MogsDefaultPatternGenerator* create()
{
    return new MogsDefaultPatternGenerator();
}

extern "C" void destroy(MogsDefaultPatternGenerator* p)
{
    delete p;
}
