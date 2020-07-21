//      MogsIKPatternGenerator.cpp
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


#include "MogsIKPatternGenerator.h"
#include <Eigen/LU>

MogsIKPatternGenerator::MogsIKPatternGenerator()
{
	t_start_ = 0.0;
	joystick_ = new MogsJoystick();
	if(!joystick_->ready())
	{
		std::cerr<<"Error you must connect a joystick"<<std::endl;
	}
}

MogsIKPatternGenerator::~MogsIKPatternGenerator()
{

}

void MogsIKPatternGenerator::read_xml(QDomElement PG_root)
{
	MogsAbstractPatternGenerator::read_xml(PG_root);
}

bool MogsIKPatternGenerator::compute(	double time,
						Eigen::Matrix <double,Eigen::Dynamic, 1 > *Q,
						std::vector< RigidBodyDynamics::MogsKinematics<double> *> * kins,
						Eigen::Matrix <double,Eigen::Dynamic, 1 > *DQ,
						Eigen::Matrix <double,Eigen::Dynamic, 1 > *DDQ,
						Eigen::Matrix <double,Eigen::Dynamic, 1 > *torques)
{
	if (time < 1)
	{
		for (int i =0;i<nb_dof_;i++)
			(*Q)(i) = sin(0.1*i);
		return true;
	}

	assert( !DQ && !DDQ && !torques);
	if(joystick_->get_stop())
		return false;

	desired_position_(0) += joystick_->get_forward_velocity() * (time - t_start_);
	desired_position_(1) += joystick_->get_side_velocity() * (time - t_start_);
	desired_position_(2) += joystick_->get_up_velocity() * (time - t_start_);

	Eigen::Matrix<double,3,1> tmp(0,0,0);
	kin_->CalcPointJacobian (*Q,kin_->getNBodies()-1,tmp,jac_,true);
	current_position_ = kin_->getBodyToBaseCoordinates( kin_->getNBodies()-1);

	(*Q) += jac_.transpose() * ( jac_ * jac_.transpose()).inverse() * (desired_position_ - current_position_) * 0.5*(time - t_start_);

	t_start_ = time;
	std::cout<<" desired_position_= "<< desired_position_.transpose() <<"\r";
// 	std::cout<<" Q= "<< (*Q).transpose() <<std::endl;
	return true;
}

void MogsIKPatternGenerator::set_robot(RigidBodyDynamics::MogsRobotProperties * in)
{
	MogsAbstractPatternGenerator::set_robot(in);
	jac_.resize(3,nb_dof_);

	kin_ = new RigidBodyDynamics::MogsKinematics<double>();
	kin_->SetRobot(in);
	Eigen::Matrix<double,Eigen::Dynamic,1> q(nb_dof_);
	for (int i=0;i<nb_dof_;i++)
		q(i) = 0.0;
	kin_->UpdateKinematicsCustom(&q);
	desired_position_ = kin_->getBodyToBaseCoordinates( kin_->getNBodies()-1);
}

extern "C" MogsIKPatternGenerator* create()
{
    return new MogsIKPatternGenerator();
}

extern "C" void destroy(MogsIKPatternGenerator* p)
{
    delete p;
}
