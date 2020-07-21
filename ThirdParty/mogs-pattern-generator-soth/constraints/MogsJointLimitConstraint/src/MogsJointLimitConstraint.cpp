//      MogsJointLimitConstraint.cpp
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
//      2009-2011:  Joint robotics Laboratory - CNRS/AISdouble,Tsukuba, Japan.
//      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
//      2012-2013:  IUT de Beziers/ LIRMM, Beziers, France
//	from 2013:  Université Blaise Pascal / axis : ISPR / theme MACCS
#include "MogsJointLimitConstraint.h"



MogsJointLimitConstraint::MogsJointLimitConstraint(RigidBodyDynamics::MogsRobotProperties * robot)
{
	if (robot->is_robot_floating_base())
		start_dof_ = 6;
	else
		start_dof_ = 0;

	nb_dof_ = robot->getNDof();
	robot->getPositionLimit(min_joint_limits_,max_joint_limits_);
// 	for(int i=0;i<robot->getNDof();i++)
//        std::cout<<"joint_limits_ = ["<< min_joint_limits_[i]<<" : "<< max_joint_limits_[i]<<"]"<<std::endl;
}

MogsJointLimitConstraint::~MogsJointLimitConstraint()
{

}

void MogsJointLimitConstraint::compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
						RigidBodyDynamics::MogsKinematics<F<double> > * kin,
						HQPSolver::Priority* task_info,
						std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
	int cpt = 0;
	 for (int i=start_dof_;i<nb_dof_;i++)
	 {
		for(int j=0;j<nb_dof_;j++)
		{
			if (i==j)
				task_info->Jacobian(cpt,j) = 1;
			else
				task_info->Jacobian(cpt,j) = 0;
		}
		task_info->error(cpt) = min_joint_limits_[i] - Q(i).x();
		cpt++;
		for(int j=0;j<nb_dof_;j++)
		{
			if (i==j)
				task_info->Jacobian(cpt,j) = -1;
			else
				task_info->Jacobian(cpt,j) = 0;
		}
		task_info->error(cpt) = Q(i).x() - max_joint_limits_[i];
		cpt++;
// 		std::cout<<"q("<<i<<") = "<< Q(i).x() <<"  in "<< min_joint_limits_[i] <<" : "<< max_joint_limits_[i]<<std::endl;
	 }
// 	 std::cout<<"error = "<< task_info->error<<std::endl;
// 	 std::cout<<"Jacobian = "<< task_info->Jacobian<<std::endl;
}

extern "C" MogsJointLimitConstraint* create(QDomElement pg_root,
                                            RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                            std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)

{
    return new MogsJointLimitConstraint( kin->model);
}

extern "C" void destroy(MogsJointLimitConstraint* p)
{
    delete p;
}
