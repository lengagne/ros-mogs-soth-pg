//      MogsMinimizeQConstraint.cpp
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

#include "MogsMinimizeQConstraint.h"

MogsMinimizeQConstraint::MogsMinimizeQConstraint(RigidBodyDynamics::MogsRobotProperties * robot)
{
	if (robot->is_robot_floating_base())
		start_dof_ = 6;
	else
		start_dof_ = 0;

	nb_dof_ = robot->getNDof();
}

MogsMinimizeQConstraint::~MogsMinimizeQConstraint()
{

}

void MogsMinimizeQConstraint::compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
						RigidBodyDynamics::MogsKinematics<F<double> > * kin,
						HQPSolver::Priority* task_info,
						std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
	int cpt = 0;
	 for (int i=start_dof_;i<nb_dof_;i++)
	 {
		task_info->error(cpt) = - Q(i).x();
		for(int j=0;j<nb_dof_;j++)
		{
			if (i==j)
				task_info->Jacobian(cpt,j) = 1;
			else
				task_info->Jacobian(cpt,j) = 0;
		}
		cpt++;
	 }
}

extern "C" MogsMinimizeQConstraint* create( QDomElement pg_root,
                                            RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                            std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new MogsMinimizeQConstraint(kin->model);
}

extern "C" void destroy(MogsMinimizeQConstraint* p)
{
    delete p;
}
