//      MogsAvoidObjectCollisionConstraint.h
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

#ifndef __MogsAvoidObjectCollisionConstraint__
#define __MogsAvoidObjectCollisionConstraint__

#include "MogsKinematics.h"
#include "MogsAbstractSothConstraint.h"
/**	Allows to avoid Object and robot collision with the current robot
*/

#define MogsAvoidObjectCollisionConstraint_tolerance 0.2

struct collision
{
	int body_id_1;
	int body_id_2;
};

class MogsAvoidObjectCollisionConstraint: public MogsAbstractSothConstraint
{
      public:

	MogsAvoidObjectCollisionConstraint();

	MogsAvoidObjectCollisionConstraint( QDomElement pg_root,
                                            RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                            std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots);

	MogsAvoidObjectCollisionConstraint( QString & robot_name,
                                            RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                            std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
    {
        init(robot_name,kin,all_the_robots);
    }

    void init(  QString & robot_name,
                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots);

	~MogsAvoidObjectCollisionConstraint();

	void compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
			RigidBodyDynamics::MogsKinematics<F<double> > * kin,
			HQPSolver::Priority* task_info,
			std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots);

	int get_inequalities(int count) const // 0: equality,  1: lower inequality, 2: upper inequality,
	{
		return 1;
	}

	double get_max(int count) const
	{
		return 0;
	}

	int get_task_size()  const
	{
		return nb_collisions_;
	}

      private:

	int num_robot_;
	int nb_collisions_;
	std::vector<collision> collisions_;
};


#endif
