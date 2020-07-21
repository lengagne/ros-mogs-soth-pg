//      MogsSameZConstraint.h
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

#ifndef __MOGSSameZConstraint__
#define __MOGSSameZConstraint__

#include "MogsKinematics.h"
#include "MogsAbstractSothConstraint.h"

class MogsSameZConstraint: public MogsAbstractSothConstraint
{
      public:

	MogsSameZConstraint(	int body_id1,
                                const Eigen::Matrix<double,3,1> position1,
                                int body_id2,
                                const Eigen::Matrix<double,3,1> position2);

	MogsSameZConstraint(	int body_id1,
                                const Eigen::Matrix<double,3,1> position1,
                                const Eigen::Matrix<double,3,1> position2):MogsSameZConstraint(body_id1,position1,body_id1,position2)
        {
            
        }

	MogsSameZConstraint(  QDomElement pg_root,
                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots = NULL);        

	~MogsSameZConstraint();

	void compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
			RigidBodyDynamics::MogsKinematics<F<double> > * kin,
			HQPSolver::Priority* task_info,
			std::vector< RigidBodyDynamics::MogsKinematics< double > *>*all_the_robots = NULL
    		);
        

	int get_inequalities(int count) const // 0: equality,  1: lower inequality, 2: upper inequality,
	{
		return 0;
	}

	double get_max(int count) const
	{
		return 0.;
	}

	int get_task_size()  const
	{
		return 1;
	}


      private:

	Eigen::Matrix < double,3, 1 > effector_position1_;
	Eigen::Matrix < double,3, 1 > effector_position2_;
	unsigned int body_id1_,body_id2_;
        
        Eigen::Matrix < F<double>,3, 1 > Ftmp1_, Ftmp2_;
};



#endif
