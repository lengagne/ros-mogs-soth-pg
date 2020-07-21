//      MogsJointEqualityConstraint.h
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
//	from 2013:  Université Clermont Auvergne Pascal / axis : ISPR / theme MACCS

#ifndef __MOGSJointEqualityConstraint__
#define __MOGSJointEqualityConstraint__


/**            <constraint type="JointEqualityConstraint">
 * required               <joint1> RAnkleRoll</joint1>
 * required               <joint2> LAnkleRoll</joint2>
 * optional               <mulitplier> 1.0</mulitplier> 
 * optional               <offset> 0.0</mulitplier> 
 *            </constraint>       
 * */

#include "MogsKinematics.h"
#include "MogsAbstractSothConstraint.h"

class MogsJointEqualityConstraint: public MogsAbstractSothConstraint
{
      public:

        MogsJointEqualityConstraint(    unsigned int joint_id1,
                                        unsigned int joint_id2,
                                        double mul=1.0,
                                        double offset=0.0);

	MogsJointEqualityConstraint(  QDomElement pg_root,
                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots = NULL);        

	~MogsJointEqualityConstraint();

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

	unsigned int joint_id1_,joint_id2_;
        double coeff_mul_, offset_;
};



#endif
