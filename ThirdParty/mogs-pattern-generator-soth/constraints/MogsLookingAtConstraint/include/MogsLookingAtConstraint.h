//      MogsLookingAtConstraint.h
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

#ifndef __MOGSLookingAtConstraint__
#define __MOGSLookingAtConstraint__

#ifdef MogsJoystick_FOUND
#include "MogsJoystick.h"
#endif
#include "MogsKinematics.h"
#include "MogsAbstractSothConstraint.h"

class MogsLookingAtConstraint: public MogsAbstractSothConstraint
{
      public:

	MogsLookingAtConstraint();

        MogsLookingAtConstraint(    int body_id,
                                    const Eigen::Matrix<double,3,1> position1,
                                    const Eigen::Matrix<double,3,1> position2,
                                    const Eigen::Matrix<double,3,1> target,
                                    unsigned int ref = 0);

	MogsLookingAtConstraint(  QDomElement pg_root,
                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots = NULL);

	~MogsLookingAtConstraint();

	void compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
			RigidBodyDynamics::MogsKinematics<F<double> > * kin,
			HQPSolver::Priority* task_info,
			std::vector< RigidBodyDynamics::MogsKinematics< double > *>*all_the_robots = NULL);
        
        Eigen::Matrix<double,3,1> get_current_effector_position()const
        {
            return current_effector_position_;
        }

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

	void set_desired_position(const Eigen::Matrix < double,3, 1 >& in )
	{
            desired_positions_.clear();
            desired_positions_.push_back(in);
            desired_position_ = in;
	}

	Eigen::Matrix < double,3, 1 > get_desired_position(  ) const
	{
		return desired_position_;
	}

	double get_distance()const
	{
	    return distance_;
	}

      private:

	Eigen::Matrix < double,3, 1 > desired_position_;
	Eigen::Matrix < double,3, 1 > pos1_,pos2_;
        Eigen::Matrix<double,3,1> current_effector_position_;
        
	unsigned int body_id_;
        unsigned int body_ref_;


	std::vector< Eigen::Matrix < double,3, 1 > > desired_positions_;
	int count_position_;
	Eigen::Matrix < F<double>,3, 1 > Ftmp1_, Ftmp2_, Ftmp_target_;

	// use in mode body
	int target_robot_id_;
	int target_body_id_;

	double distance_;
};


#endif
