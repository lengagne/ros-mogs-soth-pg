//      MogsOrientationConstraint.h
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

#ifndef __MogsOrientationConstraint__
#define __MogsOrientationConstraint__

#include "MogsKinematics.h"
#include "MogsAbstractSothConstraint.h"


#ifndef __MOGSEndEffectorConstraint__
#define __MOGSEndEffectorConstraint__
enum mode{
	DESIRED,	// set of desired position
	JOYSTICK,	// the desired position is given by a joystick
	BODY		// the desired position is one of an other robot body
};
#endif


class MogsOrientationConstraint: public MogsAbstractSothConstraint
{
      public:


	MogsOrientationConstraint(	int body_id, bool x, bool y, bool z, double vx, double vy, double vz,
					Eigen::Matrix <double,3, 1 > offset = Eigen::Matrix < double,3, 1 >::Zero()
	);

	MogsOrientationConstraint(  QDomElement pg_root,
                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots);

// 	MogsOrientationConstraint(	tinyxml2::XMLElement * pg_root,
// 					RigidBodyDynamics::MogsKinematics<F<double> >* kin);

	~MogsOrientationConstraint();

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
		int cpt = 0;
		for (int i=0;i<3;i++)	if(ctr_[i])
			cpt++;
		return cpt;
	}

      private:


	std::vector<double> value_;
	std::vector<bool> ctr_;
	unsigned int body_id_;
        unsigned int body_ref_;
        
	// use in mode body
	int target_robot_id_;
	int target_body_id_;

        mode desired_mode_;        

	Eigen::Matrix < F<double>,3, 1 > Ftmp_;
	
	SpatialTransform<F<double>> FTransform_;
	
	Eigen::Matrix <F<double>,3, 3 > FTransform_tmp_;
	
	Eigen::Matrix <double,3, 3 > offset_;
        
        Eigen::Matrix<double,3,3>  TargetRotation_;
        
};


#endif
