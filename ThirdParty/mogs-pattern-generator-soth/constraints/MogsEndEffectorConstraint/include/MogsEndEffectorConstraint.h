//      MogsEndEffectorConstraint.h
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

#ifndef __MOGSEndEffectorConstraint__
#define __MOGSEndEffectorConstraint__

#ifdef MogsJoystick_FOUND
#include "MogsJoystick.h"
#endif
#include "MogsKinematics.h"
#include "MogsAbstractSothConstraint.h"


enum mode{
	DESIRED,	// set of desired position
	JOYSTICK,	// the desired position is given by a joystick
	BODY		// the desired position is one of an other robot body
};

SpatialTransform<double> convert( SpatialTransform<F<double> > &  T)
{
    SpatialTransform<double> out;
    for (int i=0;i<3;i++)
    {
        out.r(i) = T.r(i).x();
        for (int j=0;j<3;j++)
            out.E(i,j) = T.E(i,j).x();
    }
    return out;
}

Eigen::Matrix <double,3, 1 > convert( Eigen::Matrix < F<double>,3, 1 >& T)
{
    Eigen::Matrix <double,3, 1 > out;
    for (int i=0;i<3;i++)   out(i) = T(i).x();
    return out;
}

class MogsEndEffectorConstraint: public MogsAbstractSothConstraint
{
      public:

	MogsEndEffectorConstraint();

	MogsEndEffectorConstraint(	const std::string & in,
					int body_id,
					const Eigen::Matrix<double,3,1> position,
					const Eigen::Matrix<double,3,1> initial_desired_position,
                                        unsigned int ref = 0,
                                        double threshold = 1.5e-2);

	MogsEndEffectorConstraint(	int body_id,
					int target_robot_id,
					int target_body_id,
					const Eigen::Matrix<double,3,1> position,
                                        unsigned int ref = 0,
                                        double threshold = 1.5e-2);

	MogsEndEffectorConstraint(  int body_id,
                                    const Eigen::Matrix<double,3,1> position,
                                    const Eigen::Matrix<double,3,1> target,
                                    unsigned int ref = 0,
                                    double threshold = 1.5e-2);

	MogsEndEffectorConstraint(  QDomElement pg_root,
                                    RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                    std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots = NULL);

	~MogsEndEffectorConstraint();

	virtual void compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
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
		return 3;
	}

	void set_desired_position(const Eigen::Matrix < double,3, 1 >& in )
	{
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

protected:

	Eigen::Matrix < double,3, 1 > desired_position_;
        std::vector< Eigen::Matrix < double,3, 1 > > positions_;
        double distance_;
        
private:
	Eigen::Matrix < double,3, 1 > effector_position_;
        
        Eigen::Matrix<double,3,1> current_effector_position_;
	unsigned int body_id_;
        unsigned int body_ref_;

	int count_position_;
	Eigen::Matrix < F<double>,3, 1 > Ftmp_, Ftmp2_;
        SpatialTransform<F<double> > ref_trans_,ref_trans2_;

        #ifdef MogsJoystick_FOUND
	// used in mode JOYSTICK
	MogsJoystick * pad_;
        #endif 

	// use in mode body
	int target_robot_id_;
	int target_body_id_;

	mode desired_mode_;

        double threshold_;
        
        unsigned int nb_compute_without_modif_;
};


#endif
