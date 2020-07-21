//      MogsSothPatternGenerator.h
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

#ifndef __MOGSSothPATTERNGENERATOR__
#define __MOGSSothPATTERNGENERATOR__

#include "MogsAbstractPatternGenerator.h"
#include "MogsKinematics.h"
#include "HQPSolver.h"
#include "MogsAbstractSothConstraint.h"

/**	This class reads the configuration files,
*	prepare, solve the problem and store the result.
*/
class MogsSothPatternGenerator: public MogsAbstractPatternGenerator
{
      public:
	MogsSothPatternGenerator();

	MogsSothPatternGenerator(
                                    RigidBodyDynamics::MogsRobotProperties * this_robot,
                                    std::vector< MogsAbstractSothConstraint * > constraints,
                                    double coeff = 1.0
                                );

	~MogsSothPatternGenerator();

	bool compute(	double time,
			Eigen::Matrix < double,Eigen::Dynamic, 1 > *Q,
			std::vector< RigidBodyDynamics::MogsKinematics<double> *> * kin = NULL,
			Eigen::Matrix < double,Eigen::Dynamic, 1 > *DQ = NULL,
			Eigen::Matrix < double,Eigen::Dynamic, 1 > *DDQ = NULL,
			Eigen::Matrix < double,Eigen::Dynamic, 1 > *torques = NULL);
        
        void read_constraint_xml( const mogs_string & filename);

	void read_xml(	QDomElement PG_root,
			std::vector< RigidBodyDynamics::MogsKinematics<double> *> * kins = NULL);

	void set_robot(RigidBodyDynamics::MogsRobotProperties * in);
        
        void set_init_time(double t)
        {
            t_start_ = t;
        }

      protected:

	// iniatilize the soth solver
	void initialize_solver();

	// read the constraint from the xml
	void read_root_xml(std::vector< RigidBodyDynamics::MogsKinematics<double> *> * all_robots = NULL);



protected:

	HQPSolver* solver_;

	Eigen::Matrix<F<double>,Eigen::Dynamic,1> q_;
	Eigen::Matrix<double,Eigen::Dynamic,1> dq_;

	std::vector<double> dq_max_;
	
	std::vector<double> q_min_, q_max_;

	double t_start_;

	RigidBodyDynamics::MogsKinematics<F<double> > * kin_;
	std::vector< MogsAbstractSothConstraint * > constraints_;
	int nb_tasks_;

	double coeff_;
        
        bool test_derivative_;
};


#endif
