//      MogsAbstractSothConstraint.h
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

#ifndef __MOGSAbstractSothConstraint__
#define __MOGSAbstractSothConstraint__

#include "MogsKinematics.h"
#include "HQPSolver.h"
#include "fadiff.h"

/**	This class reads the configuration files,
*	prepare, solve the problem and store the result.
*/
class MogsAbstractSothConstraint
{
      public:

	virtual void compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
				RigidBodyDynamics::MogsKinematics<F<double> > * kin,
				HQPSolver::Priority* task_info,
				std::vector< RigidBodyDynamics::MogsKinematics< double > * >*all_the_robots = NULL) = 0;

	virtual int get_inequalities(int count) const = 0; // 0: equality,  1: lower inequality, 2: upper inequality,

	virtual double get_max(int count) const = 0;

	virtual int get_task_size() const = 0 ;


      protected:

	private:
};


// the types of the class factories
typedef MogsAbstractSothConstraint* create_soth_constraint( QDomElement pg_root,
                                                            RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                            std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots);

typedef void destroy_soth_constraint(MogsAbstractSothConstraint*);


#endif
