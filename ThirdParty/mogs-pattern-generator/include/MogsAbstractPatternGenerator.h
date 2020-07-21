//      MogsAbstractPatternGenerator.h
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
//      2009-2011:  Joint robotics Laboratory - CNRS/AIST,Tsukuba, Japan.
//      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
//      2012-2013:  IUT de Beziers/ LIRMM, Beziers, France
//	from 2013:  Université Blaise Pascal / axis : ISPR / theme MACCS

#ifndef __MOGSABSTRACTPATTERNGENERATOR__
#define __MOGSABSTRACTPATTERNGENERATOR__

// Library needed to type conversions
#include "MogsKinematics.h"

/**	This class reads the configuration files,
*	prepare, solve the problem and store the result.
*/
class MogsAbstractPatternGenerator
{
      public:

	/** This function get the current time,
	 *	the current values of joint position, velocity, acceleration and torques
	 * 	and compute the needed values
	 * 	The function returns false when the pattern is ended.
	 */
	virtual bool compute(	double time,
				Eigen::Matrix <double,Eigen::Dynamic, 1 > *Q,
				std::vector< RigidBodyDynamics::MogsKinematics<double> *> * kins = NULL,
				Eigen::Matrix <double,Eigen::Dynamic, 1 > *DQ = NULL,
				Eigen::Matrix <double,Eigen::Dynamic, 1 > *DDQ = NULL,
				Eigen::Matrix <double,Eigen::Dynamic, 1 > *torques = NULL) = 0;

	unsigned int get_robot_id(	const mogs_string & robot_name,
					std::vector< RigidBodyDynamics::MogsKinematics<double> *> * all_robots) const;


	virtual void set_robot(RigidBodyDynamics::MogsRobotProperties * in);

	virtual void read_xml(	QDomElement PG_root,
                            std::vector< RigidBodyDynamics::MogsKinematics<double> *> * kins = NULL	);

      protected:
	QDomElement root_;

	RigidBodyDynamics::MogsRobotProperties * robot_;
	int nb_dof_;
};

// the types of the class factories
typedef MogsAbstractPatternGenerator* create_pattern_generator();
typedef void destroy_pattern_generator(MogsAbstractPatternGenerator*);

#endif
