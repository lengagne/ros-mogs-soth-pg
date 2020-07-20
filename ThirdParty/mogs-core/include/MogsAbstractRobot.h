//      MogsAbstractRobot.h
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

#ifndef __MOGSABSTRACTROBOT__
#define __MOGSABSTRACTROBOT__

// Library needed to type conversions
#include "MogsTypes.h"

class MogsAbstractRobot
{
      public:
// 	MogsAbstractRobot();

// 	~MogsAbstractRobot();

	virtual void compute_kinematics(	const Eigen::Matrix <double,Eigen::Dynamic, 1 > *Q,
						const Eigen::Matrix <double,Eigen::Dynamic, 1 > *QDot = NULL,
						const Eigen::Matrix <double,Eigen::Dynamic, 1 > *QDDot = NULL) = 0;

	virtual void ForwardDynamics (	const Eigen::Matrix <double, Eigen::Dynamic, 1 > &Q,
					const Eigen::Matrix <double, Eigen::Dynamic, 1 > &QDot,
					const Eigen::Matrix <double, Eigen::Dynamic, 1 > &Tau,
					Eigen::Matrix <double, Eigen::Dynamic, 1 > &QDDot,
					std::vector < Eigen::Matrix <double, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix <double, 6, 1 > > > *f_ext = NULL) = 0;


	virtual int get_nb_bodies() const =0;

	virtual int get_nb_joint() const =0;

	mogs_string get_type() const
	{
		return type_;
	}

	virtual void InverseDynamics (	const Eigen::Matrix <double, Eigen::Dynamic, 1 > &Q,
					const Eigen::Matrix <double, Eigen::Dynamic, 1 > &QDot,
					const Eigen::Matrix <double, Eigen::Dynamic, 1 > &QDDot,
					Eigen::Matrix <double, Eigen::Dynamic, 1 > &Tau,
					std::vector < Eigen::Matrix <double, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix <double, 6, 1 > > >  *f_ext = NULL) = 0;

	bool is_ready() const
	{
		return robot_loaded_ok_;
	}

	void set_name( const mogs_string& name)
	{
		name_ = name;
	}

      protected:

	mogs_string name_;	// in the project the names of the robot must be different
	mogs_string type_;	// in the project one can have several robot with the same type (but not the same name)

	bool robot_loaded_ok_;



};

// the types of the class factories
typedef MogsAbstractRobot* create_robot();
typedef void destroy_robot(MogsAbstractRobot*);

#endif
