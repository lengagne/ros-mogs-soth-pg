//      MogsAbstractCollision.h
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

#ifndef __MOGSABSTRACTCOLLISION__
#define __MOGSABSTRACTCOLLISION__

#include "MogsDynamics.h"

typedef struct{
	unsigned int robot_1;
	unsigned int body_1;
	unsigned int robot_2;
	unsigned int body_2;
	Eigen::Matrix<double,3,1> point_1;	// point of the distance on the surface
	Eigen::Matrix<double,3,1> point_2;	// point of the distance on the surface
// 	Eigen::Matrix<double,3,1> point_world;	// point of contact in the world frame
// 	Eigen::Matrix<double,3,1> normal;	// normal at the contact point

	unsigned int nb_points;	// number of contact points (in a mesh)
	std::vector<Eigen::Matrix<double,3,1> > points_1;
	std::vector<Eigen::Matrix<double,3,1> > points_2;
	std::vector<Eigen::Matrix<double,3,1> > points_world;
	std::vector<Eigen::Matrix<double,3,1> > normals;

	double distance;
}collision_value;

class MogsAbstractCollision
{
      public:

	bool compute_collision( std::vector<RigidBodyDynamics::MogsDynamics<double>* > robots,
				std::vector<collision_value> & collisions_);


	      // return true in case of collision
	      // and fill the collision_values
	virtual bool compute_collision( std::vector<RigidBodyDynamics::MogsKinematics<double>* > robots,
					std::vector<collision_value> & collisions_) = 0;

		// the result of all the distance is stored in collision_values_
	virtual void compute_distance( std::vector<RigidBodyDynamics::MogsKinematics<double>* > robots,
				       std::vector<collision_value> & collisions_,
					bool print_min_distance = false) = 0;

	virtual void init( std::vector<RigidBodyDynamics::MogsKinematics<double>* > robots,bool self_collision = false) = 0;

	virtual void init_collision_robot( RigidBodyDynamics::MogsRobotProperties * robot) = 0;

      protected:
	std::vector<collision_value> collision_values_;

};

// the types of the class factories
typedef MogsAbstractCollision* create_collision(QDomElement& ElementNode);
typedef void destroy_collision(MogsAbstractCollision*);

#endif
