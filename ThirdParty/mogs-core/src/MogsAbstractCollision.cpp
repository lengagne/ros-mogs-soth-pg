//      MogsAbstractCollision.cpp
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

#include "MogsAbstractCollision.h"

bool MogsAbstractCollision::compute_collision( 	std::vector<RigidBodyDynamics::MogsDynamics<double>* > robots,
						std::vector<collision_value> & collisions_) 
{
	int nb = robots.size();
	std::vector<RigidBodyDynamics::MogsKinematics<double>* > tmp(nb);
	for (int i=0;i<nb;i++)
	{
		tmp[i] = (RigidBodyDynamics::MogsKinematics<double>*)(robots[i]);
	}
	return compute_collision(tmp,collisions_);
}
