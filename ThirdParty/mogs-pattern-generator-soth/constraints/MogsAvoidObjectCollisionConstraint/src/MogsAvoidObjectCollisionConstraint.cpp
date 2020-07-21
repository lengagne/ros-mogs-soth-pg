//      MogsAvoidObjectCollisionConstraint.cpp
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

#include "MogsAvoidObjectCollisionConstraint.h"

MogsAvoidObjectCollisionConstraint::MogsAvoidObjectCollisionConstraint()
{

}

MogsAvoidObjectCollisionConstraint::MogsAvoidObjectCollisionConstraint(	QDomElement pg_root,
                                                                        RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                                        std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
	QDomElement Elname = pg_root.firstChildElement("name");
	if (Elname.isNull())
	{
		std::cerr<<"Error : you must defined a name in the constraint MogsAvoidObjectCollisionConstraint"<<std::endl;
		exit(0);
	}
	mogs_string body_name = Elname.text().simplified();
        init(body_name,kin,all_the_robots);
}

void MogsAvoidObjectCollisionConstraint::init(   QString & robot_name,
                                                    RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                    std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
	num_robot_ = -1;
	for (int i=0;i<all_the_robots->size();i++)
	{
		if ( robot_name == (*all_the_robots)[i]->getRobotName())
		{
			num_robot_ = i;
			break;
		}
	}
	if (num_robot_ == -1)
	{
		std::cerr<<"Error in "<< __FILE__<<" at line "<< __LINE__<<std::endl;
		std::cerr<<"Error the robot "<<robot_name.toStdString()<<" is not defined"<<std::endl;
		exit(0);
	}

	// set the collisions
	int nb_body_1 = kin->getNBodies();
	int nb_body_2 = (*all_the_robots)[num_robot_]->getNBodies();
	for (int i=0;i<nb_body_1;i++)
	{
		if(kin->get_geometry(i))
		{
			for (int j=0;j<nb_body_2;j++)	if ( (*all_the_robots)[num_robot_]->get_geometry(j))
			{
				collision tmp;
				tmp.body_id_1 = i;
				tmp.body_id_2 = j;
				collisions_.push_back(tmp);
			}
		}
	}
	nb_collisions_ = collisions_.size();
// 	std::cout<<"nb_collisions_ = "<< nb_collisions_ <<std::endl;
}


MogsAvoidObjectCollisionConstraint::~MogsAvoidObjectCollisionConstraint()
{

}

void MogsAvoidObjectCollisionConstraint::compute(	Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,
							RigidBodyDynamics::MogsKinematics<F<double> > * kin,
							HQPSolver::Priority* task_info,
							std::vector< RigidBodyDynamics::MogsKinematics< double >* > * all_the_robots)
{
	Eigen::Matrix < F<double>,3, 1 > Rpos; // position of the body of the robot
	Eigen::Matrix < double,3, 1 > Opos;	// position of the other robot to avoid collision
	F<double> distance;
	for (int i=0;i<nb_collisions_;i++)
	{
		Rpos = kin->getBodyToBaseCoordinates( collisions_[i].body_id_1);
		Opos = (*all_the_robots)[num_robot_]->getBodyToBaseCoordinates( collisions_[i].body_id_2);
		distance = ( Rpos - Opos.cast<F<double>>()).norm();
		for (int l=0;l<kin->getNDof();l++)
			task_info->Jacobian(i,l) = distance.d(l);
		task_info->error(i) = MogsAvoidObjectCollisionConstraint_tolerance - distance.x();

		std::cout<<" collisions_[i].body_id_1 = "<<  collisions_[i].body_id_1 <<std::endl;
		std::cout<<" collisions_[i].body_id_2 = "<<  collisions_[i].body_id_2 <<std::endl;
		std::cout<<" distance = "<< distance<<std::endl;
	}

// 	std::cout<<"task_info->error = "<< task_info->error.transpose()<<std::endl;
}

extern "C" MogsAvoidObjectCollisionConstraint* create(  QDomElement pg_root,
                                                        RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                        std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new MogsAvoidObjectCollisionConstraint(pg_root, kin, all_the_robots);
}

extern "C" void destroy(MogsAvoidObjectCollisionConstraint* p)
{
    delete p;
}
