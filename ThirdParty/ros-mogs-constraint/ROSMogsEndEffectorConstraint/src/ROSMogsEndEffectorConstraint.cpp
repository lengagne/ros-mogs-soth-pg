//      ROSMogsEndEffectorConstraint.cpp
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
//	from 2020:  Université Clermont Auvergne / Pascale Institute / axis : ISPR / theme MACCS

#include "ROSMogsEndEffectorConstraint.h"

// void appel( const geometry_msgs::Pose & msg)
// {
//     
// }


ROSMogsEndEffectorConstraint::ROSMogsEndEffectorConstraint(	QDomElement pg_root,
                                                        RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                        std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots ):
                                                        MogsEndEffectorConstraint(pg_root,kin,all_the_robots) ,n("~")
{
    
    
//         sub = n.subscribe<geometry_msgs::Pose>("/MogsEndEffectorConstraint/Pose", 5, &ROSMogsEndEffectorConstraint::callback,this);
//     sub = n.subscribe<geometry_msgs::Pose>("/MogsEndEffectorConstraint/Pose", 5, appel);

        pub = n.advertise<std_msgs::Float64> ("/MogsEndEffectorConstraint/Error", 500);    
}

ROSMogsEndEffectorConstraint::~ROSMogsEndEffectorConstraint()
{

}

// void ROSMogsEndEffectorConstraint::callback( const geometry_msgs::Pose & msg)
// {
//     ROS_INFO("receive new pose");
//     
// }

extern "C" ROSMogsEndEffectorConstraint* create(   QDomElement pg_root,
                                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new ROSMogsEndEffectorConstraint(pg_root, kin, all_the_robots);
}

extern "C" void destroy(ROSMogsEndEffectorConstraint* p)
{
    delete p;
}
