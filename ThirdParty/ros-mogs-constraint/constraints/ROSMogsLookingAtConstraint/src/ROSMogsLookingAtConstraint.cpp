//      ROSMogsLookingAtConstraint.cpp
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

#include "ROSMogsLookingAtConstraint.h"

// void appel( const geometry_msgs::Pose & msg)
// {
//     
// }


ROSMogsLookingAtConstraint::ROSMogsLookingAtConstraint(	QDomElement pg_root,
                                                        RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                        std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots ):
                                                        MogsLookingAtConstraint(pg_root,kin,all_the_robots),ROSConstraint(pg_root) ,n("~")

{
    desired_positions_.resize(1);
    sub = n.subscribe(topic_name_ + "/Pose", 5, &ROSMogsLookingAtConstraint::callback,this);
    pub = n.advertise<std_msgs::Float64> ( topic_name_ + "/Error", 500);    
    
    std::cout<<"Constructor of ROSMogsLookingAtConstraint ends"<<std::endl;
}

ROSMogsLookingAtConstraint::~ROSMogsLookingAtConstraint()
{

}

void ROSMogsLookingAtConstraint::compute( Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,  
                                            RigidBodyDynamics::MogsKinematics<F<double> > * kin,
                                            HQPSolver::Priority* task_info,
                                            std::vector< RigidBodyDynamics::MogsKinematics< double > *>*all_the_robots)
{
    MogsLookingAtConstraint::compute(Q,kin,task_info,all_the_robots);
    //publish the error
    output_msg_.data = distance_;
    pub.publish(output_msg_);
    
}

void ROSMogsLookingAtConstraint::callback( const geometry_msgs::Point & msg)
{
    ROS_INFO("receive new point");   
    desired_position_(0) = msg.x;
    desired_position_(1) = msg.y;
    desired_position_(2) = msg.z;
    
}

extern "C" ROSMogsLookingAtConstraint* create(   QDomElement pg_root,
                                                RigidBodyDynamics::MogsKinematics<F<double> >* kin,
                                                std::vector< RigidBodyDynamics::MogsKinematics< double > * >* all_the_robots)
{
    return new ROSMogsLookingAtConstraint(pg_root, kin, all_the_robots);
}

extern "C" void destroy(ROSMogsLookingAtConstraint* p)
{
    delete p;
}
