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
                                                        MogsEndEffectorConstraint(pg_root,kin,all_the_robots),ROSConstraint(pg_root) ,n("~")

{
    positions_.resize(1);
    sub = n.subscribe(topic_name_ + "/Point", 10, &ROSMogsEndEffectorConstraint::callback,this);
    pub = n.advertise<std_msgs::Float64> ( topic_name_ + "/Error", 1);
    pub_desired = n.advertise<geometry_msgs::Point> ( topic_name_ + "/Desired", 1);        
}

ROSMogsEndEffectorConstraint::~ROSMogsEndEffectorConstraint()
{

}

void ROSMogsEndEffectorConstraint::compute( Eigen::Matrix <F<double>,Eigen::Dynamic, 1 > &Q,  
                                            RigidBodyDynamics::MogsKinematics<F<double> > * kin,
                                            HQPSolver::Priority* task_info,
                                            std::vector< RigidBodyDynamics::MogsKinematics< double > *>*all_the_robots)
{
    MogsEndEffectorConstraint::compute(Q,kin,task_info,all_the_robots);
    //publish the error
    output_msg_.data = distance_;
    pub.publish(output_msg_);

    geometry_msgs::Point msg_p;
    msg_p.x = desired_position_(0);
    msg_p.y = desired_position_(1);
    msg_p.z = desired_position_(2);
    pub_desired.publish(msg_p);
}

void ROSMogsEndEffectorConstraint::callback( const geometry_msgs::Point & msg)
{
    ROS_INFO("receive new pose");   
    desired_position_(0) = msg.x;
    desired_position_(1) = msg.y;
    desired_position_(2) = msg.z;

    positions_[0] = desired_position_;    
}

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
