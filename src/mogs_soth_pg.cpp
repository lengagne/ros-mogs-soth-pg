#include <fstream>  
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "MogsSothPatternGenerator.h"

unsigned int nb_dof;
Eigen::Matrix <double,Eigen::Dynamic, 1 > q,dq;
std::map<std::string,unsigned int> joint_id;
std::vector<std::string> joint_names;

MogsSothPatternGenerator *pg;
ros::Publisher pubJointAngle;
std::vector< RigidBodyDynamics::MogsKinematics<double> *> robots_;

trajectory_msgs::JointTrajectory out_msg;

bool pg_is_ready = false;

void UpdateJointValue (const sensor_msgs::JointState& msg)
{
//     std::cout<<"COUT Starting PG"<<std::endl;
     ROS_INFO("Update Jointe Value PG");
    for(int i=0;i<msg.name.size();i++)
    {
        std::map<std::string,unsigned int>::const_iterator it = joint_id.find(msg.name[i]);
        if( it != joint_id.end())
        {
            q( it->second) = msg.position[i];
//             std::cout<<"name = "<< msg.name[i] <<" q("<< it->second<<") = "<< msg.position[i]<<std::endl;
        }else
        {
            std::cout<<"Cannot find "<< msg.name[i] <<std::endl;
        }
    }
    
    if (pg_is_ready)
    {
        if (!pg->compute(ros::Time::now().toSec(),&q,&robots_,&dq))
        {
            ROS_ERROR("Error in the pattern generator fails");
        }
        ROS_INFO("Preparing message from the PG");
        for (int i=0;i<joint_names.size();i++)
        {
            out_msg.points[i].positions[0] = q(i);
        }
        pubJointAngle.publish(out_msg);    
    }
}


int main(int argc, char** argv){
    // Dummy to use mogs 
    QApplication app(argc, argv);
    
    ros::init(argc, argv, "mogs_soth_pg");
    ros::NodeHandle nh;

    
    std::string robot_file;
    if (nh.getParam("/mogs_soth_pg/robot_urdf", robot_file))
        ROS_INFO("Got param: %s", robot_file.c_str());
    else
        ROS_ERROR("Failed to get param 'mogs_soth_pg/robot_urdf'");
    
//     std::string prefix_topic;
//     if (!nh.getParam("/mogs_soth_pg/prefix", prefix_topic))
//         prefix_topic="";
    
    std::string congif_pg;
    if (nh.getParam("/mogs_soth_pg/config", congif_pg))
        ROS_INFO("Got param: %s", congif_pg.c_str());
    else
        ROS_ERROR("Failed to get param 'mogs_soth_pg/config'");
       
    RigidBodyDynamics::MogsRobotProperties robot;
    if (robot.SetRobotFile(QString::fromStdString(robot_file)))
        ROS_INFO("Reading robot file ok");
    else
        ROS_ERROR("Cannot parse %s",robot_file.c_str());
    
    pg = new MogsSothPatternGenerator();
    pg->set_robot(&robot);
     ROS_INFO("Reading constraint xml");
    pg->read_constraint_xml(QString::fromStdString(congif_pg));
    pg->set_init_time(ros::Time::now().toSec());  
    
    robots_.push_back(new RigidBodyDynamics::MogsKinematics<double>(&robot));
    
    ROS_INFO("Preparing message");
    // prepare the message
    nb_dof = robot.getNDof();
    q.resize(nb_dof);
    dq.resize(nb_dof);
    
    joint_names = robot.GetJointsName();
    for (unsigned int i=0;i<joint_names.size();i++)
    {
        joint_id[joint_names[i]] = i;
        out_msg.joint_names.push_back(joint_names[i]);
        std::cout<<"joint_names["<<i<<"] = "<< joint_names[i] <<std::endl;
    }
    out_msg.points.resize(nb_dof); 
    for (int i=0;i<nb_dof;i++)
        out_msg.points[i].positions.resize(1);

    
    pubJointAngle = nh.advertise<trajectory_msgs::JointTrajectory>("/mogs_soth_pg/joint_angles", 1000); 	
    ros::Subscriber sub = nh.subscribe("/mogs_soth_pg/joint_states", 1000, UpdateJointValue);
    
    pg_is_ready = true;
    
    ros::spin();
    return 0;
};


