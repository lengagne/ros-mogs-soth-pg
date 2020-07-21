#include <fstream>  
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "MogsSothPatternGenerator.h"

unsigned int nb_dof;
Eigen::Matrix <double,Eigen::Dynamic, 1 > q;
std::map<std::string,unsigned int> joint_id;
std::vector<std::string> joint_names;

MogsSothPatternGenerator *pg;
ros::Publisher pubJointAngle;

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
            std::cout<<"name = "<< msg.name[i] <<" q("<< it->second<<") = "<< msg.position[i]<<std::endl;
        }else
        {
            std::cout<<"Cannot find "<< msg.name[i] <<std::endl;
        }
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
    
    std::string prefix_topic;
    if (!nh.getParam("/mogs_soth_pg/prefix", prefix_topic))
        prefix_topic="";
       
    RigidBodyDynamics::MogsRobotProperties robot;
    if (robot.SetRobotFile(QString::fromStdString(robot_file)))
        ROS_INFO("Reading robot file ok");
    else
        ROS_ERROR("Cannot parse %s",robot_file.c_str());
    
    pg = new MogsSothPatternGenerator();
    pg->set_robot(&robot);
    
    // prepare the message
    nb_dof = robot.getNDof();
    q.resize(nb_dof);
    
    joint_names = robot.GetJointsName();
    for (unsigned int i=0;i<joint_names.size();i++)
    {
        joint_id[joint_names[i]] = i;
//         out_msg.joint_names.push_back(joint_names[i]);
        std::cout<<"joint_names["<<i<<"] = "<< joint_names[i] <<std::endl;
    }
//     out_msg.joint_angles.resize(nb_dof);    
    

    
    pubJointAngle = nh.advertise<std_msgs::Float64MultiArray>("/mogs_soth_pg/joint_angles", 1000); 	
    ros::Subscriber sub = nh.subscribe("/mogs_soth_pg/joint_states", 1000, UpdateJointValue);
    
    ros::spin();
    return 0;
};


