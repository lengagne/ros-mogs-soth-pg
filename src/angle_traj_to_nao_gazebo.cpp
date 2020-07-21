#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <trajectory_msgs/JointTrajectory.h>  //Gazebo
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>  //Pepper

trajectory_msgs::JointTrajectory trajPepper; //Pepper
trajectory_msgs::JointTrajectory trajLeftArm;//Gazebo
trajectory_msgs::JointTrajectory trajLeftHand;
trajectory_msgs::JointTrajectory trajRightArm;
trajectory_msgs::JointTrajectory trajRightHand;
trajectory_msgs::JointTrajectory trajLeftLeg;
trajectory_msgs::JointTrajectory trajRightLeg;
trajectory_msgs::JointTrajectory trajLeftFoot;
trajectory_msgs::JointTrajectory trajRightFoot;
trajectory_msgs::JointTrajectory trajPelvis;
trajectory_msgs::JointTrajectory trajHead;
ros::Publisher pubLeftArm; 
ros::Publisher pubLeftHand;
ros::Publisher pubRightArm;
ros::Publisher pubRightHand;
ros::Publisher pubRightFoot;
ros::Publisher pubLeftFoot;
ros::Publisher pubRightLeg;
ros::Publisher pubLeftLeg;
ros::Publisher pubPelvis;
ros::Publisher pubHead;

unsigned int size=0;
int i=0;

void callback(const trajectory_msgs::JointTrajectory&  trajPepper)
{	
// 	ROS_INFO("call the callback");
	size = trajPepper.joint_names.size();     // taille du vecteur
	trajLeftArm.joint_names.resize(5); //initialisation de la taille des vecteurs
	trajLeftArm.points.resize(1);
	trajLeftHand.joint_names.resize(1); //initialisation de la taille des vecteurs
	trajLeftHand.points.resize(1);
	trajRightArm.joint_names.resize(5); //initialisation de la taille des vecteurs
	trajRightArm.points.resize(1);
	trajRightHand.joint_names.resize(1); //initialisation de la taille des vecteurs
	trajRightHand.points.resize(1);
        
	trajLeftFoot.joint_names.resize(2); //initialisation de la taille des vecteurs
	trajLeftFoot.points.resize(1);
	trajRightFoot.joint_names.resize(2); //initialisation de la taille des vecteurs
	trajRightFoot.points.resize(1);
	trajLeftLeg.joint_names.resize(3); //initialisation de la taille des vecteurs
	trajLeftLeg.points.resize(1);
	trajRightLeg.joint_names.resize(3); //initialisation de la taille des vecteurs
	trajRightLeg.points.resize(1);        
	trajPelvis.joint_names.resize(1); //initialisation de la taille des vecteurs
	trajPelvis.points.resize(1);
	trajHead.joint_names.resize(2); //initialisation de la taille des vecteurs
	trajHead.points.resize(1);
        
        
	//commande pour donner la taille du vecteur sur le terminal
// 	std::cout<<"trajLeftArm.points.size() = "<< trajLeftArm.points.size()<<std::endl;   
// 	std::cout<<"trajLeftArm.joint_names.size() = "<< trajLeftArm.joint_names.size()<<std::endl;
	trajLeftArm.points[0].positions.resize(5); 
	trajLeftHand.points[0].positions.resize(1); 
	trajRightArm.points[0].positions.resize(5); 
	trajRightHand.points[0].positions.resize(1);      
        trajLeftFoot.points[0].positions.resize(2); 
        trajRightFoot.points[0].positions.resize(2); 
        trajLeftLeg.points[0].positions.resize(3); 
        trajRightLeg.points[0].positions.resize(3);         
        trajPelvis.points[0].positions.resize(1); 
        trajHead.points[0].positions.resize(2); 

//         [, , , , , , ,
//   , LHipYawPitch, , , , , RAnklePitch,
//   RAnkleRoll, , , , RHipPitch, RHipRoll, RHipYawPitch, RKneePitch,
//   , , ]
        
  
// 	std::cout<<"trajLeftArm.points[0].positions.size() = "<< trajLeftArm.points[0].positions.size()<<std::endl;
// 	std::cout<<"trajPepper.joint_angles.size() = "<< trajPepper.joint_angles.size()<<std::endl;
// 	std::cout<<"trajLeftArm.points[0].velocities.size() = "<< trajLeftArm.points[0].velocities.size()<<std::endl;

	
	trajLeftArm.joint_names[0]="LShoulderPitch";
	trajLeftArm.joint_names[1]="LShoulderRoll";
	trajLeftArm.joint_names[2]="LElbowYaw";
	trajLeftArm.joint_names[3]="LElbowRoll";
	trajLeftArm.joint_names[4]="LWristYaw";
	
	trajRightArm.joint_names[0]="RShoulderPitch";
	trajRightArm.joint_names[1]="RShoulderRoll";
	trajRightArm.joint_names[2]="RElbowYaw";
	trajRightArm.joint_names[3]="RElbowRoll";
	trajRightArm.joint_names[4]="RWristYaw";	
	
	trajLeftHand.joint_names[0]="LHand";
	trajRightHand.joint_names[0]="RHand";
        
        trajLeftLeg.joint_names[0] = "LHipRoll";
        trajLeftLeg.joint_names[1] = "LHipPitch";
        trajLeftLeg.joint_names[2] = "LKneePitch";
        
        trajLeftFoot.joint_names[0] = "LAnklePitch";
        trajLeftFoot.joint_names[1] = "LAnkleRoll";
	
        trajRightLeg.joint_names[0] = "RHipRoll";
        trajRightLeg.joint_names[1] = "RHipPitch";
        trajRightLeg.joint_names[2] = "RKneePitch";
        
        trajRightFoot.joint_names[0] = "RAnklePitch";
        trajRightFoot.joint_names[1] = "RAnkleRoll";
        
        trajPelvis.joint_names[0] = "LHipYawPitch";
        
        trajHead.joint_names[0] = "HeadYaw";
        trajHead.joint_names[1] = "HeadPitch";        
        
        for (i=0;i<size;i++)
	{
						//BRAS GAUCHE
		if(trajPepper.joint_names[i] == "LShoulderPitch"){
			  // les noms des articulations 
		        trajLeftArm.points[0].positions[0] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
		else if (trajPepper.joint_names[i] == "LShoulderRoll"){
		        trajLeftArm.points[0].positions[1] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
		else if (trajPepper.joint_names[i] == "LElbowYaw"){
		        trajLeftArm.points[0].positions[2] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
		else if (trajPepper.joint_names[i] == "LElbowRoll"){
		        trajLeftArm.points[0].positions[3] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
		else if (trajPepper.joint_names[i] == "LWristYaw"){
		        trajLeftArm.points[0].positions[4] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
						//BRAS DROIT
		else if (trajPepper.joint_names[i] == "RShoulderPitch"){  
		        trajRightArm.points[0].positions[0] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
		else if (trajPepper.joint_names[i] == "RShoulderRoll"){
		        trajRightArm.points[0].positions[1] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
		else if (trajPepper.joint_names[i] == "RElbowYaw"){
		        trajRightArm.points[0].positions[2] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
		else if (trajPepper.joint_names[i] == "RElbowRoll"){
		      	trajRightArm.points[0].positions[3] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
		else if (trajPepper.joint_names[i] == "RWristYaw"){
		        trajRightArm.points[0].positions[4] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
						//MAIN GAUCHE
		else if (trajPepper.joint_names[i] == "LHand"){
			  // les noms des articulations 
		        trajLeftHand.points[0].positions[0] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
						//MAIN DROITE
		else if (trajPepper.joint_names[i] == "RHand"){
		        trajRightHand.points[0].positions[0] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
						//JAMBE GAUCHE 
		else if (trajPepper.joint_names[i] == "LHipRoll"){
		        trajLeftLeg.points[0].positions[0] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}		
		else if (trajPepper.joint_names[i] == "LHipPitch"){
		        trajLeftLeg.points[0].positions[1] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
		else if (trajPepper.joint_names[i] == "LKneePitch"){
		        trajLeftLeg.points[0].positions[2] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
                                                // PIED GAUCHE
		else if (trajPepper.joint_names[i] == "LAnklePitch"){
		        trajLeftFoot.points[0].positions[0] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
		else if (trajPepper.joint_names[i] == "LAnkleRoll"){
		        trajLeftFoot.points[0].positions[1] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
						//JAMBE DROITE
		else if (trajPepper.joint_names[i] == "RHipRoll"){
		        trajRightLeg.points[0].positions[0] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}		
		else if (trajPepper.joint_names[i] == "RHipPitch"){
		        trajRightLeg.points[0].positions[1] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
		else if (trajPepper.joint_names[i] == "RKneePitch"){
		        trajRightLeg.points[0].positions[2] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
                                                // PIED DROIT
		else if (trajPepper.joint_names[i] == "RAnklePitch"){
		        trajRightFoot.points[0].positions[0] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
		else if (trajPepper.joint_names[i] == "RAnkleRoll"){
		        trajRightFoot.points[0].positions[1] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}
                    // PELVIS
		else if (trajPepper.joint_names[i] == "LHipYawPitch"){
		        trajPelvis.points[0].positions[0] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}                    
                    // HEAD
		else if (trajPepper.joint_names[i] == "HeadYaw"){
		        trajHead.points[0].positions[0] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}          
		else if (trajPepper.joint_names[i] == "HeadPitch"){
		        trajHead.points[0].positions[1] = trajPepper.points[i].positions[0];  // valeurs articulaires
		}          
	}

// 	ROS_INFO("fin");

						//init
	trajLeftArm.points[0].velocities.resize(5);
	trajLeftArm.points[0].velocities[0] = 0.0;
	trajLeftArm.points[0].time_from_start = ros::Duration(1.0);
	trajLeftArm.header.stamp = ros::Time::now() + ros::Duration();

	trajRightArm.points[0].velocities.resize(5);
	trajRightArm.points[0].velocities[0] = 0.0;
	trajRightArm.points[0].time_from_start = ros::Duration(1.0);
	trajRightArm.header.stamp = ros::Time::now() + ros::Duration();

	trajLeftHand.points[0].velocities.resize(1);
	trajLeftHand.points[0].velocities[0] = 0.0;
	trajLeftHand.points[0].time_from_start = ros::Duration(1.0);
	trajLeftHand.header.stamp = ros::Time::now() + ros::Duration();

	trajRightHand.points[0].velocities.resize(1);
	trajRightHand.points[0].velocities[0] = 0.0;
	trajRightHand.points[0].time_from_start = ros::Duration(1.0);
	trajRightHand.header.stamp = ros::Time::now() + ros::Duration();
        
	trajLeftFoot.points[0].velocities.resize(2);
	trajLeftFoot.points[0].velocities[0] = 0.0;
	trajLeftFoot.points[0].time_from_start = ros::Duration(1.0);
	trajLeftFoot.header.stamp = ros::Time::now() + ros::Duration();

	trajRightFoot.points[0].velocities.resize(2);
	trajRightFoot.points[0].velocities[0] = 0.0;
	trajRightFoot.points[0].time_from_start = ros::Duration(1.0);
	trajRightFoot.header.stamp = ros::Time::now() + ros::Duration();
        
	trajLeftLeg.points[0].velocities.resize(3);
	trajLeftLeg.points[0].velocities[0] = 0.0;
	trajLeftLeg.points[0].time_from_start = ros::Duration(1.0);
	trajLeftLeg.header.stamp = ros::Time::now() + ros::Duration();

	trajRightLeg.points[0].velocities.resize(3);
	trajRightLeg.points[0].velocities[0] = 0.0;
	trajRightLeg.points[0].time_from_start = ros::Duration(1.0);
	trajRightLeg.header.stamp = ros::Time::now() + ros::Duration();
        
	trajPelvis.points[0].velocities.resize(1);
	trajPelvis.points[0].velocities[0] = 0.0;
	trajPelvis.points[0].time_from_start = ros::Duration(1.0);
	trajPelvis.header.stamp = ros::Time::now() + ros::Duration();
        
	trajHead.points[0].velocities.resize(2);
	trajHead.points[0].velocities[0] = 0.0;
	trajHead.points[0].time_from_start = ros::Duration(1.0);
	trajHead.header.stamp = ros::Time::now() + ros::Duration();  

	// pub.publish(trajGazebo);  
	ROS_INFO("publish traj");

}

int main(int argc, char **argv)
{
        ROS_INFO("Start the node");
	ros::init(argc, argv, "angle_traj_to_nao_gazebo");
	ros::NodeHandle nh;
	pubLeftArm = nh.advertise<trajectory_msgs::JointTrajectory>("/nao_dcm/LeftArm_controller/command", 1000);
	pubLeftHand = nh.advertise<trajectory_msgs::JointTrajectory>("/nao_dcm/LeftHand_controller/command", 1000);

	pubRightArm = nh.advertise<trajectory_msgs::JointTrajectory>("/nao_dcm/RightArm_controller/command", 1000);
	pubRightHand= nh.advertise<trajectory_msgs::JointTrajectory>("/nao_dcm/RightHand_controller/command", 1000);
        
	pubRightArm = nh.advertise<trajectory_msgs::JointTrajectory>("/nao_dcm/RightArm_controller/command", 1000);
	pubRightHand= nh.advertise<trajectory_msgs::JointTrajectory>("/nao_dcm/RightHand_controller/command", 1000);      
        
        pubRightFoot = nh.advertise<trajectory_msgs::JointTrajectory>("/nao_dcm/RightFoot_controller/command", 1000);
        pubLeftFoot = nh.advertise<trajectory_msgs::JointTrajectory>("/nao_dcm/LeftFoot_controller/command", 1000);
        pubRightLeg = nh.advertise<trajectory_msgs::JointTrajectory>("/nao_dcm/RightLeg_controller/command", 1000);
        pubLeftLeg = nh.advertise<trajectory_msgs::JointTrajectory>("/nao_dcm/LeftLeg_controller/command", 1000);        
        pubPelvis = nh.advertise<trajectory_msgs::JointTrajectory>("/nao_dcm/Pelvis_controller/command", 1000);
        pubHead = nh.advertise<trajectory_msgs::JointTrajectory>("/nao_dcm/Head_controller/command", 1000);
        

	ros::Subscriber sub = nh.subscribe("/joint_angles", 10, callback);
//	ros::spin();

	ros::Rate rate(10);
	//rate.sleep();

	while (ros::ok())
	{
                //envoie des valeurs articulaire sur Gazebo
		pubLeftArm.publish(trajLeftArm);
		pubRightArm.publish(trajRightArm);
		pubLeftHand.publish(trajLeftHand);
		pubRightHand.publish(trajRightHand);
                pubRightFoot.publish(trajRightFoot);
                pubLeftFoot.publish(trajLeftFoot);
                pubRightLeg.publish(trajRightLeg);
                pubLeftLeg.publish(trajLeftLeg);                
                pubPelvis.publish(trajPelvis);
                pubHead.publish(trajHead);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

