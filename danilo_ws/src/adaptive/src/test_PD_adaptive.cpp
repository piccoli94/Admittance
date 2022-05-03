/* ========================= ICR_Simulation ================ */
/*         All right this code is reserved for the ICAROS
* Desrcption:
This is a controller based on a region control law

Author: Piccoli Danilo
Email: piccoli94@gmail.com
Date: 21/07/21

Revision History:
--------------------------------
Version V1.0: Initial Version

=============================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include "boost/thread.hpp"
#include "SlidingModeClient.cpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>
class IIWA_SIM 
{
	public:
		IIWA_SIM();
		void run();
		bool init_robot_model();
		void joint_states_cb( sensor_msgs::JointState );
		void ctrl_loop();

	private:
		std::string robot_desc_string;
		ros::NodeHandle _nh;
		KDL::Tree iiwa_tree;
		KDL::Chain _k_chain;
		std::string base_link;
		std::string tip_link;
		ros::Subscriber joint_sub;
		ros::Publisher torque_pub[7];
		std_msgs::Float64 cmd[7];
		REGION_CONTROLLER *controller;
		string tau_name="/home/danilo/catkin_ws/src/adaptive/src/Recording/tau.txt";
		std::ofstream file_tau;
		Eigen::Matrix<double,7,1> q;
		Eigen::Matrix<double,7,1> tau;;
		SlidingModeClient *trafoClient;
		KDL::ChainDynParam *_dyn_param;
		KDL::JntArray *_q_in; 
		bool first;
		Eigen::Matrix<double,7,1> q_initial;
		KDL::JntArray *_dq_in;
		Matrix<double ,7,1> dq_old;
		Matrix<double ,7,1> dq;
		Matrix<double ,7,1> ddq;
		double ts=0.005;
		Matrix<double,7,1> tau_m;
		KDL::ChainJntToJacSolver *_jsolver;
		KDL::Jacobian J;
		Matrix<double ,3,7> Jin;
	//-----------------------	

};

bool IIWA_SIM::init_robot_model() {
	_nh.param("robot_description", robot_desc_string, std::string());
	if(!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)) ROS_ERROR("Failed to construct kdl tree");
	base_link = "lbr_iiwa_link_0";
	tip_link  = "lbr_iiwa_link_7";
	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) ROS_ERROR("robot not init");
	trafoClient=new SlidingModeClient(1);
	trafoClient->onStateChange();
	_dyn_param = new KDL::ChainDynParam(_k_chain,KDL::Vector(0,0,-9.81));
	_q_in = new KDL::JntArray(_k_chain.getNrOfJoints()); //crea l'array relativo ai giunti
	_dq_in = new KDL::JntArray( _k_chain.getNrOfJoints() );
	
	_jsolver = new KDL::ChainJntToJacSolver(_k_chain);
	J.resize(_k_chain.getNrOfJoints());


	return true;
}


void IIWA_SIM::joint_states_cb( sensor_msgs::JointState js ) {

	for(int i=0; i<7; i++ ) { 
		q(i) = js.position[i];
		_q_in->data[i]=q(i);
		_dq_in->data[i] = js.velocity[i];
		q_initial(0)=0;
		q_initial(1)=0;
		q_initial(2)=0;
		q_initial(3)=-1.57;
		q_initial(4)=0;
		q_initial(5)=1.57;
		q_initial(6)=0;
		dq(i)=_dq_in->data[i];


	}
	ddq=(dq-dq_old)/ts;
	dq_old=dq;
	first=true;
}


IIWA_SIM::IIWA_SIM() 
{

	if (!init_robot_model()) exit(1); 
	ROS_INFO("Robot tree correctly loaded from parameter server!");
	cout << "Joints and segments: " << iiwa_tree.getNrOfJoints() << " - " << iiwa_tree.getNrOfSegments() << endl;
	joint_sub = _nh.subscribe("/lbr_iiwa/joint_states", 0, &IIWA_SIM::joint_states_cb, this);
	
	torque_pub[0] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_1_effort_controller/command", 0);
	torque_pub[1] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_2_effort_controller/command", 0);
	torque_pub[2] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_3_effort_controller/command", 0);
	torque_pub[3] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_4_effort_controller/command", 0);
	torque_pub[4] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_5_effort_controller/command", 0);
	torque_pub[5] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_6_effort_controller/command", 0);
	torque_pub[6] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_7_effort_controller/command", 0);
	first=false;
}

void IIWA_SIM::run() {


	boost::thread ctrl_loop_t ( &IIWA_SIM::ctrl_loop, this);
	ros::spin();	

}



void IIWA_SIM::ctrl_loop() 
{
	while( !first )
	{
		q=q_initial;
		ddq=ddq*0;
		usleep(0.1);
	}	
	  KDL::JntArray G(7);
	ros::Rate r(100);
	file_tau.open (tau_name);

        while( ros::ok() ) 
	{
		
		tau_m = trafoClient->command(q);

		file_tau  << tau.transpose() << std::endl;
		_dyn_param->JntToGravity(*_q_in, G);
		tau=tau_m+G.data;
		std::cout <<"tau: " <<tau.transpose() << std::endl;
		for(int i=0; i<7; i++ ) 
		{
			cmd[i].data = tau(i);
 
		}
		
		for(int i=0; i<7; i++ ) 
		{
			torque_pub[i].publish( cmd[i] );
		}


		r.sleep();

	}

	file_tau.close();

}



int main(int argc, char** argv) 
{

	ros::init(argc, argv, "adaptive_kdl_pinocchio");
	IIWA_SIM simul;
	simul.run();
	return 0;
}





