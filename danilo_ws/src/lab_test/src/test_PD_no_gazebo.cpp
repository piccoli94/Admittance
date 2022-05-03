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
using namespace std;
class IIWA_SIM 
{
	public:
		IIWA_SIM();
		bool init_robot_model();
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
		string tau_name="/home/danilo/catkin_ws/src/lab_test/src/Recording/tau.txt";
		std::ofstream file_tau;
		Eigen::Matrix<double,7,1> q;
		Eigen::Matrix<double,7,1> tau;;
		SlidingModeClient *trafoClient;
		KDL::ChainDynParam *_dyn_param;
		KDL::JntArray *_q_in; 
		bool first=false;
		Eigen::Matrix<double,7,1> q_initial;
		KDL::JntArray *_dq_in;
		Matrix<double ,7,1> dq_old;
		Matrix<double ,7,1> dq;
		Matrix<double ,7,1> ddq;
		double ts=0.005;
		//double* tau_m;
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



IIWA_SIM::IIWA_SIM() 
{

	if (!init_robot_model()) exit(1); 
	ROS_INFO("Robot tree correctly loaded from parameter server!");
	cout << "Joints and segments: " << iiwa_tree.getNrOfJoints() << " - " << iiwa_tree.getNrOfSegments() << endl;
}





void IIWA_SIM::ctrl_loop() 
{
	if( !first )
	{
		q << 0,0,0,-1.57,0,+1.57,0;
		cout << "q: " << q.transpose() << endl;
		dq <<0,0,0,0,0,0,0;
		ddq<<0,0,0,0,0,0,0;

		for(int i=0;i<7;i++)
		{
			_q_in->data[i]=q(i);
			_dq_in->data[i]=dq(i);
		}
		first=true;
	}	
	  KDL::JntArray G(7);
	KDL::JntArray C(7);
	KDL::JntSpaceInertiaMatrix B;
	B.resize(_k_chain.getNrOfJoints());
	file_tau.open (tau_name);
	int count=0;
        while( count < 10000) 
	{
		cout << "q: " << q.transpose() << endl;
		tau = trafoClient->command(q);
		_dyn_param->JntToGravity(*_q_in, G);
		_dyn_param->JntToMass(*_q_in, B);
		_dyn_param->JntToCoriolis(*_q_in, *_dq_in, C);

		file_tau  << tau.transpose() << std::endl;
		tau=tau+G.data;
		std::cout << "tau: " << tau.transpose() << std::endl;
		ddq= B.data.inverse()*(tau-G.data-C.data);
		dq=dq+ddq*ts;
		q=q+dq*ts;
		for(int i=0;i<7;i++)
		{
			_q_in->data[i]=q(i);
			_dq_in->data[i]=dq(i);
		}
	count++;
	}

	file_tau.close();

}



int main(int argc, char** argv) 
{

	ros::init(argc, argv, "simul_kdl_pinocchio_no_gazebo");
	IIWA_SIM simul;
	simul.init_robot_model();

	simul.ctrl_loop();
	return 0;
}





