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
#include "region_controller.cpp"
#include "boost/thread.hpp"
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
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>
#include "ros/ros.h"

using namespace std;
class IIWA_SIM {
	public:
		IIWA_SIM();
		void run();
		bool init_robot_model();
		void joint_states_cb( sensor_msgs::JointState );
		void ctrl_loop();
		//void log_theta();
	private:
   		Eigen::Matrix<double,7,1> q;
    		Eigen::Matrix<double,7,1> dq;
    		Eigen::Matrix<double,7,1> ddq;
    		Eigen::Matrix<double,3,1> xdes;
    		Eigen::Matrix<double,3,1> dxdes;
    		Eigen::Matrix<double,3,1> x_i;
    		Eigen::Matrix<double,7,1>  tau;
    		double a_h;
    		double a_r;
    		double a_t;
    		double alpha;
    		double k_max;
    		double ts;
    		double coef_ks;
    		Eigen::Matrix<double,7,7> K_s;
    		Eigen::Matrix<double, 7, 1> dqr_old;
    		double k_pinvJ;
    		double sat_q;
    		double sat_dq;
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
		Eigen::Matrix<double,7,1> sat_tau;
		string theta_name="/home/danilo/catkin_ws/src/region_controller/src/theta_cap.txt";
		Eigen::Matrix<double,59,1> theta_cap;
    		std::vector<double> y_vec;
    		std::vector<double> dy_vec;
		double coef_l;
		std::vector<double> theta_vec;
		Matrix<double, 59,59> L;
      		const char* delim = "   "; 
    		std::string y_name="/home/danilo/catkin_ws/src/region_controller/src/y.txt";
    		std::string dy_name="/home/danilo/catkin_ws/src/region_controller/src/dy.txt";
		string Delta_name="/home/danilo/catkin_ws/src/region_controller/src/Recording/Delta_x.txt";
		string tau_name="/home/danilo/catkin_ws/src/region_controller/src/Recording/tau.txt";		
int count1, count2;
		double step=0.001;
		std::ofstream file_theta;
		KDL::ChainDynParam *_dyn_param;
		KDL::JntArray *_q_in; //oggetto che costruisce un vettore dei giunti del manipolatore
		std::ofstream file_delta;
		std::ofstream file_tau;
	//-----------------------	

};

bool IIWA_SIM::init_robot_model() {
	a_h=0.0001;
	a_r=2;
	a_t=3;
	alpha=10;
	k_max=200;
	ts=5e-3;
	coef_ks=0.1;
	coef_l=0.1;
	L=Eigen::Matrix<double, 59, 59>::Identity()*coef_l;
	K_s=Eigen::Matrix<double, 7, 7>::Identity()*coef_ks;
	k_pinvJ=0.001;
	//xdes << 0.6,0.2,0.614;
	//dxdes << 0,0,0;
	x_i << 0.4,0.1,0.614;

	sat_tau<<176,176,110,110,110,40,40;
	
//theta_cap << 3.45250000000000,0.0218300000000000,0,0,0.00770300000000000,-0.00388700000000000,0.0208300000000000,0,0.000100000000000000,3.48210000000000,0.0207600000000000,0,-0.00362600000000000,0.0217900000000000,0,0.00779000000000000,0,0.000100000000000000,4.05623000000000,0.0320400000000000,0,0,0.00972000000000000,0.00622700000000000,0.0304200000000000,0,0.000100000000000000,3.48220000000000,0.0217800000000000,0,0,0.0207500000000000,-0.00362500000000000,0.00778500000000000,0,0.000100000000000000,2.16330000000000,0.0128700000000000,0,0,0.00570800000000000,-0.00394600000000000,0.0111200000000000,0,0.000100000000000000,2.34660000000000,0.00650900000000000,0,0,0.00625900000000000,0.000318910000000000,0.00452700000000000,0,0.000100000000000000,3.12900000000000,0.0146400000000000,0.000591200000000000,0,0.0146500000000000,0,0.00287200000000000,0,0.000100000000000000;




	std::string line;
	ifstream y_file;
	ifstream dy_file;
	y_file.open(y_name);
	dy_file.open(dy_name);
    	double y_temp=0;
   	while (std::getline(y_file, line)) {
        	y_temp=atof(line.c_str());
        	y_vec.push_back(y_temp);
    	}
   	while (std::getline(dy_file, line)) {
        	y_temp=atof(line.c_str());
        	dy_vec.push_back(y_temp);
    	}
	
y_file.close();
dy_file.close();


	_nh.param("robot_description", robot_desc_string, std::string());
	if(!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)) ROS_ERROR("Failed to construct kdl tree");
	base_link = "lbr_iiwa_link_0";
	tip_link  = "lbr_iiwa_link_7";
	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) ROS_ERROR("robot not init");

	controller= new REGION_CONTROLLER(x_i, a_h, a_r, a_t, alpha, k_max, ts, K_s, L, k_pinvJ);


	_q_in = new KDL::JntArray( _k_chain.getNrOfJoints() );
	_dyn_param = new KDL::ChainDynParam(_k_chain,KDL::Vector(0,0,-9.81));



	return true;
}


void IIWA_SIM::joint_states_cb( sensor_msgs::JointState js ) {

	for(int i=0; i<7; i++ ) { 
		q(i) = js.position[i];
		dq(i) = js.velocity[i];
		_q_in->data[i]=js.position[i];

	}

}


IIWA_SIM::IIWA_SIM() {

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

}

void IIWA_SIM::run() {


	boost::thread ctrl_loop_t ( &IIWA_SIM::ctrl_loop, this);
	ros::spin();	

}



void IIWA_SIM::ctrl_loop() {
		
	ros::Rate r(200);
	double phi=0.0;

	count1=0;
	file_tau.open(tau_name);
	KDL::JntArray G(7);
        while( ros::ok() ) {
	
		xdes << 0.4,y_vec[count1],0.614;
		dxdes << 0.0,dy_vec[count1],0.0;
		count1++;
		if( count1>1998) count1=1999;
		_dyn_param->JntToGravity(*_q_in, G);//c'era già prima
		controller->torque_control(q,dq,xdes,dxdes,theta_cap,dqr_old);
		tau=controller->get_tau_m() + G.data;

		cout << "G:" << G.data.transpose()<< endl;
		
    		dqr_old=controller->get_qr_dot();
		file_theta.open(theta_name);

		theta_cap=controller->get_theta();
		file_theta << theta_cap;
		file_theta.close();
		for(int i=0; i<7; i++ ) {
			if( tau(i) > sat_tau(i) ) tau(i) = sat_tau(i);
			else if( tau(i) < -sat_tau(i) ) tau(i) = -sat_tau(i);
			cmd[i].data = tau(i);
		}
		for(int i=0; i<7; i++ ) {
			torque_pub[i].publish( cmd[i] );
		}	
		file_tau << tau.transpose() << endl;


		//log_theta();

		r.sleep();

	}
file_tau.close();
}





int main(int argc, char** argv) {

	ros::init(argc, argv, "simul_kdl_pinocchio");
	IIWA_SIM simul;
	simul.run();
	return 0;
}





