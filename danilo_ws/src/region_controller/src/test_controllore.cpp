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
#include "region_controller_without_reg.cpp"
#include "boost/thread.hpp"




class IIWA_SIM {
	public:
		IIWA_SIM();
		void run();
		bool init_robot_model();
		void joint_states_cb( sensor_msgs::JointState );
		void ctrl_loop();
		//void log_theta();
		void tokenize(std::string const &str, const char* delim, std::vector<std::string> &out) ;
		void take_desire_motion(const char* delim, std::string f_name,std::vector<double> &xdes, std::vector<double> &ydes, std::vector<double> &zdes);

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
		string Delta_name="/home/danilo/catkin_ws/src/region_controller/src/Recording/Delta_x.txt";
		string tau_name="/home/danilo/catkin_ws/src/region_controller/src/Recording/tau.txt";
		//string q_name="/home/danilo/catkin_ws/src/region_controller/src/Recording/q.txt";
		std::vector<double> x_vec;
    		std::vector<double> y_vec;
    		std::vector<double> z_vec;
    		std::vector<double> dx_vec;
    		std::vector<double> dy_vec;
    		std::vector<double> dz_vec;

      		const char* delim = "   "; 
    		std::string pos_name="/home/danilo/catkin_ws/src/region_controller/src/des_pos.txt";
    		std::string vel_name="/home/danilo/catkin_ws/src/region_controller/src/des_vel.txt";
		int count1, count2;
		double norm_Delta_temp;
	 	std::ofstream file_delta;
		std::ofstream file_tau;
		//std::ofstream file_q;




	//-----------------------	

};

bool IIWA_SIM::init_robot_model() {
	a_h=0.0001;
	a_r=0.25;
	a_t=0.5;
	alpha=0.5;
	k_max=20;
	ts=4e-3;
	coef_ks=1;
	K_s=Eigen::Matrix<double, 7, 7>::Identity()*coef_ks;
	k_pinvJ=0.001;
	xdes << 0.6,0.2,0.614;
	dxdes << 0,0,0;
	x_i << 0.4,0.1,0.614;

	sat_tau<<176,176,110,110,110,40,40;
	
//theta_cap << 3.45250000000000,0.0218300000000000,0,0,0.00770300000000000,-0.00388700000000000,0.0208300000000000,0,0.000100000000000000,3.48210000000000,0.0207600000000000,0,-0.00362600000000000,0.0217900000000000,0,0.00779000000000000,0,0.000100000000000000,4.05623000000000,0.0320400000000000,0,0,0.00972000000000000,0.00622700000000000,0.0304200000000000,0,0.000100000000000000,3.48220000000000,0.0217800000000000,0,0,0.0207500000000000,-0.00362500000000000,0.00778500000000000,0,0.000100000000000000,2.16330000000000,0.0128700000000000,0,0,0.00570800000000000,-0.00394600000000000,0.0111200000000000,0,0.000100000000000000,2.34660000000000,0.00650900000000000,0,0,0.00625900000000000,0.000318910000000000,0.00452700000000000,0,0.000100000000000000,3.12900000000000,0.0146400000000000,0.000591200000000000,0,0.0146500000000000,0,0.00287200000000000,0,0.000100000000000000;




/*	std::string line;
	ifstream input;
	input.open(theta_name);
 if (!input.is_open())
    {
       cout << "File cannot be found." << endl;
       system("pause");
       exit(EXIT_FAILURE);
}
    	double theta=0;
   	while (std::getline(input, line)) {
        	theta=atof(line.c_str());
        	theta_vec.push_back(theta);
    	}
    	int j=0;
    	for(j=0;j<59;j++){
        	theta_cap(j)=theta_vec[j];
	}
	
input.close();
std::cout << "theta: " << theta_cap << std::endl;
*/

	_nh.param("robot_description", robot_desc_string, std::string());
	if(!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)) ROS_ERROR("Failed to construct kdl tree");
	base_link = "lbr_iiwa_link_0";
	tip_link  = "lbr_iiwa_link_7";
	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) ROS_ERROR("robot not init");

	controller= new REGION_CONTROLLER(x_i, a_h, a_r, a_t, alpha, k_max, ts, K_s, k_pinvJ,_k_chain);

    	take_desire_motion(delim,pos_name,x_vec,y_vec,z_vec);
    	take_desire_motion(delim,vel_name,dx_vec,dy_vec,dz_vec);





	return true;
}


void IIWA_SIM::joint_states_cb( sensor_msgs::JointState js ) {

	for(int i=0; i<7; i++ ) { 
		q(i) = js.position[i];
		dq(i) = js.velocity[i];


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
		
	ros::Rate r(250);
	file_delta.open (Delta_name);
	file_tau.open (tau_name);
	//file_q.open (q_name);

        while( ros::ok() ) {
		//std::cout << "q" << q.transpose() << std::endl;
		//std::cout << "dq" << dq.transpose() << std::endl;		
		xdes << x_vec[count1] , y_vec[count1], z_vec[count1];
    	dxdes << 0 , 0, 0;
	
		count1++;
		//if(count1 == x_vec.size()-1){ count1=0;}
		controller->torque_control(q,dq,xdes,dxdes,dqr_old);
		tau=controller->get_tau_m();
		norm_Delta_temp=controller->get_norm_Delta_x();
		file_delta << norm_Delta_temp << std::endl;
		//file_q << q.transpose() << std::endl;
		file_tau <<std::fixed << std::setprecision(5)<< q.transpose() << std::endl;
    		dqr_old=controller->get_qr_dot();

		for(int i=0; i<7; i++ ) {
			if( tau(i) > sat_tau(i) ) tau(i) = sat_tau(i);
			else if( tau(i) < -sat_tau(i) ) tau(i) = -sat_tau(i);
			cmd[i].data = tau(i);
		}
		
		
		for(int i=0; i<7; i++ ) {
			torque_pub[i].publish( cmd[i] );
		}

		//log_theta();
		r.sleep();

	}
	file_delta.close();
	file_tau.close();
	//file_q.close();
}

void IIWA_SIM::take_desire_motion(const char* delim,std::string f_name, std::vector<double> &xdes,std::vector<double> &ydes,std::vector<double> &zdes)
{
  
    std::string line;
    std::ifstream infile(f_name);
        while (std::getline(infile, line)) {
        std::vector<std::string> out; 
        tokenize(line, delim, out); 
        double x,y,z;
        x=atof(out[0].c_str());
        y=atof(out[1].c_str());
        z=atof(out[2].c_str());
        xdes.push_back(x);
        ydes.push_back(y);
        zdes.push_back(z);

    }

}

void IIWA_SIM::tokenize(std::string const &str, const char* delim, 
            std::vector<std::string> &out) 
{ 

    char *token = strtok(const_cast<char*>(str.c_str()), delim); 
    while (token != nullptr) 
    { 
        out.push_back(std::string(token)); 
        token = strtok(nullptr, delim); 
    }
} 


int main(int argc, char** argv) {

	ros::init(argc, argv, "simul_kdl_pinocchio");
	IIWA_SIM simul;
	simul.run();
	return 0;
}





