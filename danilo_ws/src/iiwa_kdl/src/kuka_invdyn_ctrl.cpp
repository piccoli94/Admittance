#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chaindynparam.hpp>

using namespace std;


class KUKA_INVDYN {
	public:
		KUKA_INVDYN();
		void run();
		bool init_robot_model();

		void joint_states_cb( sensor_msgs::JointState );
		void ctrl_loop();


	private:
		ros::NodeHandle _nh;
		KDL::Tree iiwa_tree;
	
		KDL::ChainFkSolverPos_recursive *_fksolver; //Forward position solver	
		KDL::ChainIkSolverVel_pinv *_ik_solver_vel;   	//Inverse velocity solver
		KDL::ChainIkSolverPos_NR *_ik_solver_pos;

		KDL::Chain _k_chain;
	
		ros::Subscriber _js_sub;
		ros::Publisher _cartpose_pub;
		KDL::JntArray *_initial_q;
		KDL::JntArray *_q_in;
		KDL::JntArray *_dq_in;
		bool _first_js;
		bool _first_fk;
		ros::Publisher _cmd_pub[7];
		KDL::	Frame _p_out;
		KDL::ChainDynParam *_dyn_param;
};


bool KUKA_INVDYN::init_robot_model() {
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	std::string base_link = "lbr_iiwa_link_0";
	std::string tip_link  = "lbr_iiwa_link_7";
	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) return false;


	_ik_solver_vel = new KDL::ChainIkSolverVel_pinv( _k_chain );
	_ik_solver_pos = new KDL::ChainIkSolverPos_NR( _k_chain, *_fksolver, *_ik_solver_vel, 100, 1e-6 );

	_q_in = new KDL::JntArray( _k_chain.getNrOfJoints() );
	_dq_in = new KDL::JntArray( _k_chain.getNrOfJoints() );
	_initial_q = new KDL::JntArray( _k_chain.getNrOfJoints() );
	_dyn_param = new KDL::ChainDynParam(_k_chain,KDL::Vector(0,0,-9.81));

	return true;
}


KUKA_INVDYN::KUKA_INVDYN() {

	if (!init_robot_model()) exit(1); 
	ROS_INFO("Robot tree correctly loaded from parameter server!");

	cout << "Joints and segments: " << iiwa_tree.getNrOfJoints() << " - " << iiwa_tree.getNrOfSegments() << endl;
 
	_js_sub = _nh.subscribe("/lbr_iiwa/joint_states", 0, &KUKA_INVDYN::joint_states_cb, this);
	
	
	_cmd_pub[0] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_1_effort_controller/command", 0);
	_cmd_pub[1] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_2_effort_controller/command", 0);
	_cmd_pub[2] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_3_effort_controller/command", 0);
	_cmd_pub[3] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_4_effort_controller/command", 0);
	_cmd_pub[4] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_5_effort_controller/command", 0);
	_cmd_pub[5] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_6_effort_controller/command", 0);
	_cmd_pub[6] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/lbr_iiwa_joint_7_effort_controller/command", 0);

	_first_js = false;
	_first_fk = false;
}


void KUKA_INVDYN::joint_states_cb( sensor_msgs::JointState js ) {

	for(int i=0; i<7; i++ ) { 
		_q_in->data[i] = js.position[i];
		_dq_in->data[i] = js.velocity[i];
		//if( !_first_js ) _initial_q->data[i] = js.position[i];
	_initial_q->data[0]=0.2;
	_initial_q->data[1]=0.3;
	_initial_q->data[2]=0.4;
	_initial_q->data[3]=1.57;
	_initial_q->data[4]=0.5;
	_initial_q->data[5]=-1.57;
	_initial_q->data[6]=0.6;


}


	_first_js = true;
}



void KUKA_INVDYN::ctrl_loop() {

	std_msgs::Float64 cmd[7];
  KDL::JntArray coriol_(7);
  KDL::JntArray grav_(7);
	

  KDL::JntSpaceInertiaMatrix jsim_;
  jsim_.resize(_k_chain.getNrOfJoints());
	while( !_first_js ) usleep(0.1);
		
	ros::Rate r(250);
	double Kp = 250;
	double Kd = 250;

	while( ros::ok() ) {		

    Eigen::VectorXd e = _initial_q->data - _q_in->data; //Keep initial position

    Eigen::VectorXd de = -_dq_in->data; //Desired velocity: 0
		_dyn_param->JntToMass(*_q_in, jsim_);
		_dyn_param->JntToCoriolis(*_q_in, *_dq_in, coriol_);
    _dyn_param->JntToGravity(*_q_in, grav_);


		Eigen::VectorXd q_out = jsim_.data * (Kd*de + Kp*e ) + coriol_.data + grav_.data;
		//Eigen::VectorXd q_out = grav_.data; + Kp*e - Kd*de;
		for(int i=0; i<7; i++ ) {
			cmd[i].data = q_out(i);
		}
		for(int i=0; i<7; i++ ) {
			_cmd_pub[i].publish( cmd[i] );
		}
	
		cout << "e: " << e.transpose() << endl;
		cout << "tau: " << q_out.transpose() << endl;


		r.sleep();
	}


	/*
	KDL::JntArray q_out(_k_chain.getNrOfJoints());

	for(int i=0; i<9; i++ )
		F_dest.M.data[i] = _p_out.M.data[i];

	std_msgs::Float64 cmd[7];

	*/
}


void KUKA_INVDYN::run() {


	boost::thread ctrl_loop_t ( &KUKA_INVDYN::ctrl_loop, this);
	ros::spin();	

}




int main(int argc, char** argv) {

	ros::init(argc, argv, "iiwa_kdl");
	KUKA_INVDYN ik;
	ik.run();

	return 0;
}
