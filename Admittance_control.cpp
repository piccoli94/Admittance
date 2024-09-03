#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

using namespace std;
using namespace Eigen;	

class KUKA_INVKIN {
	public:
		KUKA_INVKIN();
		void run();
		bool init_robot_model();
		void get_dirkin();
		void joint_states_cb( sensor_msgs::JointState );
		void ctrl_loop();
		void goto_initial_position( float dp[7] );

	private:
		ros::NodeHandle _nh;
		KDL::Tree iiwa_tree;
	
		KDL::ChainFkSolverPos_recursive *_fksolver; //Forward position solver	
		KDL::ChainIkSolverVel_pinv *_ik_solver_vel;   	//Inverse velocity solver
		KDL::ChainIkSolverPos_NR *_ik_solver_pos;

		KDL::Chain _k_chain;
	
		ros::Subscriber _js_sub;
		ros::Publisher _cartpose_pub;
		KDL::JntArray *_q_in;
		KDL::JntArray *_q_Q;

		bool _first_js;
		bool _first_fk;
		ros::Publisher _cmd_pub[7];
		KDL::	Frame _p_out;
		KDL::	Frame _pq_out;
		geometry_msgs::Pose cqpose;

		Matrix<double,3,3> M, D, K;
		Matrix<double,3,1> x,xd,xdd,fh,fd,x_des,error,error_old,x_des_k,x_des_dot;
		geometry_msgs::Pose cpose;
		double f_p,a,k_1,a_h,a_r,kp;
		double gamma,beta,sigma;
		Matrix<double,2,1> f_2d,e_2d;

};


bool KUKA_INVKIN::init_robot_model() {
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	std::string base_link = "lbr_iiwa_link_0";
	std::string tip_link  = "lbr_iiwa_link_7";
	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) return false;
	_fksolver = new KDL::ChainFkSolverPos_recursive( _k_chain );

	_ik_solver_vel = new KDL::ChainIkSolverVel_pinv( _k_chain );
	_ik_solver_pos = new KDL::ChainIkSolverPos_NR( _k_chain, *_fksolver, *_ik_solver_vel, 100, 1e-6 );

	_q_in = new KDL::JntArray( _k_chain.getNrOfJoints() );
	_q_Q = new KDL::JntArray( _k_chain.getNrOfJoints() );
	fh << 5,0,0;
	x_des << 0.5, 0.0, 0.614;
	M=Matrix<double,3,3>::Identity()*10;
	D=Matrix<double,3,3>::Identity()*15;
	K=Matrix<double,3,3>::Identity()*50;
	a_r=0.2;
	a_h=0.1;
	a=100;
	k_1=500;
	beta=M_PI/6;
	sigma=M_PI/4;
	return true;
}


KUKA_INVKIN::KUKA_INVKIN() {

	if (!init_robot_model()) exit(1); 
	ROS_INFO("Robot tree correctly loaded from parameter server!");

	cout << "Joints and segments: " << iiwa_tree.getNrOfJoints() << " - " << iiwa_tree.getNrOfSegments() << endl;
 
	_cartpose_pub = _nh.advertise<geometry_msgs::Pose>("/lbr_iiwa/eef_pose", 0);
	_js_sub = _nh.subscribe("/lbr_iiwa/joint_states", 0, &KUKA_INVKIN::joint_states_cb, this);
	
	_cmd_pub[0] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint1_position_controller/command", 0);
	_cmd_pub[1] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint2_position_controller/command", 0);
	_cmd_pub[2] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint3_position_controller/command", 0);
	_cmd_pub[3] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint4_position_controller/command", 0);
	_cmd_pub[4] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint5_position_controller/command", 0);
	_cmd_pub[5] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint6_position_controller/command", 0);
	_cmd_pub[6] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint7_position_controller/command", 0);

	_first_js = false;
	_first_fk = false;
}


void KUKA_INVKIN::joint_states_cb( sensor_msgs::JointState js ) {

	for(int i=0; i<7; i++ ) 
		_q_in->data[i] = js.position[i];

	_first_js = true;
}


void KUKA_INVKIN::goto_initial_position( float dp[7] ) {
	
	ros::Rate r(10);
	float min_e = 1000;

	std_msgs::Float64 cmd[7];

	float max_e = 1000;
	while( max_e > 0.002 ) {
 		max_e = -1000;
		for(int i=0; i<7; i++) {
 			cmd[i].data = dp[i];
			_cmd_pub[i].publish (cmd[i]);
			float e = fabs( cmd[i].data - _q_in->data[i] );
			max_e = ( e > max_e ) ? e : max_e;
			//cout << fabs( cmd[i].data - _q_in->data[i] ) << endl;
		}
		r.sleep();
	}

	sleep(2);
}


void KUKA_INVKIN::get_dirkin() {

	ros::Rate r(50);

	KDL::JntArray q_curr(_k_chain.getNrOfJoints());


	while( !_first_js ) usleep(0.1);


	while(ros::ok()) {


		_fksolver->JntToCart(*_q_in, _p_out);

		cpose.position.x = _p_out.p.x();
		cpose.position.y = _p_out.p.y();
		cpose.position.z = _p_out.p.z();

		double qx, qy, qz, qw;
		_p_out.M.GetQuaternion( qx, qy, qz, qw);
		cpose.orientation.w = qw;
		cpose.orientation.x = qx;
		cpose.orientation.y = qy;
		cpose.orientation.x = qz;

		_cartpose_pub.publish( cpose );		
		_first_fk = true;
	
		r.sleep();
	}
}



void KUKA_INVKIN::ctrl_loop() {

	std_msgs::Float64 d;
	

	while( !_first_fk ) usleep(0.1);
	
	float i_cmd[7];
	i_cmd[0] = 0.0;
	i_cmd[1] = i_cmd[2] = i_cmd[4] = i_cmd[6] = 0.0;
	i_cmd[3] = -1.57;
	i_cmd[5] = 1.57;
	goto_initial_position( i_cmd );

	for(int i=0;i<7;i++)
	{
		_q_Q->data[i] = i_cmd[i];
	}
	_fksolver->JntToCart(*_q_Q, _pq_out);
		cqpose.position.x = _pq_out.p.x();
		cqpose.position.y = _pq_out.p.y();
		cqpose.position.z = _pq_out.p.z();

		double qx, qy, qz, qw;
		_pq_out.M.GetQuaternion( qx, qy, qz, qw);
		cqpose.orientation.w = qw;
		cqpose.orientation.x = qx;
		cqpose.orientation.y = qy;
		cqpose.orientation.x = qz;
	
	ros::Rate r(50);

	KDL::Frame F_dest;
	KDL::JntArray q_out(_k_chain.getNrOfJoints());
/*

	F_dest.p.data[0] = _p_out.p.x() - 0.2;
	F_dest.p.data[1] = _p_out.p.y();
	F_dest.p.data[2] = _p_out.p.z() - 0.1;

	for(int i=0; i<9; i++ )
		F_dest.M.data[i] = _p_out.M.data[i];
*/

	x(0)=cpose.position.x;
	x(1)=cpose.position.y;
	x(2)=cpose.position.z;

	xd(0)=0;
	xd(1)=0;
	xd(2)=0;

	xdd(0)=0;
	xdd(1)=0;
	xdd(2)=0;

	x_des_k = x;
	x_des.setZero();
	x_des_dot << 0.01,0,0;
	std_msgs::Float64 cmd[7];
	while( ros::ok() ) {		
		fh<<0.5,1.5,0;
		error=x-x_des_k;

		f_2d(0)=fh(0);
		f_2d(1)=fh(1);
		e_2d(0)=x_des_dot(0);
		e_2d(1)=x_des_dot(1);

		if( ( e_2d.norm(  ) * f_2d.norm(  ) ) == 0 ) gamma= 0;
		else gamma= acos( (  e_2d(0) * f_2d(0) + e_2d(1) * f_2d(1) ) / ( e_2d.norm(  ) * f_2d.norm(  ) ) );

		if(gamma <= beta) 
		{
			fd << 0,0,fh(2);
		}
		else if(gamma > beta && gamma <= beta + sigma )
		{
		/*	c_x=optiman_direction(f_2d,e_2d);
			if( norm(c_x) != 0 )    c_x_unit = c_x / norm(c_x);
			else c_x_unit= makeVector( 0, 0, 0 );
			C_x[0]= -c_x_unit[0] * norm(f_2d) * pow(cos( M_PI*( gamma - beta ) / ( 2 * sigma ) ), 2 );
			C_x[1]= -c_x_unit[1] * norm(f_2d) * pow(cos( M_PI*( gamma - beta ) / ( 2 * sigma ) ), 2 );		
			C_x[2] = fm[2];
			*/
			fd << fh(0)*(1-cos( M_PI*(gamma-beta)/(2*sigma)) ),fh(1)*(1-cos(M_PI*(gamma-beta)/(2*sigma)) ),fh(2);
		}
		else
		{
			fd << fh(0),fh(1),fh(2);
		}



		if(error.norm()<=a_h) {kp=0;}
		else if(error.norm()<=a_r && error.norm() >a_h  )
		{
			f_p=1 - exp( - a * ( pow ( error.norm() , 2 ) - pow ( a_h ,2 ) ) );
        	kp= k_1 * pow( std::max( f_p , 0.0 ), 2 )  *  exp( - a * ( pow ( error.norm() , 2 ) - pow ( a_h ,2 ) ) );
		}
		else {kp=0;}
		K(0,0)=kp;
		K(1,1)=kp;
		K(2,2)=kp;


		if(error_old.norm()<a_h && error.norm()>a_h) x_des=x;


		xdd= M.inverse()*( (fh-fd) -D*xd  - K*( x-x_des) );
		xd= xd + xdd/50;
		x= x+ xd/50;

		error_old=error;
		cout << "K(e)= " << K(0,0) << endl;
		cout << "error= " << error.norm() << endl;
		cout << "x_des= " << x_des.transpose() << endl; 
		cout << "gamma= " << gamma << endl;
		cout << "fh= " << fh.transpose() << endl; 
		cout << "fd= " << fd.transpose() << endl; 



	//	if(x_des(0)<0.5) x_des(0)+= 0.01;
		F_dest.p.data[0]=x(0);
		F_dest.p.data[1]=x(1);
		F_dest.p.data[2]=x(2);
		for(int i=0; i<9; i++ )		F_dest.M.data[i] = _pq_out.M.data[i];

		if( _ik_solver_pos->CartToJnt(*_q_in, F_dest, q_out) != KDL::SolverI::E_NOERROR ) 
			cout << "failing in ik!" << endl;

		for(int i=0; i<7; i++) {
			cmd[i].data = q_out.data[i];
		}
		for(int i=0; i<7; i++) {
			_cmd_pub[i].publish (cmd[i]);
		}
		cout << "x y z: " << x.transpose() << endl;
	/*	cout<<"q: ";
		for(int i=0;i<7;i++)
		{
			cout<< q_out.data[i]<< " ";
		}
		cout << endl;
	*/	r.sleep();
	}


}


void KUKA_INVKIN::run() {

	boost::thread get_dirkin_t( &KUKA_INVKIN::get_dirkin, this);
	boost::thread ctrl_loop_t ( &KUKA_INVKIN::ctrl_loop, this);
	ros::spin();	

}




int main(int argc, char** argv) {

	ros::init(argc, argv, "iiwa_kdl");
	KUKA_INVKIN ik;
	ik.run();

	return 0;
}
